/*
 * Host-side check of the driver against a fake ICM-45686. No hardware, no ESP
 * toolchain -- see test/Makefile.
 *
 * The fake models the two things that are easy to get wrong and hard to
 * notice: the read bit lives in the top of the address byte, and the sample
 * bytes only assemble correctly if the driver and the device agree on byte
 * order. Both are checked before anything else.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "icm45686.h"
#include "icm45686_math.h"
#include "icm45686_regs.h"
#include "imu.h"

#define G		9.80665f
#define DEG		0.017453292519943295f

static uint8_t regs[256];
static uint16_t ireg_addr;
static uint8_t ireg[0x10000];
static int transfers, reset_count, longest_read, added, removed;
static esp_err_t bus_status;
static int fail_write_in;	/* fail the Nth write from now; -1: never */
static TickType_t delays[8];
static int ndelays;

static void chip_reset(void)
{
	memset(regs, 0, sizeof(regs));
	memset(ireg, 0, sizeof(ireg));
	regs[ICM_WHO_AM_I] = ICM_WHO_AM_I_VALUE;
	regs[ICM_REG_MISC2] = ICM_IREG_DONE;	/* indirect access idle */
	transfers = reset_count = longest_read = added = removed = 0;
	ndelays = 0;
	bus_status = ESP_OK;
	fail_write_in = -1;
}

esp_err_t spi_bus_add_device(spi_host_device_t host,
			     const spi_device_interface_config_t *cfg,
			     spi_device_handle_t *out)
{
	(void)host;
	assert(cfg->clock_speed_hz > 0 && cfg->clock_speed_hz <= 24000000);
	assert(cfg->mode == 3);
	*out = (spi_device_handle_t)(void *)regs;
	added++;
	return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t d)
{
	(void)d;
	removed++;
	return ESP_OK;
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t dev, spi_transaction_t *t)
{
	const uint8_t *tx = t->tx_buffer;
	uint8_t *rx = t->rx_buffer;
	size_t n = t->length / 8;
	uint8_t addr = tx[0] & 0x7F;
	bool read = (tx[0] & ICM_SPI_READ) != 0;
	uint8_t v;

	(void)dev;
	if (bus_status != ESP_OK)
		return bus_status;
	transfers++;

	if (read) {
		assert(rx && "a read must supply a receive buffer");
		if ((int)(n - 1) > longest_read)
			longest_read = (int)(n - 1);
		rx[0] = 0;
		for (size_t i = 1; i < n; i++)
			rx[i] = regs[(addr + i - 1) & 0xFF];
		return ESP_OK;
	}

	if (fail_write_in >= 0 && fail_write_in-- == 0)
		return ESP_ERR_TIMEOUT;
	v = tx[1];
	if (addr == ICM_REG_MISC2 && (v & ICM_SOFT_RST)) {
		uint8_t who = regs[ICM_WHO_AM_I];	/* survives a reset */

		memset(regs, 0, sizeof(regs));
		regs[ICM_WHO_AM_I] = who;
		regs[ICM_REG_MISC2] = ICM_IREG_DONE;
		reset_count++;
		return ESP_OK;
	}
	regs[addr] = v;
	if (addr == ICM_IREG_ADDR_15_8)
		ireg_addr = (uint16_t)((ireg_addr & 0x00FF) |
				       ((uint16_t)v << 8));
	else if (addr == ICM_IREG_ADDR_7_0)
		ireg_addr = (uint16_t)((ireg_addr & 0xFF00) | v);
	else if (addr == ICM_IREG_DATA)
		ireg[ireg_addr] = v;
	return ESP_OK;
}

/* Counts the drivers' calloc() calls; linked with -Wl,--wrap=calloc. */
static int allocs;
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
}

void vTaskDelay(TickType_t t)
{
	if (ndelays < 8)
		delays[ndelays++] = t;
}

/* ------------------------------------------------------------ helpers ---- */

static struct icm45686_config base_cfg(void)
{
	struct icm45686_config c = {
		.host = SPI2_HOST,
		.cs_gpio = 5,
		.accel_fs = ICM45686_ACCEL_FS_4G,
		.gyro_fs = ICM45686_GYRO_FS_500DPS,
		.odr = ICM45686_ODR_200HZ,
	};

	return c;
}

static struct imu_dev open_ok(void)
{
	struct icm45686_config c = base_cfg();
	struct imu_dev imu = { 0 };

	chip_reset();
	assert(icm45686_probe(&imu, &c) == 0);
	assert(imu.priv && imu.ops);
	return imu;
}

/* put a little-endian sample into the data registers */
static void put_sample(int off, int16_t v)
{
	regs[off] = (uint8_t)(v & 0xFF);
	regs[off + 1] = (uint8_t)((uint16_t)v >> 8);
}

/* -------------------------------------------------------------- tests ---- */

static void test_probe_and_reset(void)
{
	struct icm45686_config c = base_cfg();
	struct imu_dev imu = { 0 };

	chip_reset();
	imu.mount[2][2] = -1;			/* probe must not touch it */
	assert(icm45686_probe(&imu, &c) == 0);
	assert(!strcmp(imu.name, "icm45686"));
	assert(imu.mount[2][2] == -1);
	assert(reset_count == 1 && "probe must soft reset before trusting");
	icm45686_remove(&imu);
	assert(!imu.priv && !imu.ops);

	/* A part that answers with the wrong identity is not this part. */
	chip_reset();
	regs[ICM_WHO_AM_I] = 0x47;
	assert(icm45686_probe(&imu, &c) == -ENODEV);
	assert(strstr(stub_log_last, "E imu: icm45686: "));
	assert(strstr(stub_log_last, "WHO_AM_I=0x47, expected 0xe9"));
	assert(!imu.priv && !imu.ops);
	assert(added == 1 && removed == 1 && "nothing left on the bus");

	chip_reset();
	regs[ICM_WHO_AM_I] = 0x00;		/* bus stuck low */
	assert(icm45686_probe(&imu, &c) == -ENODEV);
	chip_reset();
	regs[ICM_WHO_AM_I] = 0xFF;		/* bus stuck high / no device */
	assert(icm45686_probe(&imu, &c) == -ENODEV);

	/* A failing transfer comes back as its errno. */
	chip_reset();
	bus_status = ESP_ERR_TIMEOUT;
	assert(icm45686_probe(&imu, &c) == -ETIMEDOUT);
	assert(added == 1 && removed == 1);
}

/*
 * The ~1 ms soft-reset settle is asked for as 2 ms. At the stub's 100 Hz tick
 * pdMS_TO_TICKS(2) is 0, and vTaskDelay(1) can return almost at once, so only
 * 2 ticks or more is a real wait.
 */
static void test_reset_wait_survives_the_tick(void)
{
	struct imu_dev imu = open_ok();

	/* The fake reports IREG_DONE at once, so the reset is the only wait. */
	assert(ndelays == 1);
	assert(delays[0] >= 2);
	icm45686_remove(&imu);
}

static void test_config_encoding(void)
{
	struct imu_dev imu = open_ok();

	/* 4 g is code 3, 200 Hz is code 8 -> 0x38 */
	assert(regs[ICM_ACCEL_CONFIG0] ==
	       ((ICM45686_ACCEL_FS_4G << 4) | ICM45686_ODR_200HZ));
	/* 500 dps is code 3, same rate */
	assert(regs[ICM_GYRO_CONFIG0] ==
	       ((ICM45686_GYRO_FS_500DPS << 4) | ICM45686_ODR_200HZ));
	/* both sensors in low-noise mode: accel bits 1:0, gyro bits 3:2 */
	assert(regs[ICM_PWR_MGMT0] ==
	       ((ICM_MODE_LN << 0) | (ICM_MODE_LN << 2)));

	/* Endianness pinned through indirect access, not left to chance. */
	assert(ireg[ICM_SREG_CTRL] ==
	       (ICM_SREG_LITTLE_ENDIAN << ICM_SREG_ENDIAN_SHIFT));

	assert(icm45686_set_ranges(&imu, ICM45686_ACCEL_FS_32G,
				   ICM45686_GYRO_FS_15_625DPS) == 0);
	assert(regs[ICM_ACCEL_CONFIG0] == ((0 << 4) | ICM45686_ODR_200HZ));
	assert(regs[ICM_GYRO_CONFIG0] == ((8 << 4) | ICM45686_ODR_200HZ));

	assert(icm45686_set_odr(&imu, ICM45686_ODR_6400HZ) == 0);
	assert((regs[ICM_ACCEL_CONFIG0] & 0x0F) == ICM45686_ODR_6400HZ);
	assert((regs[ICM_GYRO_CONFIG0] & 0x0F) == ICM45686_ODR_6400HZ);

	/* Codes outside the enums are refused rather than written. */
	assert(icm45686_set_ranges(&imu, (enum icm45686_accel_fs)5,
				   ICM45686_GYRO_FS_500DPS) == -EINVAL);
	assert(icm45686_set_ranges(&imu, ICM45686_ACCEL_FS_4G,
				   (enum icm45686_gyro_fs)9) == -EINVAL);
	assert(icm45686_set_odr(&imu, (enum icm45686_odr)0) == -EINVAL);
	assert(icm45686_set_odr(&imu, (enum icm45686_odr)2) == -EINVAL);

	icm45686_remove(&imu);
}

static void test_power(void)
{
	struct imu_dev imu = open_ok();

	assert(icm45686_set_power(&imu, true, false) == 0);
	assert((regs[ICM_PWR_MGMT0] & 0x03) == ICM_MODE_LN);
	assert(((regs[ICM_PWR_MGMT0] >> 2) & 0x03) == ICM_MODE_OFF);

	assert(icm45686_set_power(&imu, false, true) == 0);
	assert((regs[ICM_PWR_MGMT0] & 0x03) == ICM_MODE_OFF);
	assert(((regs[ICM_PWR_MGMT0] >> 2) & 0x03) == ICM_MODE_LN);

	/* Tearing down must not leave the sensors drawing current. */
	icm45686_remove(&imu);
	assert(regs[ICM_PWR_MGMT0] == 0);
}

static void test_burst_read(void)
{
	struct imu_dev imu = open_ok();
	int16_t a[3], g[3], t;

	put_sample(0x00, 1000);
	put_sample(0x02, -2000);
	put_sample(0x04, 16384);
	put_sample(0x06, -1);
	put_sample(0x08, 32767);
	put_sample(0x0A, -32768);
	put_sample(0x0C, 13248);

	transfers = 0;
	longest_read = 0;
	assert(icm45686_read_raw(&imu, a, g, &t) == 0);

	/*
	 * One transaction, covering all fourteen bytes -- accel and gyro have
	 * to come from the same sample instant.
	 */
	assert(transfers == 1);
	assert(longest_read == ICM_DATA_BURST_LEN);

	assert(a[0] == 1000 && a[1] == -2000 && a[2] == 16384);
	assert(g[0] == -1 && g[1] == 32767 && g[2] == -32768);
	assert(t == 13248);

	/* temp is optional */
	assert(icm45686_read_raw(&imu, a, g, NULL) == 0);

	icm45686_remove(&imu);
}

static void test_conversion(void)
{
	struct icm45686_config c = base_cfg();
	struct imu_dev imu = { 0 };
	struct imu_sample s;

	c.accel_fs = ICM45686_ACCEL_FS_2G;	/* 16384 counts per g */
	c.gyro_fs = ICM45686_GYRO_FS_250DPS;	/* 131.072 counts per dps */
	chip_reset();
	assert(icm45686_probe(&imu, &c) == 0);

	put_sample(0x00, 16384);
	put_sample(0x02, -16384);
	put_sample(0x04, 0);
	put_sample(0x06, 131);
	put_sample(0x08, -131);
	put_sample(0x0A, 0);
	put_sample(0x0C, 0);

	/* Through the core, in SI. */
	transfers = 0;
	assert(imu_read(&imu, &s) == 0);
	assert(transfers == 1 && "imu_read is still one burst");
	assert(fabs(s.accel[0] - G) < 1e-4);
	assert(fabs(s.accel[1] + G) < 1e-4);
	assert(s.accel[2] == 0.0f);
	assert(fabs(s.gyro[0] - 131.0f / 131.072f * DEG) < 1e-6);
	assert(fabs(s.gyro[1] + 131.0f / 131.072f * DEG) < 1e-6);
	assert(fabs(s.temp_c - 25.0f) < 1e-4);

	assert(fabs(icm45686_accel_sensitivity(&imu) - 16384.0f) < 1e-3);
	assert(fabs(icm45686_gyro_sensitivity(&imu) - 131.072f) < 1e-3);

	/* Widening the range must change what the same counts mean. */
	assert(icm45686_set_ranges(&imu, ICM45686_ACCEL_FS_4G,
				   ICM45686_GYRO_FS_250DPS) == 0);
	assert(imu_read(&imu, &s) == 0);
	assert(fabs(s.accel[0] - 2.0f * G) < 1e-4);

	icm45686_remove(&imu);
}

static void test_data_ready(void)
{
	struct imu_dev imu = open_ok();
	bool ready = true;

	regs[ICM_INT1_STATUS0] = 0;
	assert(icm45686_data_ready(&imu, &ready) == 0 && !ready);

	regs[ICM_INT1_STATUS0] = ICM_INT_DRDY;
	assert(icm45686_data_ready(&imu, &ready) == 0 && ready);

	/* Another flag on its own is not data ready. */
	regs[ICM_INT1_STATUS0] = ICM_INT_FIFO_FULL | ICM_INT_FIFO_THS;
	assert(icm45686_data_ready(&imu, &ready) == 0 && !ready);

	icm45686_remove(&imu);
}

static int other_read(struct imu_dev *imu, struct imu_sample *s)
{
	(void)imu;
	(void)s;
	return 0;
}

static void test_null_and_foreign_arguments(void)
{
	static const struct imu_ops other_ops = { .read = other_read };
	struct imu_dev other = { .name = "other", .ops = &other_ops,
				 .priv = regs };
	struct icm45686_config c = base_cfg();
	struct imu_dev imu;
	int16_t a[3], g[3];
	bool b;
	uint8_t v;

	assert(icm45686_probe(NULL, &c) == -EINVAL);
	assert(icm45686_probe(&imu, NULL) == -EINVAL);
	assert(icm45686_read_raw(NULL, a, g, NULL) == -EINVAL);
	assert(icm45686_data_ready(NULL, &b) == -EINVAL);
	assert(icm45686_set_ranges(NULL, ICM45686_ACCEL_FS_2G,
				   ICM45686_GYRO_FS_250DPS) == -EINVAL);
	assert(icm45686_set_odr(NULL, ICM45686_ODR_100HZ) == -EINVAL);
	assert(icm45686_set_power(NULL, true, true) == -EINVAL);
	assert(icm45686_reg_read(NULL, 0, &v, 1) == -EINVAL);
	assert(icm45686_reg_write(NULL, 0, 0) == -EINVAL);
	assert(icm45686_accel_sensitivity(NULL) == 0.0f);
	assert(icm45686_gyro_sensitivity(NULL) == 0.0f);
	icm45686_remove(NULL);

	/* Another chip's imu_dev is refused, never cast to ours. */
	assert(icm45686_read_raw(&other, a, g, NULL) == -EINVAL);
	assert(icm45686_set_power(&other, true, true) == -EINVAL);
	assert(icm45686_gyro_sensitivity(&other) == 0.0f);
	icm45686_remove(&other);
	assert(other.ops == &other_ops && "not ours to remove");

	imu = open_ok();
	assert(icm45686_read_raw(&imu, NULL, g, NULL) == -EINVAL);
	assert(icm45686_read_raw(&imu, a, NULL, NULL) == -EINVAL);
	assert(icm45686_data_ready(&imu, NULL) == -EINVAL);
	assert(icm45686_reg_read(&imu, 0, &v, 0) == -EINVAL);
	assert(icm45686_reg_read(&imu, 0, &v, 99) == -EINVAL);
	icm45686_remove(&imu);
}

/*
 * What the cache says must be what the chip holds, even after a failure: the
 * scale imu_read() uses, and the ODRs the next range change writes back.
 */
static void assert_config_matches_chip(struct imu_dev *imu)
{
	uint8_t a0 = regs[ICM_ACCEL_CONFIG0], g0 = regs[ICM_GYRO_CONFIG0];
	int afs = (a0 >> ICM_ACCEL_FS_SHIFT) & 0x7;
	int gfs = g0 >> ICM_GYRO_FS_SHIFT;
	struct imu_sample s;

	assert(icm45686_accel_sensitivity(imu) ==
	       icm45686_accel_sensitivity_code(afs));
	assert(icm45686_gyro_sensitivity(imu) ==
	       icm45686_gyro_sensitivity_code(gfs));

	put_sample(0x00, 1024);
	put_sample(0x06, 1000);
	assert(imu_read(imu, &s) == 0);
	assert(fabs(s.accel[0] - icm45686_accel_g(1024, afs) * G) < 1e-4);
	assert(fabs(s.gyro[0] - icm45686_gyro_dps(1000, gfs) * DEG) < 1e-6);

	/* Rewriting the ranges the chip already has must not move an ODR. */
	assert(icm45686_set_ranges(imu, (enum icm45686_accel_fs)afs,
				   (enum icm45686_gyro_fs)gfs) == 0);
	assert(regs[ICM_ACCEL_CONFIG0] == a0 && regs[ICM_GYRO_CONFIG0] == g0);
}

static void test_failed_writes_keep_the_cache_true(void)
{
	int k;

	/* k = which of the two register writes fails */
	for (k = 0; k < 2; k++) {
		struct imu_dev imu = open_ok();	/* 4 g, 500 dps, 200 Hz */

		fail_write_in = k;
		assert(icm45686_set_ranges(&imu, ICM45686_ACCEL_FS_32G,
					   ICM45686_GYRO_FS_15_625DPS) ==
		       -ETIMEDOUT);
		assert_config_matches_chip(&imu);
		icm45686_remove(&imu);
	}

	for (k = 0; k < 2; k++) {
		struct imu_dev imu = open_ok();

		fail_write_in = k;
		assert(icm45686_set_odr(&imu, ICM45686_ODR_6400HZ) ==
		       -ETIMEDOUT);
		assert_config_matches_chip(&imu);
		icm45686_remove(&imu);
	}
}

static void test_second_probe_is_refused(void)
{
	struct icm45686_config c = base_cfg();
	struct imu_dev imu = open_ok();
	void *priv = imu.priv;
	int a = added, t = transfers, n = allocs;

	assert(icm45686_probe(&imu, &c) == -EBUSY);
	assert(added == a && transfers == t && "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(imu.priv == priv && icm45686_accel_sensitivity(&imu) > 0.0f);
	icm45686_remove(&imu);
}

int main(void)
{
	test_probe_and_reset();
	test_reset_wait_survives_the_tick();
	test_config_encoding();
	test_power();
	test_burst_read();
	test_conversion();
	test_data_ready();
	test_null_and_foreign_arguments();
	test_failed_writes_keep_the_cache_true();
	test_second_probe_is_refused();
	printf("all icm45686 driver checks passed\n");
	return 0;
}
