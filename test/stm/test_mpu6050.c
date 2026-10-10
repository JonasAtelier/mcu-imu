/*
 * Host-side check of the driver against a fake MPU-6050 behind a fake HAL. No
 * hardware, no ARM toolchain -- see test/Makefile.
 *
 * The fake is a plain register file plus the one behaviour that matters:
 * DEVICE_RESET clears it. What is actually being guarded here is the burst
 * layout -- temperature sits between the accelerometer and the gyroscope, so
 * a driver that assumes the obvious order reads gyro bytes as temperature and
 * temperature bytes as gyro X, and both look like plausible numbers. The fake
 * HAL also insists on the HAL's 8-bit address form, the 7-bit address shifted
 * left one, which is the classic way to talk to nobody on STM32.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "imu.h"
#include "mpu6050.h"
#include "mpu6050_math.h"
#include "mpu6050_regs.h"

#define G		9.80665f
#define DEG		0.017453292519943295f

static I2C_HandleTypeDef bus;
static uint8_t regs[256];
static uint16_t dev_addr;	/* 7-bit, of the last transfer */
static int reset_count, longest_read, writes, reads;
static HAL_StatusTypeDef bus_status;
static HAL_StatusTypeDef probe_status;	/* IsDeviceReady on a present chip */
static bool absent;		/* nothing ACKs at the address */
static uint16_t probed_addr;
static int fail_write_in;	/* fail the Nth write from now; -1: never */
static uint32_t delays[8];
static int ndelays;

static void chip_reset(void)
{
	memset(regs, 0, sizeof(regs));
	regs[MPU6050_WHO_AM_I] = MPU6050_WHO_AM_I_VALUE;
	/* asleep out of reset */
	regs[MPU6050_PWR_MGMT_1] = MPU6050_SLEEP;
	reset_count = longest_read = writes = reads = 0;
	ndelays = 0;
	bus_status = HAL_OK;
	probe_status = HAL_OK;
	fail_write_in = -1;
	absent = false;
	probed_addr = 0;
	dev_addr = 0;
}

/*
 * The address-only probe. The real one returns HAL_ERROR once every trial
 * went unACKed; a NACKed transfer is HAL_ERROR too, which the transfer fakes
 * below return for an absent chip.
 */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
					uint16_t address, uint32_t trials,
					uint32_t timeout)
{
	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(trials >= 1 && timeout > 0);
	probed_addr = address >> 1;
	if (absent)
		return HAL_ERROR;
	return probe_status;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
				    uint16_t address, uint16_t reg,
				    uint16_t reg_size, uint8_t *data,
				    uint16_t len, uint32_t timeout)
{
	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xFF);
	assert(len == 1 && "a register write is one value");
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	if (fail_write_in >= 0 && fail_write_in-- == 0)
		return HAL_TIMEOUT;
	writes++;
	dev_addr = address >> 1;

	if (reg == MPU6050_PWR_MGMT_1 && (data[0] & MPU6050_DEVICE_RESET)) {
		uint8_t who = regs[MPU6050_WHO_AM_I];

		memset(regs, 0, sizeof(regs));
		regs[MPU6050_WHO_AM_I] = who;
		regs[MPU6050_PWR_MGMT_1] = MPU6050_SLEEP;
		reset_count++;
		return HAL_OK;
	}
	regs[reg] = data[0];
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t address,
				   uint16_t reg, uint16_t reg_size,
				   uint8_t *data, uint16_t len,
				   uint32_t timeout)
{
	assert(hi2c == &bus);
	assert(!(address & 1) && "the HAL takes the 7-bit address << 1");
	assert(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xFF);
	assert(timeout > 0);
	if (absent)
		return HAL_ERROR;
	if (bus_status != HAL_OK)
		return bus_status;
	reads++;
	dev_addr = address >> 1;
	if ((int)len > longest_read)
		longest_read = (int)len;
	for (uint16_t i = 0; i < len; i++)
		data[i] = regs[(reg + i) & 0xFF];
	return HAL_OK;
}

/* Counts the drivers' calloc() and free() calls; see test/Makefile. */
static int allocs, frees;
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
}
void __real_free(void *p);
void __wrap_free(void *p);
void __wrap_free(void *p)
{
	frees++;
	__real_free(p);
}

void HAL_Delay(uint32_t ms)
{
	if (ndelays < 8)
		delays[ndelays++] = ms;
}

/* ------------------------------------------------------------ helpers ---- */

static struct mpu6050_config base_cfg(void)
{
	struct mpu6050_config c = {
		.accel_fs = MPU6050_ACCEL_FS_4G,
		.gyro_fs = MPU6050_GYRO_FS_500DPS,
		.dlpf = MPU6050_DLPF_44HZ,
		.sample_rate_hz = 100.0f,
	};

	return c;
}

static struct imu_dev open_ok(void)
{
	struct mpu6050_config c = base_cfg();
	struct imu_dev imu = { 0 };

	chip_reset();
	assert(mpu6050_probe(&imu, &bus, &c) == 0);
	assert(imu.priv && imu.ops);
	return imu;
}

/* big endian, high byte first */
static void put_sample(int reg, int16_t v)
{
	regs[reg] = (uint8_t)((uint16_t)v >> 8);
	regs[reg + 1] = (uint8_t)(v & 0xFF);
}

/* -------------------------------------------------------------- tests ---- */

static void test_probe(void)
{
	struct mpu6050_config c = base_cfg();
	struct imu_dev imu = { 0 };

	chip_reset();
	imu.mount[0][1] = 1;			/* probe must not touch it */
	assert(mpu6050_probe(&imu, &bus, &c) == 0);
	assert(!strcmp(imu.name, "mpu6050"));
	assert(imu.mount[0][1] == 1);
	assert(dev_addr == MPU6050_ADDR_LOW && "0 must select AD0-low");
	assert(reset_count == 1);
	/* Reset leaves it asleep on the RC oscillator; probe fixes both. */
	assert(regs[MPU6050_PWR_MGMT_1] == MPU6050_CLKSEL_PLL_X);
	assert(!(regs[MPU6050_PWR_MGMT_1] & MPU6050_SLEEP));
	mpu6050_remove(&imu);
	assert(!imu.priv && !imu.ops);

	chip_reset();
	c.address = MPU6050_ADDR_HIGH;
	assert(mpu6050_probe(&imu, &bus, &c) == 0);
	assert(dev_addr == MPU6050_ADDR_HIGH);
	mpu6050_remove(&imu);

	/* A wrong identity is not this part, and the log says what it saw. */
	chip_reset();
	c = base_cfg();
	regs[MPU6050_WHO_AM_I] = 0x71;		/* what an MPU-9250 says */
	allocs = frees = 0;
	assert(mpu6050_probe(&imu, &bus, &c) == -ENODEV);
	assert(strstr(stub_log_last, "imu: mpu6050: "));
	assert(strstr(stub_log_last, "WHO_AM_I=0x71, expected 0x68"));
	assert(!imu.priv && !imu.ops);
	assert(allocs == 1 && frees == 1 && "nothing leaked");
	chip_reset();
	regs[MPU6050_WHO_AM_I] = 0xFF;
	assert(mpu6050_probe(&imu, &bus, &c) == -ENODEV);

	/*
	 * No ACK is a different failure from the wrong chip, and is decided
	 * before anything is allocated or written.
	 */
	chip_reset();
	c.address = MPU6050_ADDR_HIGH;
	absent = true;
	allocs = 0;
	assert(mpu6050_probe(&imu, &bus, &c) == -ENXIO);
	assert(probed_addr == MPU6050_ADDR_HIGH);
	assert(allocs == 0 && writes == 0 && reads == 0);
	assert(!imu.priv && !imu.ops);
	c = base_cfg();

	/* A bus that is busy or hangs during the presence check is not absence. */
	chip_reset();
	probe_status = HAL_BUSY;	/* also: a handle never initialised */
	allocs = 0;
	assert(mpu6050_probe(&imu, &bus, &c) == -EAGAIN);
	assert(allocs == 0 && writes == 0 && reads == 0);
	chip_reset();
	probe_status = HAL_TIMEOUT;
	assert(mpu6050_probe(&imu, &bus, &c) == -ETIMEDOUT);
	assert(allocs == 0);

	/* A NACK in the middle of a transfer is an I/O error, not absence. */
	chip_reset();
	bus_status = HAL_ERROR;
	allocs = frees = 0;
	assert(mpu6050_probe(&imu, &bus, &c) == -EIO);
	assert(allocs == 1 && frees == 1);
	chip_reset();
	bus_status = HAL_TIMEOUT;
	assert(mpu6050_probe(&imu, &bus, &c) == -ETIMEDOUT);
	chip_reset();
	/* Not -EBUSY: from a probe that means "already bound". */
	bus_status = HAL_BUSY;
	assert(mpu6050_probe(&imu, &bus, &c) == -EAGAIN);

	/* Addresses other than the two the part can take are refused. */
	chip_reset();
	c.address = 0x53;
	allocs = 0;
	assert(mpu6050_probe(&imu, &bus, &c) == -EINVAL);
	assert(allocs == 0 && probed_addr == 0);
}

/*
 * HAL_Delay() adds the tick that may be about to fire on its own, so a wait
 * is long enough as long as it asks for at least the milliseconds the chip
 * needs -- never a rounded-down tick count.
 */
static void test_waits_are_long_enough(void)
{
	struct imu_dev imu = open_ok();

	assert(ndelays == 2);
	assert(delays[0] >= 100 && "100 ms reset wait");
	assert(delays[1] >= 10 && "10 ms wake wait");

	ndelays = 0;
	assert(mpu6050_sleep(&imu, false) == 0);
	assert(ndelays == 1 && delays[0] >= 10 && "10 ms wake wait");
	mpu6050_remove(&imu);
}

static void test_ranges(void)
{
	struct imu_dev imu = open_ok();

	assert(regs[MPU6050_ACCEL_CONFIG] == (MPU6050_ACCEL_FS_4G << 3));
	assert(regs[MPU6050_GYRO_CONFIG] == (MPU6050_GYRO_FS_500DPS << 3));
	assert(fabs(mpu6050_accel_sensitivity(&imu) - 8192.0f) < 1e-3);
	assert(fabs(mpu6050_gyro_sensitivity(&imu) - 65.5f) < 1e-3);

	assert(mpu6050_set_ranges(&imu, MPU6050_ACCEL_FS_16G,
				  MPU6050_GYRO_FS_2000DPS) == 0);
	assert(regs[MPU6050_ACCEL_CONFIG] == (3 << 3));
	assert(regs[MPU6050_GYRO_CONFIG] == (3 << 3));
	assert(fabs(mpu6050_accel_sensitivity(&imu) - 2048.0f) < 1e-3);

	assert(mpu6050_set_ranges(&imu, (enum mpu6050_accel_fs)4,
				  MPU6050_GYRO_FS_250DPS) == -EINVAL);
	assert(mpu6050_set_ranges(&imu, MPU6050_ACCEL_FS_2G,
				  (enum mpu6050_gyro_fs)4) == -EINVAL);
	mpu6050_remove(&imu);
}

static void test_rate_follows_filter(void)
{
	struct imu_dev imu = open_ok();

	/* 100 Hz with a 1 kHz base is divider 9. */
	assert(regs[MPU6050_CONFIG] == MPU6050_DLPF_44HZ);
	assert(regs[MPU6050_SMPLRT_DIV] == 9);
	assert(fabs(mpu6050_sample_rate(&imu) - 100.0f) < 1e-3);

	/*
	 * The same 100 Hz with the filter off needs a different divider,
	 * because the base rate jumped to 8 kHz. Getting this wrong runs the
	 * part 8x fast.
	 */
	assert(mpu6050_set_rate(&imu, MPU6050_DLPF_OFF, 100.0f) == 0);
	assert(regs[MPU6050_CONFIG] == MPU6050_DLPF_OFF);
	assert(regs[MPU6050_SMPLRT_DIV] == 79);
	assert(fabs(mpu6050_sample_rate(&imu) - 100.0f) < 1e-3);

	/* 10 Hz is unreachable at 8 kHz and must be refused, not clamped. */
	assert(mpu6050_set_rate(&imu, MPU6050_DLPF_OFF, 10.0f) == -EINVAL);
	/* Untouched by the failed call. */
	assert(regs[MPU6050_SMPLRT_DIV] == 79);
	/* The same 10 Hz is fine once the filter drops the base to 1 kHz. */
	assert(mpu6050_set_rate(&imu, MPU6050_DLPF_10HZ, 10.0f) == 0);
	assert(regs[MPU6050_SMPLRT_DIV] == 99);

	assert(mpu6050_set_rate(&imu, (enum mpu6050_dlpf)8, 100.0f) == -EINVAL);
	assert(mpu6050_set_rate(&imu, MPU6050_DLPF_44HZ, 0.0f) == -EINVAL);
	mpu6050_remove(&imu);
}

static void test_burst_layout(void)
{
	struct imu_dev imu = open_ok();
	int16_t a[3], g[3], t;

	/* Distinct values so a mixed-up offset cannot pass by luck. */
	put_sample(0x3B, 111);
	put_sample(0x3D, 222);
	put_sample(0x3F, 333);
	put_sample(0x41, 4444);			/* TEMP_OUT, in the middle */
	put_sample(0x43, -555);
	put_sample(0x45, -666);
	put_sample(0x47, -777);

	longest_read = 0;
	assert(mpu6050_read_raw(&imu, a, g, &t) == 0);
	assert(longest_read == MPU6050_BURST_LEN && "one burst, not three");

	assert(a[0] == 111 && a[1] == 222 && a[2] == 333);
	assert(t == 4444);
	assert(g[0] == -555 && g[1] == -666 && g[2] == -777);

	assert(mpu6050_read_raw(&imu, a, g, NULL) == 0);
	mpu6050_remove(&imu);
}

static void test_conversion(void)
{
	struct mpu6050_config c = base_cfg();
	struct imu_dev imu = { 0 };
	struct imu_sample s;

	c.accel_fs = MPU6050_ACCEL_FS_2G;
	c.gyro_fs = MPU6050_GYRO_FS_250DPS;
	chip_reset();
	assert(mpu6050_probe(&imu, &bus, &c) == 0);

	put_sample(0x3B, 16384);
	put_sample(0x3D, -16384);
	put_sample(0x3F, 0);
	put_sample(0x41, 0);
	put_sample(0x43, 131);
	put_sample(0x45, -131);
	put_sample(0x47, 0);

	/* Through the core: 1 g and 1 dps, in SI. */
	assert(imu_read(&imu, &s) == 0);
	assert(fabs(s.accel[0] - G) < 1e-4);
	assert(fabs(s.accel[1] + G) < 1e-4);
	assert(fabs(s.gyro[0] - DEG) < 1e-6);
	assert(fabs(s.gyro[1] + DEG) < 1e-6);
	/* raw 0 is 36.53 C, not 0 C and not 36 C. */
	assert(fabs(s.temp_c - 36.53f) < 1e-3);

	mpu6050_remove(&imu);
}

static void test_bypass_and_sleep(void)
{
	struct imu_dev imu = open_ok();

	/* left on by whoever came before */
	regs[MPU6050_USER_CTRL] = MPU6050_I2C_MST_EN;
	assert(mpu6050_i2c_bypass(&imu, true) == 0);
	assert(regs[MPU6050_INT_PIN_CFG] & MPU6050_I2C_BYPASS_EN);
	/* Bypass does nothing while the internal master is on. */
	assert(!(regs[MPU6050_USER_CTRL] & MPU6050_I2C_MST_EN));

	assert(mpu6050_i2c_bypass(&imu, false) == 0);
	assert(!(regs[MPU6050_INT_PIN_CFG] & MPU6050_I2C_BYPASS_EN));

	assert(mpu6050_sleep(&imu, true) == 0);
	assert(regs[MPU6050_PWR_MGMT_1] & MPU6050_SLEEP);
	/* Sleeping must not disturb the clock source. */
	assert((regs[MPU6050_PWR_MGMT_1] & MPU6050_CLKSEL_MASK) ==
	       MPU6050_CLKSEL_PLL_X);

	assert(mpu6050_sleep(&imu, false) == 0);
	assert(!(regs[MPU6050_PWR_MGMT_1] & MPU6050_SLEEP));

	/* Tearing down must leave it asleep, not drawing current. */
	mpu6050_remove(&imu);
	assert(regs[MPU6050_PWR_MGMT_1] & MPU6050_SLEEP);
}

static void test_data_ready(void)
{
	struct imu_dev imu = open_ok();
	bool ready = true;

	regs[MPU6050_INT_STATUS] = 0;
	assert(mpu6050_data_ready(&imu, &ready) == 0 && !ready);
	regs[MPU6050_INT_STATUS] = MPU6050_DATA_RDY;
	assert(mpu6050_data_ready(&imu, &ready) == 0 && ready);
	regs[MPU6050_INT_STATUS] = 0x10;		/* some other flag */
	assert(mpu6050_data_ready(&imu, &ready) == 0 && !ready);
	mpu6050_remove(&imu);
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
	struct mpu6050_config c = base_cfg();
	struct imu_dev imu;
	int16_t a[3], g[3];
	bool b;
	uint8_t v;

	assert(mpu6050_probe(NULL, &bus, &c) == -EINVAL);
	assert(mpu6050_probe(&imu, &bus, NULL) == -EINVAL);
	assert(mpu6050_probe(&imu, NULL, &c) == -EINVAL);
	assert(mpu6050_read_raw(NULL, a, g, NULL) == -EINVAL);
	assert(mpu6050_data_ready(NULL, &b) == -EINVAL);
	assert(mpu6050_set_ranges(NULL, MPU6050_ACCEL_FS_2G,
				  MPU6050_GYRO_FS_250DPS) == -EINVAL);
	assert(mpu6050_set_rate(NULL, MPU6050_DLPF_44HZ, 100.0f) == -EINVAL);
	assert(mpu6050_sleep(NULL, true) == -EINVAL);
	assert(mpu6050_i2c_bypass(NULL, true) == -EINVAL);
	assert(mpu6050_reg_read(NULL, 0, &v, 1) == -EINVAL);
	assert(mpu6050_reg_write(NULL, 0, 0) == -EINVAL);
	assert(mpu6050_accel_sensitivity(NULL) == 0.0f);
	assert(mpu6050_gyro_sensitivity(NULL) == 0.0f);
	assert(mpu6050_sample_rate(NULL) == 0.0f);
	mpu6050_remove(NULL);

	/* Another chip's imu_dev is refused, never cast to ours. */
	assert(mpu6050_read_raw(&other, a, g, NULL) == -EINVAL);
	assert(mpu6050_sleep(&other, true) == -EINVAL);
	assert(mpu6050_reg_write(&other, 0, 0) == -EINVAL);
	assert(mpu6050_sample_rate(&other) == 0.0f);
	mpu6050_remove(&other);
	assert(other.ops == &other_ops && "not ours to remove");

	imu = open_ok();
	assert(mpu6050_read_raw(&imu, NULL, g, NULL) == -EINVAL);
	assert(mpu6050_read_raw(&imu, a, NULL, NULL) == -EINVAL);
	assert(mpu6050_data_ready(&imu, NULL) == -EINVAL);
	assert(mpu6050_reg_read(&imu, 0, &v, 0) == -EINVAL);
	mpu6050_remove(&imu);
}

/* What the cache says must be what the chip holds, even after a failure. */
static void assert_ranges_match_chip(struct imu_dev *imu)
{
	int afs = regs[MPU6050_ACCEL_CONFIG] >> MPU6050_FS_SHIFT;
	int gfs = regs[MPU6050_GYRO_CONFIG] >> MPU6050_FS_SHIFT;
	struct imu_sample s;

	assert(mpu6050_accel_sensitivity(imu) ==
	       mpu6050_accel_sensitivity_of(afs));
	assert(mpu6050_gyro_sensitivity(imu) ==
	       mpu6050_gyro_sensitivity_of(gfs));

	put_sample(0x3B, 2048);
	put_sample(0x43, 164);
	assert(imu_read(imu, &s) == 0);
	assert(fabs(s.accel[0] - mpu6050_accel_g(2048, afs) * G) < 1e-4);
	assert(fabs(s.gyro[0] - mpu6050_gyro_dps(164, gfs) * DEG) < 1e-6);
}

static void test_failed_writes_keep_the_cache_true(void)
{
	int k;

	/* k = which of the two register writes fails */
	for (k = 0; k < 2; k++) {
		struct imu_dev imu = open_ok();	/* 4 g, 500 dps */

		fail_write_in = k;
		assert(mpu6050_set_ranges(&imu, MPU6050_ACCEL_FS_16G,
					  MPU6050_GYRO_FS_2000DPS) ==
		       -ETIMEDOUT);
		assert_ranges_match_chip(&imu);
		mpu6050_remove(&imu);
	}

	for (k = 0; k < 2; k++) {
		struct imu_dev imu = open_ok();	/* 44 Hz filter, div 9 */

		fail_write_in = k;
		/* filter off moves the base to 8 kHz and the divider to 79 */
		assert(mpu6050_set_rate(&imu, MPU6050_DLPF_OFF, 100.0f) ==
		       -ETIMEDOUT);
		assert(mpu6050_sample_rate(&imu) ==
		       mpu6050_sample_rate_of(regs[MPU6050_CONFIG],
					      regs[MPU6050_SMPLRT_DIV]));
		mpu6050_remove(&imu);
	}
}

static void test_rate_above_base_is_refused(void)
{
	struct imu_dev imu = open_ok();
	int before = writes;

	/* The 44 Hz filter means a 1 kHz base: 2 kHz is out of reach. */
	assert(mpu6050_set_rate(&imu, MPU6050_DLPF_44HZ, 2000.0f) == -EINVAL);
	assert(writes == before && "nothing written for a refused rate");
	/* Exactly the base rate is still fine, at divider 0. */
	assert(mpu6050_set_rate(&imu, MPU6050_DLPF_44HZ, 1000.0f) == 0);
	assert(regs[MPU6050_SMPLRT_DIV] == 0);
	mpu6050_remove(&imu);
}

static void test_second_probe_is_refused(void)
{
	static const struct imu_ops other_ops = { .read = other_read };
	struct imu_dev other = { .name = "other", .ops = &other_ops };
	struct mpu6050_config c = base_cfg();
	struct imu_dev imu = open_ok();
	void *priv = imu.priv;
	int w = writes, r = reads, n = allocs;

	assert(mpu6050_probe(&imu, &bus, &c) == -EBUSY);
	assert(writes == w && reads == r && "no bus traffic");
	assert(allocs == n && "no allocation");
	assert(imu.priv == priv && mpu6050_sample_rate(&imu) > 0.0f);
	mpu6050_remove(&imu);

	/* Bound by another driver: refused too, and left exactly as it was. */
	assert(mpu6050_probe(&other, &bus, &c) == -EBUSY);
	assert(!strcmp(other.name, "other") && other.ops == &other_ops);
}

int main(void)
{
	test_probe();
	test_waits_are_long_enough();
	test_ranges();
	test_rate_follows_filter();
	test_burst_layout();
	test_conversion();
	test_bypass_and_sleep();
	test_data_ready();
	test_null_and_foreign_arguments();
	test_failed_writes_keep_the_cache_true();
	test_rate_above_base_is_refused();
	test_second_probe_is_refused();
	printf("all mpu6050 driver checks passed\n");
	return 0;
}
