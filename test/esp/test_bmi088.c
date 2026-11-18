/*
 * Host-side check of the BMI088 ESP-IDF transport and probe/remove
 * (bmi088_bus.c) against fake I2C and SPI dies: the SPI dummy byte, errno
 * mapping, and that a failed probe leaves nothing registered on the bus.
 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

/*
 * The value the accelerometer stub returns as its SPI dummy byte. It is
 * deliberately not a plausible sample so that a driver forgetting to discard
 * it shows up as a decoded value rather than as a subtle offset.
 */
#define SPI_DUMMY_POISON	0xa5

struct fake_dev {
	uint8_t *registers;
	int is_accel;
};

static uint8_t accel_regs[256];
static uint8_t gyro_regs[256];
static struct fake_dev fake_accel;
static struct fake_dev fake_gyro;

static esp_err_t bus_status;
static uint16_t absent_address;	/* this address NACKs; 0: none */
static int probe_count;
static int add_count;
static int xfer_count;
static int rm_count;
static int zero_tick_delays;
static int tests_failed;

#define EXPECT_TRUE(condition) do {					\
	if (!(condition)) {						\
		fprintf(stderr, "%s:%d: expected %s\n",			\
			__FILE__, __LINE__, #condition);		\
		tests_failed++;						\
	}								\
} while (0)

#define EXPECT_EQ(actual, expected) do {				\
	long actual_value = (long)(actual);				\
	long expected_value = (long)(expected);				\
	if (actual_value != expected_value) {				\
		fprintf(stderr, "%s:%d: got %ld, expected %ld\n",	\
			__FILE__, __LINE__, actual_value, expected_value);\
		tests_failed++;						\
	}								\
} while (0)

#define EXPECT_NEAR(actual, expected) \
	EXPECT_TRUE(fabsf((actual) - (expected)) < 0.001f)

/* --- ESP-IDF fakes ---------------------------------------------------- */

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				   const i2c_device_config_t *config,
				   i2c_master_dev_handle_t *out_handle)
{
	(void)bus;

	if (bus_status != ESP_OK)
		return bus_status;

	add_count++;
	*out_handle = (i2c_master_dev_handle_t)
		(config->device_address == BMI088_ACCEL_I2C_ADDR_LOW ||
		 config->device_address == BMI088_ACCEL_I2C_ADDR_HIGH ?
		 &fake_accel : &fake_gyro);

	return ESP_OK;
}

/*
 * The address-only probe. IDF 5.5 reports a NACK here as ESP_ERR_NOT_FOUND;
 * a NACKed transfer is ESP_ERR_INVALID_STATE instead.
 */
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address,
			   int xfer_timeout_ms)
{
	(void)bus;
	(void)xfer_timeout_ms;
	probe_count++;
	return address == absent_address ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
	(void)device;

	rm_count++;
	return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
			      const uint8_t *data, size_t length,
			      int timeout_ms)
{
	struct fake_dev *fake = (struct fake_dev *)device;

	(void)timeout_ms;
	if (bus_status != ESP_OK)
		return bus_status;
	xfer_count++;
	if (length != 2)
		return ESP_ERR_INVALID_SIZE;

	/* Reset commands do not become ordinary stored register values. */
	if (data[0] != 0x7e && data[0] != 0x14)
		fake->registers[data[0]] = data[1];

	return ESP_OK;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
				      const uint8_t *tx, size_t tx_length,
				      uint8_t *rx, size_t rx_length,
				      int timeout_ms)
{
	struct fake_dev *fake = (struct fake_dev *)device;
	size_t index;

	(void)timeout_ms;
	if (bus_status != ESP_OK)
		return bus_status;
	xfer_count++;
	if (tx_length != 1)
		return ESP_ERR_INVALID_SIZE;

	for (index = 0; index < rx_length; index++)
		rx[index] = fake->registers[(uint8_t)(tx[0] + index)];

	return ESP_OK;
}

esp_err_t spi_bus_add_device(spi_host_device_t host,
			     const spi_device_interface_config_t *config,
			     spi_device_handle_t *out_handle)
{
	(void)host;

	if (bus_status != ESP_OK)
		return bus_status;

	add_count++;
	*out_handle = (spi_device_handle_t)(config->spics_io_num == 5 ?
					   &fake_accel : &fake_gyro);

	return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t device)
{
	(void)device;

	rm_count++;
	return ESP_OK;
}

esp_err_t spi_device_polling_transmit(spi_device_handle_t device,
				      spi_transaction_t *transfer)
{
	struct fake_dev *fake = (struct fake_dev *)device;
	const uint8_t *tx = transfer->tx_buffer;
	uint8_t *rx = transfer->rx_buffer;
	size_t total = transfer->length / 8;
	size_t skip = fake->is_accel ? 2 : 1;
	size_t index;

	if (bus_status != ESP_OK)
		return bus_status;
	xfer_count++;
	if (total < 2)
		return ESP_ERR_INVALID_SIZE;

	if (tx[0] & 0x80) {
		if (!rx)
			return ESP_ERR_INVALID_ARG;

		memset(rx, 0, total);
		if (fake->is_accel)
			rx[1] = SPI_DUMMY_POISON;
		for (index = skip; index < total; index++)
			rx[index] = fake->registers[(uint8_t)
				((tx[0] & 0x7f) + (index - skip))];

		return ESP_OK;
	}

	if (tx[0] != 0x7e && tx[0] != 0x14)
		fake->registers[tx[0]] = tx[1];

	return ESP_OK;
}

/* Counts the driver's calloc() calls; linked with -Wl,--wrap=calloc. */
static int allocs;
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
}

void esp_rom_delay_us(uint32_t usec)
{
	(void)usec;
}

void vTaskDelay(TickType_t ticks)
{
	/* A truncated-to-zero delay would not wait at all on real hardware. */
	if (ticks == 0)
		zero_tick_delays++;
}

/* --- tests ----------------------------------------------------------- */

static void setup(void)
{
	memset(accel_regs, 0, sizeof(accel_regs));
	memset(gyro_regs, 0, sizeof(gyro_regs));
	fake_accel.registers = accel_regs;
	fake_accel.is_accel = 1;
	fake_gyro.registers = gyro_regs;
	fake_gyro.is_accel = 0;

	accel_regs[0x00] = 0x1e;
	gyro_regs[0x00] = 0x0f;
	accel_regs[0x41] = 0xa4;
	gyro_regs[0x10] = 0x80;

	/* x = 4660, y = -32768, z = 32767 on both dies. */
	accel_regs[0x12] = 0x34;
	accel_regs[0x13] = 0x12;
	accel_regs[0x14] = 0x00;
	accel_regs[0x15] = 0x80;
	accel_regs[0x16] = 0xff;
	accel_regs[0x17] = 0x7f;
	memcpy(&gyro_regs[0x02], &accel_regs[0x12], 6);

	bus_status = ESP_OK;
	absent_address = 0;
	probe_count = 0;
	add_count = 0;
	xfer_count = 0;
	rm_count = 0;
	zero_tick_delays = 0;
}

static struct bmi088_i2c_config i2c_config(void)
{
	struct bmi088_i2c_config config = { 0 };

	/* Only the adapter's device handles are dereferenced, not the bus. */
	config.bus = (i2c_master_bus_handle_t)&fake_accel;

	return config;
}

static struct bmi088_spi_config spi_config(void)
{
	struct bmi088_spi_config config = { 0 };

	config.host = SPI2_HOST;
	config.accel_cs_pin = 5;
	config.gyro_cs_pin = 17;

	return config;
}

static void test_i2c_scaled_reads(void)
{
	struct bmi088_i2c_config bus_config = i2c_config();
	struct bmi088_vec3 accel;
	struct bmi088_vec3 gyro;
	struct imu_dev imu = { 0 };

	setup();
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), 0);
	EXPECT_EQ(add_count, 2);
	EXPECT_TRUE(!strcmp(imu.name, "bmi088"));

	EXPECT_EQ(bmi088_read_accel(&imu, &accel), 0);
	EXPECT_NEAR(accel.x, 8.3672f);
	EXPECT_NEAR(accel.y, -58.8399f);

	/* Pins the gyro range table: a reversed table gives 2.1806 for x. */
	EXPECT_EQ(bmi088_read_gyro(&imu, &gyro), 0);
	EXPECT_NEAR(gyro.x, 4.9641f);
	EXPECT_NEAR(gyro.y, -34.9066f);

	/* Every init delay must survive pdMS_TO_TICKS() truncation. */
	EXPECT_EQ(zero_tick_delays, 0);

	bmi088_remove(&imu);
	EXPECT_EQ(rm_count, 2);
}

static void test_spi_dummy_byte(void)
{
	struct bmi088_spi_config bus_config = spi_config();
	struct bmi088_vec3_raw raw;
	struct imu_dev imu = { 0 };

	setup();
	EXPECT_EQ(bmi088_probe_spi(&imu, &bus_config, NULL), 0);
	EXPECT_EQ(add_count, 2);

	/*
	 * The accelerometer's dummy byte must be discarded and the
	 * gyroscope's absent one must not be, so both dies decode the same
	 * register contents to the same values.
	 */
	EXPECT_EQ(bmi088_read_accel_raw(&imu, &raw), 0);
	EXPECT_EQ(raw.x, 4660);
	EXPECT_EQ(raw.y, -32768);
	EXPECT_EQ(raw.z, 32767);

	EXPECT_EQ(bmi088_read_gyro_raw(&imu, &raw), 0);
	EXPECT_EQ(raw.x, 4660);
	EXPECT_EQ(raw.y, -32768);
	EXPECT_EQ(raw.z, 32767);

	bmi088_remove(&imu);
	EXPECT_EQ(rm_count, 2);
}

static void test_data_ready_and_temperature(void)
{
	struct bmi088_i2c_config bus_config = i2c_config();
	struct imu_dev imu = { 0 };
	float temperature = 99.0f;
	uint8_t ready = 0xff;

	setup();
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), 0);

	accel_regs[0x03] = 0x80;
	gyro_regs[0x0a] = 0x80;
	EXPECT_EQ(bmi088_data_ready(&imu, &ready), 0);
	EXPECT_EQ(ready, BMI088_DATA_READY_ACCEL | BMI088_DATA_READY_GYRO);

	accel_regs[0x03] = 0x00;
	EXPECT_EQ(bmi088_data_ready(&imu, &ready), 0);
	EXPECT_EQ(ready, BMI088_DATA_READY_GYRO);

	accel_regs[0x22] = 0xc1;
	accel_regs[0x23] = 0x00;
	EXPECT_EQ(bmi088_read_temperature(&imu, &temperature), 0);
	EXPECT_NEAR(temperature, -40.0f);

	bmi088_remove(&imu);
}

static void test_failures_release_devices(void)
{
	struct bmi088_i2c_config bus_config = i2c_config();
	struct imu_dev imu = { 0 };

	/* A bus fault must map to errno and leave nothing registered. */
	setup();
	bus_status = ESP_ERR_TIMEOUT;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ETIMEDOUT);
	EXPECT_EQ(rm_count, 0);

	/* So must a sensor that reports a persistent error after config. */
	setup();
	accel_regs[0x02] = 0x01;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -EIO);
	EXPECT_EQ(add_count, 2);
	EXPECT_EQ(rm_count, 2);

	/* And a wrong chip ID. */
	setup();
	accel_regs[0x00] = 0x00;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ENODEV);
	EXPECT_EQ(rm_count, 2);
	EXPECT_TRUE(!imu.priv && !imu.ops);
	EXPECT_TRUE(strstr(stub_log_last, "E imu: bmi088: ") != NULL);

	setup();
	EXPECT_EQ(bmi088_probe_i2c(NULL, &bus_config, NULL), -EINVAL);
	EXPECT_EQ(bmi088_probe_i2c(&imu, NULL, NULL), -EINVAL);
	EXPECT_EQ(add_count, 0);
}

static void test_absent_die(void)
{
	struct bmi088_i2c_config bus_config = i2c_config();
	struct imu_dev imu = { 0 };
	int before;

	/*
	 * Either die missing is -ENXIO, decided before anything is allocated,
	 * added or transferred, and the log says which die.
	 */
	setup();
	absent_address = BMI088_ACCEL_I2C_ADDR_LOW;
	before = allocs;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ENXIO);
	EXPECT_EQ(allocs, before);
	EXPECT_EQ(add_count, 0);
	EXPECT_EQ(xfer_count, 0);
	EXPECT_TRUE(!imu.priv && !imu.ops);
	EXPECT_TRUE(strstr(stub_log_last, "accel") != NULL);

	setup();
	absent_address = BMI088_GYRO_I2C_ADDR_LOW;
	before = allocs;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ENXIO);
	EXPECT_EQ(allocs, before);
	EXPECT_EQ(add_count, 0);
	EXPECT_EQ(xfer_count, 0);
	EXPECT_TRUE(!imu.priv && !imu.ops);
	EXPECT_TRUE(strstr(stub_log_last, "gyro") != NULL);

	/* Both present: both addresses were probed. */
	setup();
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), 0);
	EXPECT_EQ(probe_count, 2);

	/* A NACK in the middle of a transfer is an I/O error, not absence. */
	{
		struct imu_sample s;

		bus_status = ESP_ERR_INVALID_STATE;
		EXPECT_EQ(imu_read(&imu, &s), -EIO);
		bus_status = ESP_OK;
	}
	bmi088_remove(&imu);
}

static void test_rejects_bad_bus_config(void)
{
	struct bmi088_spi_config spi_bad = spi_config();
	struct bmi088_i2c_config i2c_bad = i2c_config();
	struct imu_dev imu = { 0 };

	setup();
	spi_bad.mode = 1;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	spi_bad = spi_config();
	spi_bad.clock_hz = 20000000;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	spi_bad = spi_config();
	spi_bad.gyro_cs_pin = spi_bad.accel_cs_pin;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	i2c_bad.clock_hz = 1000000;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bad, NULL), -EINVAL);

	i2c_bad = i2c_config();
	i2c_bad.accel_address = 0x20;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bad, NULL), -EINVAL);

	/* Negative used to reach IDF, where -1 waits for ever; now refused. */
	i2c_bad = i2c_config();
	i2c_bad.timeout_ms = -1;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bad, NULL), -EINVAL);

	EXPECT_EQ(add_count, 0);
}

static int other_read(struct imu_dev *imu, struct imu_sample *s)
{
	(void)imu;
	(void)s;
	return 0;
}

static void test_rejects_foreign_imu_dev(void)
{
	static const struct imu_ops other_ops = { .read = other_read };
	struct imu_dev other = { .name = "other", .ops = &other_ops,
				 .priv = accel_regs };
	struct bmi088_vec3 accel;
	uint8_t ready;

	/* Another chip's imu_dev is refused, never cast to ours. */
	EXPECT_EQ(bmi088_read_accel(&other, &accel), -EINVAL);
	EXPECT_EQ(bmi088_data_ready(&other, &ready), -EINVAL);
	bmi088_remove(&other);
	EXPECT_TRUE(other.ops == &other_ops && other.priv == accel_regs);
}

static void test_second_probe_is_refused(void)
{
	struct bmi088_i2c_config i2c = i2c_config();
	struct bmi088_spi_config spi = spi_config();
	struct imu_dev imu = { 0 };
	int adds, xfers, before;
	void *priv;

	setup();
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c, NULL), 0);
	priv = imu.priv;
	adds = add_count;
	xfers = xfer_count;
	before = allocs;

	/* Bound already: refused on either bus, with no traffic or calloc. */
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c, NULL), -EBUSY);
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi, NULL), -EBUSY);
	EXPECT_EQ(add_count, adds);
	EXPECT_EQ(xfer_count, xfers);
	EXPECT_EQ(allocs, before);
	EXPECT_TRUE(imu.priv == priv);

	bmi088_remove(&imu);
	EXPECT_EQ(rm_count, 2);
}

int main(void)
{
	test_i2c_scaled_reads();
	test_spi_dummy_byte();
	test_data_ready_and_temperature();
	test_failures_release_devices();
	test_absent_die();
	test_rejects_bad_bus_config();
	test_rejects_foreign_imu_dev();
	test_second_probe_is_refused();

	if (tests_failed) {
		fprintf(stderr, "%d bmi088 bus assertion(s) failed\n",
			tests_failed);
		return 1;
	}

	printf("all bmi088 bus checks passed\n");
	return 0;
}
