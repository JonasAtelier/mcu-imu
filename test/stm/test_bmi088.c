/*
 * Host-side check of the BMI088 STM32 HAL transport and probe/remove
 * (bmi088_bus.c) against fake I2C and SPI dies: the SPI dummy byte, the
 * accelerometer's switch to SPI on a chip-select rising edge, errno mapping,
 * and that a failed probe leaves nothing allocated.
 */
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "imu.h"
#include "stm32_hal_stub.h"

/*
 * The value the accelerometer stub returns as its SPI dummy byte. It is
 * deliberately not a plausible sample so that a driver forgetting to discard
 * it shows up as a decoded value rather than as a subtle offset.
 */
#define SPI_DUMMY_POISON	0xa5
/* What an accelerometer still in I2C mode leaves on MISO: nothing useful. */
#define I2C_MODE_POISON		0x5a

struct fake_dev {
	uint8_t *registers;
	int is_accel;
};

static uint8_t accel_regs[256];
static uint8_t gyro_regs[256];
static struct fake_dev fake_accel;
static struct fake_dev fake_gyro;

static I2C_HandleTypeDef i2c;
/* What the app's HAL_SPI_Init() would have left: mode 0, 8-bit frames. */
static SPI_HandleTypeDef spi = {
	.Init = {
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_LOW,
		.CLKPhase = SPI_PHASE_1EDGE,
	},
};
static SPI_HandleTypeDef *expected_spi;	/* the handle a probe was given */
static GPIO_TypeDef port_a, port_b;
#define ACCEL_CS_PORT	(&port_a)
#define ACCEL_CS_PIN	GPIO_PIN_4
#define GYRO_CS_PORT	(&port_b)
#define GYRO_CS_PIN	GPIO_PIN_0

static bool accel_cs_low, gyro_cs_low;
/*
 * The accelerometer starts in I2C mode and moves to SPI on a rising edge of
 * CSB1; a soft reset puts it back (datasheet 3 and 6.1).
 */
static bool accel_spi_mode;
static bool accel_reset_pending;
static int accel_i2c_mode_reads;

static HAL_StatusTypeDef bus_status;
static uint16_t absent_address;	/* this 7-bit address NACKs; 0: none */
static int probe_count;
static int xfer_count;
static int zero_ms_delays;
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

/* --- STM32 HAL fakes -------------------------------------------------- */

static struct fake_dev *i2c_fake(uint16_t address)
{
	uint16_t seven_bit = address >> 1;

	if (address & 1)
		return NULL;	/* not the HAL's 7-bit << 1 form */
	if (seven_bit == BMI088_ACCEL_I2C_ADDR_LOW ||
	    seven_bit == BMI088_ACCEL_I2C_ADDR_HIGH)
		return &fake_accel;
	if (seven_bit == BMI088_GYRO_I2C_ADDR_LOW ||
	    seven_bit == BMI088_GYRO_I2C_ADDR_HIGH)
		return &fake_gyro;
	return NULL;
}

/* The real one returns HAL_ERROR once every trial went unACKed. */
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
					uint16_t address, uint32_t trials,
					uint32_t timeout)
{
	(void)trials;
	(void)timeout;
	EXPECT_TRUE(hi2c == &i2c);
	probe_count++;
	if (!i2c_fake(address) || address >> 1 == absent_address)
		return HAL_ERROR;
	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
				    uint16_t address, uint16_t reg,
				    uint16_t reg_size, uint8_t *data,
				    uint16_t length, uint32_t timeout)
{
	struct fake_dev *fake = i2c_fake(address);

	(void)timeout;
	EXPECT_TRUE(hi2c == &i2c && fake);
	EXPECT_TRUE(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xff);
	if (bus_status != HAL_OK)
		return bus_status;
	xfer_count++;
	if (!fake || length != 1)
		return HAL_ERROR;

	/* Reset commands do not become ordinary stored register values. */
	if (reg != 0x7e && reg != 0x14)
		fake->registers[reg] = data[0];

	return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t address,
				   uint16_t reg, uint16_t reg_size,
				   uint8_t *data, uint16_t length,
				   uint32_t timeout)
{
	struct fake_dev *fake = i2c_fake(address);
	uint16_t index;

	(void)timeout;
	EXPECT_TRUE(hi2c == &i2c && fake);
	EXPECT_TRUE(reg_size == I2C_MEMADD_SIZE_8BIT && reg <= 0xff);
	if (bus_status != HAL_OK)
		return bus_status;
	xfer_count++;
	if (!fake)
		return HAL_ERROR;

	for (index = 0; index < length; index++)
		data[index] = fake->registers[(uint8_t)(reg + index)];

	return HAL_OK;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
	bool low = state == GPIO_PIN_RESET;

	if (port == ACCEL_CS_PORT && pin == ACCEL_CS_PIN) {
		/* A rising edge: SPI from here on, unless it ended a reset. */
		if (accel_cs_low && !low) {
			accel_spi_mode = !accel_reset_pending;
			accel_reset_pending = false;
		}
		accel_cs_low = low;
	} else if (port == GYRO_CS_PORT && pin == GYRO_CS_PIN) {
		gyro_cs_low = low;
	} else {
		EXPECT_TRUE(!"a pin that is not one of our chip selects");
	}
}

/* One SPI transaction to whichever die's chip select is low. */
static HAL_StatusTypeDef spi_transfer(const uint8_t *tx, uint8_t *rx,
				      size_t total)
{
	struct fake_dev *fake;
	size_t skip;
	size_t index;

	if (bus_status != HAL_OK)
		return bus_status;
	xfer_count++;
	if (accel_cs_low == gyro_cs_low) {
		EXPECT_TRUE(!"exactly one die selected");
		return HAL_ERROR;
	}
	fake = accel_cs_low ? &fake_accel : &fake_gyro;
	skip = fake->is_accel ? 2 : 1;
	if (total < 2)
		return HAL_ERROR;

	if (fake->is_accel && !accel_spi_mode) {
		/* Still in I2C mode: it neither answers nor listens. */
		if (rx) {
			memset(rx, I2C_MODE_POISON, total);
			accel_i2c_mode_reads++;
		}
		return HAL_OK;
	}

	if (tx[0] & 0x80) {
		if (!rx)
			return HAL_ERROR;

		memset(rx, 0, total);
		if (fake->is_accel)
			rx[1] = SPI_DUMMY_POISON;
		for (index = skip; index < total; index++)
			rx[index] = fake->registers[(uint8_t)
				((tx[0] & 0x7f) + (index - skip))];

		return HAL_OK;
	}

	if (fake->is_accel && tx[0] == 0x7e && tx[1] == 0xb6)
		accel_reset_pending = true;
	if (tx[0] != 0x7e && tx[0] != 0x14)
		fake->registers[tx[0]] = tx[1];

	return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,
					  const uint8_t *tx, uint8_t *rx,
					  uint16_t n, uint32_t timeout)
{
	(void)timeout;
	EXPECT_TRUE(h == expected_spi);
	return spi_transfer(tx, rx, n);
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *tx,
				   uint16_t n, uint32_t timeout)
{
	(void)timeout;
	EXPECT_TRUE(h == expected_spi);
	return spi_transfer(tx, NULL, n);
}

/* Counts the driver's calloc() and free() calls; see test/Makefile. */
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
	/* The chip half asks for microseconds; 0 ms would be a lost wait. */
	if (ms == 0)
		zero_ms_delays++;
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

	/* Power-on: both deselected, accelerometer in I2C mode. */
	accel_cs_low = false;
	gyro_cs_low = false;
	accel_spi_mode = false;
	accel_reset_pending = false;
	accel_i2c_mode_reads = 0;

	expected_spi = &spi;
	bus_status = HAL_OK;
	absent_address = 0;
	probe_count = 0;
	xfer_count = 0;
	zero_ms_delays = 0;
	allocs = 0;
	frees = 0;
}

static struct bmi088_i2c_config i2c_config(void)
{
	struct bmi088_i2c_config config = { 0 };

	config.bus = &i2c;

	return config;
}

static struct bmi088_spi_config spi_config(void)
{
	struct bmi088_spi_config config = { 0 };

	config.bus = &spi;
	config.accel_cs_port = ACCEL_CS_PORT;
	config.accel_cs_pin = ACCEL_CS_PIN;
	config.gyro_cs_port = GYRO_CS_PORT;
	config.gyro_cs_pin = GYRO_CS_PIN;

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
	EXPECT_EQ(allocs, 1);
	EXPECT_TRUE(!strcmp(imu.name, "bmi088"));

	EXPECT_EQ(bmi088_read_accel(&imu, &accel), 0);
	EXPECT_NEAR(accel.x, 8.3672f);
	EXPECT_NEAR(accel.y, -58.8399f);

	/* Pins the gyro range table: a reversed table gives 2.1806 for x. */
	EXPECT_EQ(bmi088_read_gyro(&imu, &gyro), 0);
	EXPECT_NEAR(gyro.x, 4.9641f);
	EXPECT_NEAR(gyro.y, -34.9066f);

	/* Every init delay must round up, never down to 0 ms. */
	EXPECT_EQ(zero_ms_delays, 0);

	bmi088_remove(&imu);
	EXPECT_EQ(frees, 1);
	EXPECT_TRUE(!imu.priv && !imu.ops);
}

static void test_spi_dummy_byte(void)
{
	struct bmi088_spi_config bus_config = spi_config();
	struct bmi088_vec3_raw raw;
	struct imu_dev imu = { 0 };

	setup();
	EXPECT_EQ(bmi088_probe_spi(&imu, &bus_config, NULL), 0);
	EXPECT_EQ(allocs, 1);

	/*
	 * Two reads met the accelerometer in I2C mode and were thrown away on
	 * purpose: the one at power-on and the one after its soft reset. Each
	 * one's rising CS edge is what switched it to SPI.
	 */
	EXPECT_EQ(accel_i2c_mode_reads, 2);
	EXPECT_TRUE(accel_spi_mode);
	EXPECT_TRUE(!accel_cs_low && !gyro_cs_low);

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

	EXPECT_EQ(zero_ms_delays, 0);

	bmi088_remove(&imu);
	EXPECT_EQ(frees, 1);
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
	struct bmi088_spi_config spi_bus = spi_config();
	struct imu_dev imu = { 0 };

	/* A bus fault must map to errno and leave nothing allocated. */
	setup();
	bus_status = HAL_TIMEOUT;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ETIMEDOUT);
	EXPECT_EQ(allocs, 1);
	EXPECT_EQ(frees, 1);

	setup();
	bus_status = HAL_BUSY;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bus, NULL), -EAGAIN);
	EXPECT_EQ(frees, allocs);
	EXPECT_TRUE(!accel_cs_low && !gyro_cs_low);

	/* So must a sensor that reports a persistent error after config. */
	setup();
	accel_regs[0x02] = 0x01;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -EIO);
	EXPECT_EQ(allocs, 1);
	EXPECT_EQ(frees, 1);

	/* And a wrong chip ID. */
	setup();
	accel_regs[0x00] = 0x00;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ENODEV);
	EXPECT_EQ(frees, 1);
	EXPECT_TRUE(!imu.priv && !imu.ops);
	EXPECT_TRUE(strstr(stub_log_last, "imu: bmi088: ") != NULL);

	setup();
	EXPECT_EQ(bmi088_probe_i2c(NULL, &bus_config, NULL), -EINVAL);
	EXPECT_EQ(bmi088_probe_i2c(&imu, NULL, NULL), -EINVAL);
	EXPECT_EQ(allocs, 0);
}

static void test_absent_die(void)
{
	struct bmi088_i2c_config bus_config = i2c_config();
	struct imu_dev imu = { 0 };

	/*
	 * Either die missing is -ENXIO, decided before anything is allocated
	 * or transferred, and the log says which die.
	 */
	setup();
	absent_address = BMI088_ACCEL_I2C_ADDR_LOW;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ENXIO);
	EXPECT_EQ(allocs, 0);
	EXPECT_EQ(xfer_count, 0);
	EXPECT_TRUE(!imu.priv && !imu.ops);
	EXPECT_TRUE(strstr(stub_log_last, "accel") != NULL);

	setup();
	absent_address = BMI088_GYRO_I2C_ADDR_LOW;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &bus_config, NULL), -ENXIO);
	EXPECT_EQ(allocs, 0);
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

		bus_status = HAL_ERROR;
		EXPECT_EQ(imu_read(&imu, &s), -EIO);
		bus_status = HAL_OK;
	}
	bmi088_remove(&imu);
}

static void test_rejects_bad_bus_config(void)
{
	struct bmi088_spi_config spi_bad = spi_config();
	struct bmi088_i2c_config i2c_bad = i2c_config();
	SPI_HandleTypeDef mode1 = spi, mode3 = spi, wide = spi;
	struct imu_dev imu = { 0 };

	mode1.Init.CLKPhase = SPI_PHASE_2EDGE;
	mode3.Init.CLKPolarity = SPI_POLARITY_HIGH;
	mode3.Init.CLKPhase = SPI_PHASE_2EDGE;
	wide.Init.DataSize = SPI_DATASIZE_16BIT;

	setup();
	spi_bad.bus = &mode1;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	spi_bad.bus = &wide;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	spi_bad.bus = NULL;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	spi_bad = spi_config();
	spi_bad.gyro_cs_port = spi_bad.accel_cs_port;
	spi_bad.gyro_cs_pin = spi_bad.accel_cs_pin;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	spi_bad = spi_config();
	spi_bad.gyro_cs_pin = 0;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), -EINVAL);

	i2c_bad.accel_address = 0x20;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bad, NULL), -EINVAL);

	i2c_bad = i2c_config();
	i2c_bad.bus = NULL;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bad, NULL), -EINVAL);

	i2c_bad = i2c_config();
	i2c_bad.timeout_ms = -1;
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bad, NULL), -EINVAL);

	EXPECT_EQ(allocs, 0);
	EXPECT_EQ(xfer_count, 0);

	/* Mode 3 is the other mode the BMI088 takes. */
	spi_bad = spi_config();
	spi_bad.bus = &mode3;
	expected_spi = &mode3;
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bad, NULL), 0);
	bmi088_remove(&imu);
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
	struct bmi088_i2c_config i2c_bus = i2c_config();
	struct bmi088_spi_config spi_bus = spi_config();
	struct imu_dev imu = { 0 };
	int probes, xfers, before;
	void *priv;

	setup();
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bus, NULL), 0);
	priv = imu.priv;
	probes = probe_count;
	xfers = xfer_count;
	before = allocs;

	/* Bound already: refused on either bus, with no traffic or calloc. */
	EXPECT_EQ(bmi088_probe_i2c(&imu, &i2c_bus, NULL), -EBUSY);
	EXPECT_EQ(bmi088_probe_spi(&imu, &spi_bus, NULL), -EBUSY);
	EXPECT_EQ(probe_count, probes);
	EXPECT_EQ(xfer_count, xfers);
	EXPECT_EQ(allocs, before);
	EXPECT_TRUE(imu.priv == priv);

	bmi088_remove(&imu);
	EXPECT_EQ(frees, 1);
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
