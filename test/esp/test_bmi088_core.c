/*
 * Host-side check of the chip half of the BMI088 driver (bmi088.c) against a
 * mock struct bmi088_bus_ops: register sequence, scaling, temperature and
 * error paths, with no transport in the way. The ESP-IDF transport is
 * test_bmi088.c's job.
 */
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bmi088.h"
#include "bmi088_priv.h"
#include "esp_log.h"
#include "imu.h"

#define EXPECTED_WRITES 11

struct mock_write {
	enum bmi088_die die;
	uint8_t reg;
	uint8_t value;
};

struct mock_bus {
	uint8_t accel[256];
	uint8_t gyro[256];
	struct mock_write writes[32];
	uint32_t delays[32];
	size_t write_count;
	size_t delay_count;
	int fail_read;
	int fail_write;
};

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

static int mock_read(void *context, enum bmi088_die die, uint8_t reg,
		     uint8_t *data, size_t length)
{
	struct mock_bus *bus = context;
	uint8_t *registers = die == BMI088_DIE_ACCEL ?
			     bus->accel : bus->gyro;

	if (bus->fail_read)
		return bus->fail_read;

	memcpy(data, &registers[reg], length);
	return 0;
}

static int mock_write(void *context, enum bmi088_die die, uint8_t reg,
		      uint8_t value)
{
	struct mock_bus *bus = context;
	uint8_t *registers = die == BMI088_DIE_ACCEL ?
			     bus->accel : bus->gyro;

	if (bus->fail_write)
		return bus->fail_write;

	bus->writes[bus->write_count].die = die;
	bus->writes[bus->write_count].reg = reg;
	bus->writes[bus->write_count].value = value;
	bus->write_count++;

	/* Reset commands do not become ordinary stored register values. */
	if (reg != 0x7e && reg != 0x14)
		registers[reg] = value;

	return 0;
}

static void mock_delay(void *context, uint32_t usec)
{
	struct mock_bus *bus = context;

	bus->delays[bus->delay_count++] = usec;
}

static const struct bmi088_bus_ops mock_ops = {
	.read = mock_read,
	.write = mock_write,
	.delay_us = mock_delay,
};

static void setup_bus(struct mock_bus *bus)
{
	memset(bus, 0, sizeof(*bus));
	bus->accel[0x00] = 0x1e;
	bus->gyro[0x00] = 0x0f;
	bus->accel[0x41] = 0xa4;
	bus->gyro[0x10] = 0x80;
}

static void test_default_init(void)
{
	struct mock_bus bus;
	struct bmi088 dev;
	struct imu_dev imu = { .name = "bmi088", .ops = &bmi088_imu_ops,
				.priv = &dev };
	int ret;

	setup_bus(&bus);
	memset(&dev, 0, sizeof(dev));

	ret = bmi088_init(&imu, &mock_ops, &bus, NULL);

	EXPECT_EQ(ret, 0);
	EXPECT_EQ(dev.initialized, 1);
	EXPECT_EQ(bus.write_count, EXPECTED_WRITES);
	EXPECT_EQ(bus.writes[0].reg, 0x7c);
	EXPECT_EQ(bus.writes[0].value, 0x03);
	EXPECT_EQ(bus.writes[1].reg, 0x7d);
	EXPECT_EQ(bus.writes[1].value, 0x00);
	EXPECT_EQ(bus.writes[2].reg, 0x7e);
	EXPECT_EQ(bus.writes[2].value, 0xb6);
	EXPECT_EQ(bus.writes[3].reg, 0x14);
	EXPECT_EQ(bus.writes[3].value, 0xb6);
	EXPECT_EQ(bus.writes[6].reg, 0x41);
	EXPECT_EQ(bus.writes[6].value, 0xa5);
	EXPECT_EQ(bus.writes[7].reg, 0x40);
	EXPECT_EQ(bus.writes[7].value, 0xa8);
	EXPECT_EQ(bus.writes[8].reg, 0x0f);
	EXPECT_EQ(bus.writes[8].value, 0x00);
	EXPECT_EQ(bus.writes[9].reg, 0x10);
	EXPECT_EQ(bus.writes[9].value, 0x84);
	EXPECT_EQ(bus.writes[10].reg, 0x15);
	EXPECT_EQ(bus.writes[10].value, 0x80);
	EXPECT_EQ(bus.delays[0], 1000);
	EXPECT_EQ(bus.delays[1], 1000);
	EXPECT_EQ(bus.delays[2], 1000);
	EXPECT_EQ(bus.delays[3], 1000);
	EXPECT_EQ(bus.delays[4], 30000);
}

static void test_raw_and_scaled_reads(void)
{
	struct bmi088_vec3_raw raw;
	struct bmi088_vec3 accel;
	struct bmi088 dev;
	struct imu_dev imu = { .name = "bmi088", .ops = &bmi088_imu_ops,
				.priv = &dev };
	struct mock_bus bus;
	int ret;

	setup_bus(&bus);
	memset(&dev, 0, sizeof(dev));
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, NULL), 0);

	bus.accel[0x12] = 0x34;
	bus.accel[0x13] = 0x12;
	bus.accel[0x14] = 0x00;
	bus.accel[0x15] = 0x80;
	bus.accel[0x16] = 0xff;
	bus.accel[0x17] = 0x7f;

	ret = bmi088_read_accel_raw(&imu, &raw);
	EXPECT_EQ(ret, 0);
	EXPECT_EQ(raw.x, 4660);
	EXPECT_EQ(raw.y, -32768);
	EXPECT_EQ(raw.z, 32767);

	ret = bmi088_read_accel(&imu, &accel);
	EXPECT_EQ(ret, 0);
	EXPECT_TRUE(fabsf(accel.x - 8.3672f) < 0.001f);
	EXPECT_TRUE(fabsf(accel.y + 58.8399f) < 0.001f);
}

static void test_temperature(void)
{
	struct bmi088 dev;
	struct imu_dev imu = { .name = "bmi088", .ops = &bmi088_imu_ops,
				.priv = &dev };
	struct mock_bus bus;
	float temperature = 99.0f;

	setup_bus(&bus);
	memset(&dev, 0, sizeof(dev));
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, NULL), 0);

	bus.accel[0x22] = 0x00;
	bus.accel[0x23] = 0x60;
	EXPECT_EQ(bmi088_read_temperature(&imu, &temperature), 0);
	EXPECT_TRUE(fabsf(temperature - 23.375f) < 0.001f);

	bus.accel[0x22] = 0xc1;
	bus.accel[0x23] = 0x00;
	EXPECT_EQ(bmi088_read_temperature(&imu, &temperature), 0);
	EXPECT_TRUE(fabsf(temperature - (-40.0f)) < 0.001f);

	bus.accel[0x22] = 0x80;
	bus.accel[0x23] = 0x00;
	EXPECT_EQ(bmi088_read_temperature(&imu, &temperature), 0);
	EXPECT_TRUE(isnan(temperature));
}

static void test_errors_do_not_change_output(void)
{
	struct bmi088_vec3_raw raw = { 1, 2, 3 };
	struct bmi088_config invalid;
	struct bmi088 dev;
	struct imu_dev imu = { .name = "bmi088", .ops = &bmi088_imu_ops,
				.priv = &dev };
	struct mock_bus bus;

	setup_bus(&bus);
	memset(&dev, 0, sizeof(dev));
	invalid = bmi088_default_config();
	invalid.accel_range = (enum bmi088_accel_range)99;
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, &invalid), -EINVAL);
	EXPECT_EQ(bus.write_count, 0);

	setup_bus(&bus);
	bus.accel[0x00] = 0x00;
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, NULL), -ENODEV);
	EXPECT_EQ(dev.initialized, 0);
	EXPECT_TRUE(!strcmp(stub_log_last, "E imu: bmi088: "
			    "accel CHIP_ID=0x00, expected 0x1e"));

	setup_bus(&bus);
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, NULL), 0);
	bus.fail_read = -ETIMEDOUT;
	EXPECT_EQ(bmi088_read_accel_raw(&imu, &raw), -ETIMEDOUT);
	EXPECT_EQ(raw.x, 1);
	EXPECT_EQ(raw.y, 2);
	EXPECT_EQ(raw.z, 3);
}

static void test_imu_read(void)
{
	struct bmi088 dev;
	struct imu_dev imu = { .name = "bmi088", .ops = &bmi088_imu_ops,
				.priv = &dev };
	struct mock_bus bus;
	struct imu_sample s;

	setup_bus(&bus);
	memset(&dev, 0, sizeof(dev));
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, NULL), 0);

	/* x = 4660, y = -32768 on both dies; 23.375 C. */
	bus.accel[0x12] = 0x34;
	bus.accel[0x13] = 0x12;
	bus.accel[0x14] = 0x00;
	bus.accel[0x15] = 0x80;
	memcpy(&bus.gyro[0x02], &bus.accel[0x12], 6);
	bus.accel[0x22] = 0x00;
	bus.accel[0x23] = 0x60;

	/* Already SI: m/s^2 at +/-6 g, rad/s at +/-2000 dps. */
	EXPECT_EQ(imu_read(&imu, &s), 0);
	EXPECT_TRUE(fabsf(s.accel[0] - 8.3672f) < 0.001f);
	EXPECT_TRUE(fabsf(s.accel[1] + 58.8399f) < 0.001f);
	EXPECT_TRUE(fabsf(s.gyro[0] - 4.9641f) < 0.001f);
	EXPECT_TRUE(fabsf(s.gyro[1] + 34.9066f) < 0.001f);
	EXPECT_TRUE(fabsf(s.temp_c - 23.375f) < 0.001f);

	/* An invalid temperature is NaN; the rest of the sample stands. */
	bus.accel[0x22] = 0x80;
	bus.accel[0x23] = 0x00;
	EXPECT_EQ(imu_read(&imu, &s), 0);
	EXPECT_TRUE(isnan(s.temp_c));
	EXPECT_TRUE(fabsf(s.accel[0] - 8.3672f) < 0.001f);

	/* Before init completes there is nothing to read. */
	dev.initialized = 0;
	EXPECT_EQ(imu_read(&imu, &s), -ENODEV);
}

static void test_data_ready(void)
{
	struct bmi088 dev;
	struct imu_dev imu = { .name = "bmi088", .ops = &bmi088_imu_ops,
				.priv = &dev };
	struct mock_bus bus;
	uint8_t ready = 0xff;

	setup_bus(&bus);
	memset(&dev, 0, sizeof(dev));
	EXPECT_EQ(bmi088_init(&imu, &mock_ops, &bus, NULL), 0);

	bus.accel[0x03] = 0x80;
	bus.gyro[0x0a] = 0x00;
	EXPECT_EQ(bmi088_data_ready(&imu, &ready), 0);
	EXPECT_EQ(ready, BMI088_DATA_READY_ACCEL);
}

int main(void)
{
	test_default_init();
	test_raw_and_scaled_reads();
	test_temperature();
	test_errors_do_not_change_output();
	test_imu_read();
	test_data_ready();

	if (tests_failed) {
		fprintf(stderr, "%d test assertion(s) failed\n", tests_failed);
		return 1;
	}

	printf("all bmi088 core checks passed\n");
	return 0;
}
