/*
 * The JY901 driver tests, shared by test/esp/test_jy901.c and
 * test/stm/test_jy901.c. Each of those supplies only the glue from its
 * platform's UART and I2C calls to the fake module below, so the module and
 * the cases are written once and run through both ports; each glue adds its
 * own platform checks after run_cases(), and says in DEVICES_PER_PROBE how
 * many bus devices an I2C probe adds: one on ESP-IDF, none on the HAL.
 *
 * The fake has two faces. On the UART it is a scripted line: everything the
 * module will send, in order. Bytes before `fresh_at` are stale -- already
 * waiting when a read starts -- and dropping stale input moves the read
 * position past them; a read that wants more than the script holds runs
 * out, which is the port's timeout. Frames are built here with their own
 * sum, never the driver's. On I2C it is a file of 16-bit registers, read
 * and written low byte first and running on to the next register (§7.1,
 * §7.2), and any one transfer can be made to fail.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "imu.h"
#include "jy901.h"

static uint8_t line[2048];
static size_t line_len;		/* bytes scripted */
static size_t line_pos;		/* next byte a read takes */
static size_t fresh_at;		/* bytes before this are stale */
static uint8_t sent[64];	/* everything the driver sent on the UART */
static size_t sent_len;
static int allocs, frees;

#define CHIP_REGISTERS	0x60
static uint16_t chip_registers[CHIP_REGISTERS];
static bool i2c_absent;			/* nothing ACKs */
static uint8_t i2c_address_seen;	/* 7-bit, as the glue saw it */
static int i2c_transfers;		/* reads and writes so far */
static int i2c_fail_at;			/* this transfer fails; 0 none */
static int i2c_writes;
static uint8_t last_read_register;
static size_t last_read_len;
static uint8_t last_write_register;
static uint8_t last_write[8];
static size_t last_write_len;
static int devices;			/* added and not removed, by the glue */

static void line_reset(void)
{
	line_len = line_pos = fresh_at = 0;
	sent_len = 0;
}

static void put(const uint8_t *b, size_t n)
{
	assert(line_len + n <= sizeof(line));
	memcpy(line + line_len, b, n);
	line_len += n;
}

/* Everything scripted so far was waiting before the read began. */
static void mark_fresh(void)
{
	fresh_at = line_len;
}

/* §6.1: 0x55, type, four values low byte first, SUM (+ sum_error). */
static void put_frame_sum(uint8_t type, int16_t v0, int16_t v1, int16_t v2,
			  int16_t v3, int sum_error)
{
	const int16_t v[4] = { v0, v1, v2, v3 };
	uint8_t f[11] = { 0x55, type };
	unsigned int sum = 0;

	for (int i = 0; i < 4; i++) {
		f[2 + 2 * i] = (uint8_t)((uint16_t)v[i] & 0xFF);
		f[3 + 2 * i] = (uint8_t)((uint16_t)v[i] >> 8);
	}
	for (int i = 0; i < 10; i++)
		sum += f[i];
	f[10] = (uint8_t)(sum + (unsigned int)sum_error);
	put(f, sizeof(f));
}

static void put_frame(uint8_t type, int16_t v0, int16_t v1, int16_t v2,
		      int16_t v3)
{
	put_frame_sum(type, v0, v1, v2, v3, 0);
}

/* The default cycle, §6.2.8: 0x51, 0x52, 0x53, 0x54. */
static void put_cycle(int16_t ax, int16_t gz, int16_t yaw, int16_t temp)
{
	put_frame(0x51, ax, 0, 0, temp);
	put_frame(0x52, 0, 0, gz, temp);
	put_frame(0x53, 0, 0, yaw, temp);
	put_frame(0x54, 100, 200, 300, temp);
}

/* A module at 0x50 at rest: IICADDR set, every other register zero. */
static void chip_reset(void)
{
	memset(chip_registers, 0, sizeof(chip_registers));
	chip_registers[0x1a] = 0x0050;
	i2c_absent = false;
	i2c_transfers = i2c_fail_at = i2c_writes = 0;
	last_read_len = last_write_len = 0;
}

/* --- for the glue: inline, since each glue uses only some -------------- */

static inline void line_drop_stale(void)
{
	if (line_pos < fresh_at)
		line_pos = fresh_at;
}

static inline int line_has_stale(void)
{
	return line_pos < fresh_at;
}

/* Up to n bytes; fewer means the line ran dry. */
static inline size_t line_take(uint8_t *dst, size_t n)
{
	size_t left = line_len - line_pos;

	if (n > left)
		n = left;
	memcpy(dst, line + line_pos, n);
	line_pos += n;
	return n;
}

static inline void line_sent(const uint8_t *b, size_t n)
{
	assert(sent_len + n <= sizeof(sent));
	memcpy(sent + sent_len, b, n);
	sent_len += n;
}

/* Register reg onward into rx, low byte first; false: the transfer fails. */
static inline bool chip_i2c_read(uint8_t reg, uint8_t *rx, size_t len)
{
	if (++i2c_transfers == i2c_fail_at)
		return false;
	assert(reg + (len + 1) / 2 <= CHIP_REGISTERS);
	last_read_register = reg;
	last_read_len = len;
	for (size_t i = 0; i < len; i++) {
		uint16_t value = chip_registers[reg + i / 2];

		rx[i] = (uint8_t)(i % 2 ? value >> 8 : value & 0xFF);
	}
	return true;
}

static inline bool chip_i2c_write(uint8_t reg, const uint8_t *tx, size_t len)
{
	if (++i2c_transfers == i2c_fail_at)
		return false;
	assert(len <= sizeof(last_write) && len % 2 == 0);
	i2c_writes++;
	last_write_register = reg;
	last_write_len = len;
	memcpy(last_write, tx, len);
	for (size_t i = 0; i < len; i += 2)
		chip_registers[reg + i / 2] =
			(uint16_t)(tx[i] | (uint16_t)tx[i + 1] << 8);
	return true;
}

/* Counts the driver's calloc() and free() calls; see test/Makefile. */
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
	if (p)
		frees++;
	__real_free(p);
}

/* --- UART cases ---------------------------------------------------------- */

static int near(float a, float b)
{
	return fabsf(a - b) < 1e-3f;
}

static struct imu_dev probe_uart_ok(void)
{
	struct imu_dev imu = { 0 };

	line_reset();
	put_cycle(0, 0, 0, 2500);
	assert(jy901_probe_uart(&imu, TEST_UART, NULL) == 0);
	assert(strcmp(imu.name, "jy901") == 0);
	assert(strstr(stub_log_last, "streaming on UART"));
	return imu;
}

static void test_uart_probe(void)
{
	struct imu_dev imu;

	allocs = frees = 0;
	imu = probe_uart_ok();
	assert(allocs == 1 && sent_len == 0);	/* nothing written */
	assert(jy901_probe_uart(&imu, TEST_UART, NULL) == -EBUSY);
	jy901_remove(&imu);
	assert(!imu.ops && !imu.priv && frees == 1);

	/* Silence: -ENXIO, nothing allocated. */
	allocs = frees = 0;
	line_reset();
	assert(jy901_probe_uart(&imu, TEST_UART, NULL) == -ENXIO);
	assert(allocs == 0 && !imu.ops && !imu.priv);

	/* Bytes, but never a frame whose SUM matches: -EBADMSG. */
	line_reset();
	for (int i = 0; i < 30; i++)
		put_frame_sum(0x51, 1, 2, 3, 4, 1);
	assert(jy901_probe_uart(&imu, TEST_UART, NULL) == -EBADMSG);
	assert(allocs == 0);

	/* Valid frames, but the gyroscope output is off: -ENODATA. */
	line_reset();
	for (int i = 0; i < 15; i++) {
		put_frame(0x51, 1, 2, 3, 4);
		put_frame(0x53, 1, 2, 3, 4);
	}
	assert(jy901_probe_uart(&imu, TEST_UART, NULL) == -ENODATA);
	assert(allocs == 0);

	/* A stream that stops part way is the port's timeout. */
	line_reset();
	put_frame(0x51, 1, 2, 3, 4);
	put((const uint8_t *)"\x55\x52\x00", 3);
	assert(jy901_probe_uart(&imu, TEST_UART, NULL) == -ETIMEDOUT);
	assert(allocs == 0);

	assert(jy901_probe_uart(NULL, TEST_UART, NULL) == -EINVAL);
}

static void test_uart_read(void)
{
	struct imu_dev imu = probe_uart_ok();
	struct imu_sample s;

	/* 1 g on X, -16 g on Z; 1000 deg/s on Y; 25.00 degC. Stale first. */
	put_frame(0x51, 4096, 0, 0, 0);
	put_frame(0x52, 0, 0, 0, 0);
	mark_fresh();
	put_frame(0x51, 2048, 0, -32768, 2500);
	put_frame(0x52, 0, 16384, 0, 2500);
	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[0], 9.80665f) && near(s.accel[1], 0.0f));
	assert(fabsf(s.accel[2] + 16.0f * 9.80665f) < 1e-2f);
	assert(near(s.gyro[1], 1000.0f * 0.017453292519943295f));
	assert(near(s.temp_c, 25.0f));

	/* Gyroscope before accelerometer, other types in between. */
	mark_fresh();
	put_frame(0x50, 1, 2, 3, 4);
	put_frame(0x52, 0, 0, 16384, 0);
	put_frame(0x59, 1, 2, 3, 4);
	put_frame(0x51, 0, 2048, 0, 100);
	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[1], 9.80665f) && near(s.temp_c, 1.0f));
	assert(near(s.gyro[2], 1000.0f * 0.017453292519943295f));
	jy901_remove(&imu);
}

static void test_uart_sync(void)
{
	static const uint8_t tail[5] = { 0x00, 0x12, 0x34, 0x56, 0x78 };
	static const uint8_t noise[] = {
		0x55, 0x51, 0x01,		/* a false header, cut short */
		0x55, 0x55, 0x55,		/* headers with no frame */
		0xAA, 0x55, 0x99, 0x55,		/* 0x99 is not a type */
	};
	struct imu_dev imu = probe_uart_ok();
	struct imu_sample s;

	/* The read starts in the middle of a frame. */
	mark_fresh();
	put(tail, sizeof(tail));
	put_frame(0x51, 2048, 0, 0, 0);
	put_frame(0x52, 0, 0, 0, 0);
	assert(imu_read(&imu, &s) == 0 && near(s.accel[0], 9.80665f));

	/* Noise full of 0x55s before the real frames. */
	mark_fresh();
	put(noise, sizeof(noise));
	put_frame(0x51, 0, 0, 2048, 0);
	put_frame(0x52, 0, 0, 0, 0);
	assert(imu_read(&imu, &s) == 0 && near(s.accel[2], 9.80665f));

	/* A bad-SUM frame whose data starts 0x55 0x51 is skipped. */
	mark_fresh();
	put_frame_sum(0x51, 0x5155, 0, 0, 0, 7);
	put_frame(0x51, 4096, 0, 0, 0);
	put_frame(0x52, 0, 0, 0, 0);
	assert(imu_read(&imu, &s) == 0 && near(s.accel[0], 2 * 9.80665f));

	/* Nothing but bad SUMs: -EBADMSG. */
	mark_fresh();
	for (int i = 0; i < 30; i++)
		put_frame_sum(0x52, 1, 2, 3, 4, 1);
	assert(imu_read(&imu, &s) == -EBADMSG);

	/* The stream stops: -ETIMEDOUT, and the sample is untouched. */
	mark_fresh();
	put_frame(0x51, 0, 0, 0, 0);
	s.accel[0] = 42.0f;
	assert(imu_read(&imu, &s) == -ETIMEDOUT);
	assert(s.accel[0] == 42.0f);
	jy901_remove(&imu);
}

static void test_uart_extras(void)
{
	struct imu_dev imu = probe_uart_ok();
	float angles[3];

	/* §6.1.4: x 90, y -45, z 180 - one step. */
	mark_fresh();
	put_frame(0x51, 0, 0, 0, 0);
	put_frame(0x53, 16384, -8192, 32767, 0);
	assert(jy901_read_angles(&imu, angles) == 0);
	assert(near(angles[0], 90.0f) && near(angles[1], -45.0f));
	assert(fabsf(angles[2] - 179.9945f) < 1e-3f);

	/* Angle output off: -ENODATA. */
	mark_fresh();
	for (int i = 0; i < 15; i++) {
		put_frame(0x51, 0, 0, 0, 0);
		put_frame(0x52, 0, 0, 0, 0);
	}
	assert(jy901_read_angles(&imu, angles) == -ENODATA);

	/* §6.2: 0xFF 0xAA address dataL dataH, e.g. RATE = 0x06 (§6.2.9). */
	sent_len = 0;
	assert(jy901_command(&imu, 0x03, 0x0006) == 0);
	assert(sent_len == 5 && sent[0] == 0xFF && sent[1] == 0xAA &&
	       sent[2] == 0x03 && sent[3] == 0x06 && sent[4] == 0x00);
	jy901_remove(&imu);
}

/* --- I2C cases ----------------------------------------------------------- */

static struct jy901_i2c_config i2c_config(uint8_t address)
{
	struct jy901_i2c_config config = { .bus = TEST_I2C };

	config.address = address;
	return config;
}

static struct imu_dev probe_i2c_ok(void)
{
	struct jy901_i2c_config config = i2c_config(0);
	struct imu_dev imu = { 0 };

	chip_reset();
	assert(jy901_probe_i2c(&imu, &config) == 0);
	assert(strcmp(imu.name, "jy901") == 0);
	assert(strstr(stub_log_last, "up on I2C at 0x50"));
	return imu;
}

static void test_i2c_probe(void)
{
	struct jy901_i2c_config config = i2c_config(0);
	struct imu_dev imu;

	/* 0x50 by default; IICADDR, then one burst; nothing written. */
	allocs = frees = 0;
	imu = probe_i2c_ok();
	assert(allocs == 1 && i2c_writes == 0 &&
	       devices == DEVICES_PER_PROBE);
	assert(i2c_address_seen == 0x50);
	assert(i2c_transfers == 2 && last_read_register == 0x34 &&
	       last_read_len == 26);
	assert(jy901_probe_i2c(&imu, &config) == -EBUSY);
	jy901_remove(&imu);
	assert(!imu.ops && !imu.priv && frees == 1 && devices == 0);

	/* Nothing at the address: -ENXIO, nothing allocated or added. */
	allocs = frees = 0;
	chip_reset();
	i2c_absent = true;
	assert(jy901_probe_i2c(&imu, &config) == -ENXIO);
	assert(allocs == 0 && devices == 0 && !imu.ops && !imu.priv);

	/* Something at 0x50 whose 0x1a is not 0x0050: -ENODEV. */
	chip_reset();
	chip_registers[0x1a] = 0x5051;
	assert(jy901_probe_i2c(&imu, &config) == -ENODEV);
	assert(allocs == 0 && devices == 0 && i2c_transfers == 1);
	chip_reset();
	chip_registers[0x1a] = 0x1250;		/* a nonzero high byte */
	assert(jy901_probe_i2c(&imu, &config) == -ENODEV);

	/* Another address, which IICADDR must then hold. */
	chip_reset();
	chip_registers[0x1a] = 0x0051;
	config = i2c_config(0x51);
	assert(jy901_probe_i2c(&imu, &config) == 0);
	assert(i2c_address_seen == 0x51);
	jy901_remove(&imu);
	chip_reset();
	assert(jy901_probe_i2c(&imu, &config) == -ENODEV);

	/* A bus error at either transfer ends it, nothing bound or leaked. */
	config = i2c_config(0);
	for (int fail = 1; fail <= 2; fail++) {
		chip_reset();
		i2c_fail_at = fail;
		assert(jy901_probe_i2c(&imu, &config) == -ETIMEDOUT);
		assert(allocs == frees && devices == 0);
		assert(!imu.ops && !imu.priv);
	}

	/* Refused before any traffic. */
	chip_reset();
	config = i2c_config(0x80);
	assert(jy901_probe_i2c(&imu, &config) == -EINVAL);
	assert(jy901_probe_i2c(&imu, NULL) == -EINVAL);
	assert(jy901_probe_i2c(NULL, &config) == -EINVAL);
	config.bus = NULL;
	config.address = 0;
	assert(jy901_probe_i2c(&imu, &config) == -EINVAL);
	assert(i2c_transfers == 0 && allocs == frees);
}

static void test_i2c_read(void)
{
	struct imu_dev imu = probe_i2c_ok();
	struct imu_sample s;

	/*
	 * AX..AZ 1 g, 0, -16 g; GX..GZ 0, 1000 deg/s, -500 deg/s; then
	 * magnetometer and angles with values of their own, so a wrong offset
	 * cannot land on the right number; TEMP 25.00 degC.
	 */
	chip_registers[0x34] = 2048;
	chip_registers[0x35] = 0;
	chip_registers[0x36] = (uint16_t)-32768;
	chip_registers[0x37] = 0;
	chip_registers[0x38] = 16384;
	chip_registers[0x39] = (uint16_t)-8192;
	for (int reg = 0x3a; reg <= 0x3f; reg++)
		chip_registers[reg] = (uint16_t)(0x1111 * (reg - 0x39));
	chip_registers[0x40] = 2500;

	i2c_transfers = 0;
	assert(imu_read(&imu, &s) == 0);
	assert(i2c_transfers == 1);		/* one burst, one moment */
	assert(last_read_register == 0x34 && last_read_len == 26);
	assert(near(s.accel[0], 9.80665f) && near(s.accel[1], 0.0f));
	assert(fabsf(s.accel[2] + 16.0f * 9.80665f) < 1e-2f);
	assert(near(s.gyro[0], 0.0f));
	assert(near(s.gyro[1], 1000.0f * 0.017453292519943295f));
	assert(near(s.gyro[2], -500.0f * 0.017453292519943295f));
	assert(near(s.temp_c, 25.0f));

	/* A failed burst leaves the sample untouched. */
	i2c_transfers = 0;
	i2c_fail_at = 1;
	s.accel[0] = 42.0f;
	assert(imu_read(&imu, &s) == -ETIMEDOUT);
	assert(s.accel[0] == 42.0f);
	i2c_fail_at = 0;
	jy901_remove(&imu);
}

static void test_i2c_extras(void)
{
	struct imu_dev imu = probe_i2c_ok(), other = { 0 };
	float angles[3];

	/* §7.2 p.49's own example: -176.33, -1.19, 51.89 degrees. */
	chip_registers[0x3d] = 0x829C;
	chip_registers[0x3e] = 0xFF28;
	chip_registers[0x3f] = 0x24E6;
	assert(jy901_read_angles(&imu, angles) == 0);
	assert(last_read_register == 0x3d && last_read_len == 6);
	assert(fabsf(angles[0] + 176.33f) < 5e-3f);
	assert(fabsf(angles[1] + 1.19f) < 5e-3f);
	assert(fabsf(angles[2] - 51.89f) < 5e-3f);

	/* §7.1: a register write, dataL then dataH; nothing on the UART. */
	line_reset();
	assert(jy901_command(&imu, 0x03, 0x0006) == 0);
	assert(i2c_writes == 1 && last_write_register == 0x03 &&
	       last_write_len == 2 && last_write[0] == 0x06 &&
	       last_write[1] == 0x00 && sent_len == 0);
	assert(jy901_command(&imu, 0x02, 0x011E) == 0);
	assert(chip_registers[0x02] == 0x011E);

	/* Someone else's device, or none. */
	assert(jy901_read_angles(&other, angles) == -EINVAL);
	assert(jy901_command(NULL, 0, 0) == -EINVAL);
	assert(jy901_read_angles(&imu, NULL) == -EINVAL);
	jy901_remove(&other);		/* safe on an unprobed device */
	jy901_remove(&imu);
	assert(allocs == frees && devices == 0);
}

static void run_cases(void)
{
	test_uart_probe();
	test_uart_read();
	test_uart_sync();
	test_uart_extras();
	test_i2c_probe();
	test_i2c_read();
	test_i2c_extras();
}
