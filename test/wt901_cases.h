/*
 * The WT901 driver tests, shared by test/esp/test_wt901.c and
 * test/stm/test_wt901.c. Each of those supplies only the glue from its
 * platform's UART calls to the fake line below, so the module and the cases
 * are written once and run through both ports.
 *
 * The fake is a scripted line: everything the module will send, in order.
 * Bytes before `fresh_at` are stale -- already waiting when a read starts --
 * and dropping stale input moves the read position past them. A read that
 * wants more than the script still holds runs out, which is the port's
 * timeout. Frames are built here with their own sum, never the driver's.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "imu.h"
#include "wt901.h"

static uint8_t line[2048];
static size_t line_len;		/* bytes scripted */
static size_t line_pos;		/* next byte a read takes */
static size_t fresh_at;		/* bytes before this are stale */
static uint8_t sent[64];	/* everything the driver sent */
static size_t sent_len;
static int allocs, frees;

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
	frees++;
	__real_free(p);
}

/* --- cases -------------------------------------------------------------- */

static int near(float a, float b)
{
	return fabsf(a - b) < 1e-3f;
}

static struct imu_dev probe_ok(void)
{
	struct imu_dev imu = { 0 };

	line_reset();
	put_cycle(0, 0, 0, 2500);
	assert(wt901_probe(&imu, TEST_UART, NULL) == 0);
	assert(strcmp(imu.name, "wt901") == 0);
	assert(strstr(stub_log_last, "streaming on UART"));
	return imu;
}

static void test_probe(void)
{
	struct imu_dev imu;

	allocs = frees = 0;
	imu = probe_ok();
	assert(allocs == 1 && sent_len == 0);	/* nothing written */
	assert(wt901_probe(&imu, TEST_UART, NULL) == -EBUSY);
	wt901_remove(&imu);
	assert(!imu.ops && !imu.priv && frees == 1);

	/* Silence: -ENXIO, nothing allocated. */
	allocs = frees = 0;
	line_reset();
	assert(wt901_probe(&imu, TEST_UART, NULL) == -ENXIO);
	assert(allocs == 0 && !imu.ops && !imu.priv);

	/* Bytes, but never a frame whose SUM matches: -EBADMSG. */
	line_reset();
	for (int i = 0; i < 30; i++)
		put_frame_sum(0x51, 1, 2, 3, 4, 1);
	assert(wt901_probe(&imu, TEST_UART, NULL) == -EBADMSG);
	assert(allocs == 0);

	/* Valid frames, but the gyroscope output is off: -ENODATA. */
	line_reset();
	for (int i = 0; i < 15; i++) {
		put_frame(0x51, 1, 2, 3, 4);
		put_frame(0x53, 1, 2, 3, 4);
	}
	assert(wt901_probe(&imu, TEST_UART, NULL) == -ENODATA);
	assert(allocs == 0);

	/* A stream that stops part way is the port's timeout. */
	line_reset();
	put_frame(0x51, 1, 2, 3, 4);
	put((const uint8_t *)"\x55\x52\x00", 3);
	assert(wt901_probe(&imu, TEST_UART, NULL) == -ETIMEDOUT);
	assert(allocs == 0);

	assert(wt901_probe(NULL, TEST_UART, NULL) == -EINVAL);

	/* A negative timeout: refused with a good stream on the line. */
	{
		const struct wt901_config negative = { .timeout_ms = -1 };

		line_reset();
		put_cycle(0, 0, 0, 2500);
		assert(wt901_probe(&imu, TEST_UART, &negative) == -EINVAL);
		assert(allocs == 0 && !imu.ops);
	}
}

static void test_scales(void)
{
	struct imu_dev imu = probe_ok();
	struct imu_sample s;

	/* 1 g on X, -16 g on Z; 1000 deg/s on Y; 25.00 degC. */
	put_frame(0x51, 2048, 0, -32768, 2500);
	put_frame(0x52, 0, 16384, 0, 2500);
	mark_fresh();
	put_frame(0x51, 2048, 0, -32768, 2500);
	put_frame(0x52, 0, 16384, 0, 2500);
	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[0], 9.80665f) && near(s.accel[1], 0.0f));
	assert(fabsf(s.accel[2] + 16.0f * 9.80665f) < 1e-2f);
	assert(near(s.gyro[1], 1000.0f * 0.017453292519943295f));
	assert(near(s.temp_c, 25.0f));
	wt901_remove(&imu);
}

static void test_stale_and_order(void)
{
	struct imu_dev imu = probe_ok();
	struct imu_sample s;

	/* What waited before the read is dropped; the fresh pair counts. */
	put_cycle(2048, 100, 0, 0);
	put_cycle(2048, 100, 0, 0);
	mark_fresh();
	put_cycle(4096, 200, 0, 0);
	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[0], 2.0f * 9.80665f));

	/* Gyroscope before accelerometer, other types in between. */
	mark_fresh();
	put_frame(0x50, 1, 2, 3, 4);
	put_frame(0x52, 0, 0, 16384, 0);
	put_frame(0x59, 1, 2, 3, 4);
	put_frame(0x54, 1, 2, 3, 4);
	put_frame(0x51, 0, 2048, 0, 100);
	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[1], 9.80665f) && near(s.temp_c, 1.0f));
	assert(near(s.gyro[2], 1000.0f * 0.017453292519943295f));
	wt901_remove(&imu);
}

static void test_sync(void)
{
	static const uint8_t tail[5] = { 0x00, 0x12, 0x34, 0x56, 0x78 };
	static const uint8_t noise[] = {
		0x55, 0x51, 0x01,		/* a false header, cut short */
		0x55, 0x55, 0x55,		/* headers with no frame */
		0xAA, 0x55, 0x99, 0x55,		/* 0x99 is not a type */
	};
	struct imu_dev imu = probe_ok();
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

	/*
	 * A frame with a bad SUM, whose data even starts 0x55 0x51, is
	 * skipped, and the real frame after it is found.
	 */
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
	wt901_remove(&imu);
}

static void test_extras(void)
{
	struct imu_dev imu = probe_ok(), other = { 0 };
	float rpy[3];

	/* §6.1.4: x 90, y -45, z 180 - one step. */
	mark_fresh();
	put_frame(0x51, 0, 0, 0, 0);
	put_frame(0x53, 16384, -8192, 32767, 0);
	assert(wt901_read_angles(&imu, rpy) == 0);
	assert(near(rpy[0], 90.0f) && near(rpy[1], -45.0f));
	assert(fabsf(rpy[2] - 179.9945f) < 1e-3f);

	/* Angle output off: -ENODATA. */
	mark_fresh();
	for (int i = 0; i < 15; i++) {
		put_frame(0x51, 0, 0, 0, 0);
		put_frame(0x52, 0, 0, 0, 0);
	}
	assert(wt901_read_angles(&imu, rpy) == -ENODATA);

	/* §6.2: 0xFF 0xAA address dataL dataH, e.g. RATE = 0x06 (§6.2.9). */
	sent_len = 0;
	assert(wt901_command(&imu, 0x03, 0x0006) == 0);
	assert(sent_len == 5 && sent[0] == 0xFF && sent[1] == 0xAA &&
	       sent[2] == 0x03 && sent[3] == 0x06 && sent[4] == 0x00);
	sent_len = 0;
	assert(wt901_command(&imu, 0x02, 0x011E) == 0);
	assert(sent[3] == 0x1E && sent[4] == 0x01);

	/* Someone else's device, or none. */
	assert(wt901_read_angles(&other, rpy) == -EINVAL);
	assert(wt901_command(NULL, 0, 0) == -EINVAL);
	assert(wt901_read_angles(&imu, NULL) == -EINVAL);
	wt901_remove(&other);		/* safe on an unprobed device */
	wt901_remove(&imu);
	assert(allocs == frees);
}

int main(void)
{
	test_probe();
	test_scales();
	test_stale_and_order();
	test_sync();
	test_extras();
	printf("all wt901 driver checks passed\n");
	return 0;
}
