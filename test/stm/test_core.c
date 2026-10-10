/*
 * Host-side check of the core: imu_read() and the log macros, against a fake
 * chip whose ops->read hands back whatever the test put in it.
 *
 * The mount matrix is what is worth guarding. A transposed matrix is a
 * rotation the other way, and with a 90 degree mount that still produces
 * plausible numbers -- just on the wrong axes.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "imu.h"
#include "imu_log.h"

#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 1e-6)

static struct imu_sample fake_sample;
static int fake_ret;
static int fake_calls;

static int fake_read(struct imu_dev *imu, struct imu_sample *s)
{
	(void)imu;
	fake_calls++;
	if (fake_ret)
		return fake_ret;
	memcpy(s->accel, fake_sample.accel, sizeof(s->accel));
	memcpy(s->gyro, fake_sample.gyro, sizeof(s->gyro));
	s->temp_c = fake_sample.temp_c;
	return 0;
}

static const struct imu_ops fake_ops = { .read = fake_read };

static struct imu_dev fake_dev(void)
{
	struct imu_dev imu = { .name = "fake", .ops = &fake_ops };

	fake_sample = (struct imu_sample){
		.accel = { 1.0f, 2.0f, 3.0f },
		.gyro = { 0.1f, 0.2f, 0.3f },
		.temp_c = 25.0f,
	};
	fake_ret = 0;
	fake_calls = 0;
	return imu;
}

static void test_identity_when_zero(void)
{
	struct imu_dev imu = fake_dev();
	struct imu_sample s;

	/* The weak default clock: HAL_GetTick() milliseconds, as microseconds. */
	stub_tick = 123456789;
	assert(imu_read(&imu, &s) == 0);
	assert(fake_calls == 1);
	assert(s.t_us == 123456789000LL);
	assert(s.accel[0] == 1.0f && s.accel[1] == 2.0f && s.accel[2] == 3.0f);
	assert(s.gyro[0] == 0.1f && s.gyro[1] == 0.2f && s.gyro[2] == 0.3f);
	assert(s.temp_c == 25.0f);
}

static void test_mount_rotation(void)
{
	/* Chip rotated +90 degrees about Z: chip X is board Y. */
	static const int8_t rot_z[3][3] = {
		{ 0, -1, 0 },
		{ 1,  0, 0 },
		{ 0,  0, 1 },
	};
	/* Chip upside down: X and Z flip. */
	static const int8_t flip[3][3] = {
		{ -1, 0,  0 },
		{  0, 1,  0 },
		{  0, 0, -1 },
	};
	struct imu_dev imu = fake_dev();
	struct imu_sample s;

	memcpy(imu.mount, rot_z, sizeof(rot_z));
	assert(imu_read(&imu, &s) == 0);
	/* board = M * chip, rows are board axes. The transpose: (2,-1,3). */
	assert(NEAR(s.accel[0], -2.0f));
	assert(NEAR(s.accel[1], 1.0f));
	assert(NEAR(s.accel[2], 3.0f));
	assert(NEAR(s.gyro[0], -0.2f));
	assert(NEAR(s.gyro[1], 0.1f));
	assert(NEAR(s.gyro[2], 0.3f));
	assert(s.temp_c == 25.0f);	/* a scalar; nothing to rotate */

	memcpy(imu.mount, flip, sizeof(flip));
	assert(imu_read(&imu, &s) == 0);
	assert(NEAR(s.accel[0], -1.0f));
	assert(NEAR(s.accel[1], 2.0f));
	assert(NEAR(s.accel[2], -3.0f));
}

static void test_errno_passthrough(void)
{
	struct imu_dev imu = fake_dev();
	struct imu_sample s = { .temp_c = 99.0f, .t_us = 7 };

	/* Whatever the chip says comes back unchanged, and s is left alone. */
	fake_ret = -ETIMEDOUT;
	assert(imu_read(&imu, &s) == -ETIMEDOUT);
	fake_ret = -ENXIO;
	assert(imu_read(&imu, &s) == -ENXIO);
	fake_ret = -ERANGE;
	assert(imu_read(&imu, &s) == -ERANGE);
	assert(s.temp_c == 99.0f && s.t_us == 7);

	/* Nothing to call is the caller's mistake, not a chip error. */
	fake_calls = 0;
	assert(imu_read(NULL, &s) == -EINVAL);
	assert(imu_read(&imu, NULL) == -EINVAL);
	imu.ops = NULL;
	assert(imu_read(&imu, &s) == -EINVAL);
	assert(fake_calls == 0);
}

static void test_log_prefix(void)
{
	struct imu_dev imu = fake_dev();
	struct imu_sample s;

	imu_err(&imu, "WHO_AM_I=0x%02x, expected 0x%02x", 0x71, 0x68);
	assert(!strcmp(stub_log_last,
		       "imu: fake: WHO_AM_I=0x71, expected 0x68"));

	/* No arguments after the format must still compile and work. */
	imu_warn(&imu, "no arguments");
	assert(!strcmp(stub_log_last, "imu: fake: no arguments"));
	imu_info(&imu, "i");
	assert(!strcmp(stub_log_last, "imu: fake: i"));
	/* test/Makefile builds with CONFIG_IMU_DEBUG=1, so this one prints. */
	imu_dbg(&imu, "d");
	assert(!strcmp(stub_log_last, "imu: fake: d"));

	stub_tick = 42;
	assert(imu_read(&imu, &s) == 0);
	imu_log_sample(&imu, &s);
	assert(!strncmp(stub_log_last, "imu: fake: t 42000 us  a ", 25));
	assert(strstr(stub_log_last, "  1.000   2.000   3.000 m/s^2"));
	assert(strstr(stub_log_last, "  0.100   0.200   0.300 rad/s"));
	assert(strstr(stub_log_last, " 25.0 C"));

	/*
	 * The timestamp is formatted by hand, because newlib-nano's printf has
	 * no %lld. Both ends of the range, and zero.
	 */
	s.t_us = 0;
	imu_log_sample(&imu, &s);
	assert(!strncmp(stub_log_last, "imu: fake: t 0 us  a ", 21));
	s.t_us = INT64_MAX;
	imu_log_sample(&imu, &s);
	assert(strstr(stub_log_last, " t 9223372036854775807 us "));
	s.t_us = INT64_MIN;
	imu_log_sample(&imu, &s);
	assert(strstr(stub_log_last, " t -9223372036854775808 us "));
	s.t_us = -1234567;
	imu_log_sample(&imu, &s);
	assert(strstr(stub_log_last, " t -1234567 us "));
}

int main(void)
{
	test_identity_when_zero();
	test_mount_rotation();
	test_errno_passthrough();
	test_log_prefix();
	printf("all core checks passed\n");
	return 0;
}
