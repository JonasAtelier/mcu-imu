/*
 * Host-side check of the register conversions:
 *
 *     make -C test test
 *
 * The sample-rate maths gets the most attention, because DLPF_CFG quietly
 * changes the base rate the divider works from. A rate that is reachable with
 * one filter is not reachable with another, and the failure is silent.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "mpu6050_math.h"
#include "mpu6050_regs.h"

#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 1e-4 * fabs(b) + 1e-9)

static void test_sensitivity_tables(void)
{
	/* 2 g is 16384 counts per g, and each step up halves it. */
	assert(NEAR(mpu6050_accel_sensitivity_of(0), 16384.0f));
	assert(NEAR(mpu6050_accel_sensitivity_of(1), 8192.0f));
	assert(NEAR(mpu6050_accel_sensitivity_of(2), 4096.0f));
	assert(NEAR(mpu6050_accel_sensitivity_of(3), 2048.0f));
	for (int i = 0; i < 3; i++)
		assert(NEAR(mpu6050_accel_sensitivity_of(i),
			    mpu6050_accel_sensitivity_of(i + 1) * 2.0f));

	/* Zephyr carries these x10 as 1310, 655, 328, 164. */
	assert(NEAR(mpu6050_gyro_sensitivity_of(0), 131.0f));
	assert(NEAR(mpu6050_gyro_sensitivity_of(1), 65.5f));
	assert(NEAR(mpu6050_gyro_sensitivity_of(2), 32.8f));
	assert(NEAR(mpu6050_gyro_sensitivity_of(3), 16.4f));

	/* Unlike the ICM-45686, these run the intuitive way: 0 is narrowest. */
	assert(mpu6050_accel_fs_g(0) < mpu6050_accel_fs_g(3));
	assert(mpu6050_gyro_fs_dps(0) < mpu6050_gyro_fs_dps(3));
	assert(NEAR(mpu6050_accel_fs_g(0), 2.0f));
	assert(NEAR(mpu6050_gyro_fs_dps(3), 2000.0f));

	/* Sensitivity and full scale must agree: counts * fs == 32768. */
	for (int i = 0; i < 4; i++)
		assert(NEAR(mpu6050_accel_sensitivity_of(i) *
			    mpu6050_accel_fs_g(i), 32768.0f));

	/* Out of range reports 0, never a plausible number. */
	assert(mpu6050_accel_sensitivity_of(4) == 0.0f);
	assert(mpu6050_accel_sensitivity_of(-1) == 0.0f);
	assert(mpu6050_gyro_sensitivity_of(4) == 0.0f);
	assert(mpu6050_accel_fs_g(4) == 0.0f);
	assert(mpu6050_gyro_fs_dps(-1) == 0.0f);
}

static void test_conversions(void)
{
	assert(NEAR(mpu6050_accel_g(16384, 0), 1.0f));
	assert(NEAR(mpu6050_accel_g(-16384, 0), -1.0f));
	assert(NEAR(mpu6050_accel_g(16384, 3), 8.0f));
	assert(NEAR(mpu6050_gyro_dps(131, 0), 1.0f));
	assert(NEAR(mpu6050_gyro_dps(-131, 0), -1.0f));
	assert(NEAR(mpu6050_gyro_dps(164, 3), 10.0f));

	/* An invalid code must not divide by zero into a reading. */
	assert(mpu6050_accel_g(1000, 4) == 0.0f);
	assert(mpu6050_gyro_dps(1000, 9) == 0.0f);

	/*
	 * Temperature: raw/340 + 36.53. Linux carries the offset as
	 * 12420/340, which is 36.5294..., not the 36.0 Zephyr rounds it to.
	 */
	assert(NEAR(mpu6050_temp_c(0), 36.53f));
	assert(NEAR(mpu6050_temp_c(-12420), 36.53f - 12420.0f / 340.0f));
	assert(fabs(mpu6050_temp_c(-12420)) < 0.01f);
	assert(NEAR(mpu6050_temp_c(340), 37.53f));
	assert(NEAR(mpu6050_temp_c(-340), 35.53f));
}

static void test_base_rate_follows_the_filter(void)
{
	/* 8 kHz at the two ends, 1 kHz for every filter in between. */
	assert(NEAR(mpu6050_gyro_output_rate(0), 8000.0f));
	assert(NEAR(mpu6050_gyro_output_rate(7), 8000.0f));
	for (int f = 1; f <= 6; f++)
		assert(NEAR(mpu6050_gyro_output_rate(f), 1000.0f));

	assert(mpu6050_gyro_output_rate(-1) == 0.0f);
	assert(mpu6050_gyro_output_rate(8) == 0.0f);
}

static void test_sample_rate(void)
{
	/* base / (1 + div) */
	assert(NEAR(mpu6050_sample_rate_of(1, 0), 1000.0f));
	assert(NEAR(mpu6050_sample_rate_of(1, 9), 100.0f));
	assert(NEAR(mpu6050_sample_rate_of(1, 255), 1000.0f / 256.0f));
	assert(NEAR(mpu6050_sample_rate_of(0, 0), 8000.0f));
	assert(NEAR(mpu6050_sample_rate_of(0, 7), 1000.0f));

	/*
	 * The same divider means a different rate once the filter changes --
	 * this is the trap the API exists to close.
	 */
	assert(mpu6050_sample_rate_of(0, 9) != mpu6050_sample_rate_of(1, 9));
	assert(NEAR(mpu6050_sample_rate_of(0, 9), 800.0f));

	assert(mpu6050_sample_rate_of(8, 0) == 0.0f);
}

static void test_divider_solver(void)
{
	uint8_t div;

	assert(mpu6050_divider_for(100.0f, 1, &div) && div == 9);
	assert(mpu6050_divider_for(1000.0f, 1, &div) && div == 0);
	/* 8000/80 */
	assert(mpu6050_divider_for(100.0f, 0, &div) && div == 79);
	assert(NEAR(mpu6050_sample_rate_of(0, 79), 100.0f));

	/* Faster than the base rate is unreachable too, not clamped. */
	assert(!mpu6050_divider_for(5000.0f, 1, &div));
	assert(!mpu6050_divider_for(2000.0f, 1, &div));
	assert(!mpu6050_divider_for(1001.0f, 1, &div));
	assert(!mpu6050_divider_for(8001.0f, 0, &div));
	assert(mpu6050_divider_for(8000.0f, 0, &div) && div == 0);

	/* Slower than base/256 is genuinely unreachable and must be refused. */
	/* 1000/256 = 3.9 Hz floor, 8000/256 = 31.25 Hz floor */
	assert(!mpu6050_divider_for(3.0f, 1, &div));
	assert(!mpu6050_divider_for(10.0f, 0, &div));
	/* ...but the same 10 Hz is fine once the filter drops the base. */
	assert(mpu6050_divider_for(10.0f, 1, &div) && div == 99);

	assert(!mpu6050_divider_for(0.0f, 1, &div));
	assert(!mpu6050_divider_for(-5.0f, 1, &div));
	assert(!mpu6050_divider_for(100.0f, 9, &div));

	/* Round trip: whatever the solver picks must read back close. */
	for (float want = 5.0f; want < 900.0f; want += 7.3f)
		if (mpu6050_divider_for(want, 1, &div))
			assert(fabs(mpu6050_sample_rate_of(1, div) - want) <=
			       want * 0.5f + 1.0f);
}

static void test_sample_assembly(void)
{
	const uint8_t p[2] = { 0x12, 0x34 };
	const uint8_t neg[2] = { 0xFF, 0xFF };
	const uint8_t min[2] = { 0x80, 0x00 };
	const uint8_t max[2] = { 0x7F, 0xFF };

	/* Big endian, fixed -- no configuration bit on this part. */
	assert(mpu6050_sample(p) == 0x1234);
	assert(mpu6050_sample(neg) == -1);
	assert(mpu6050_sample(min) == -32768);
	assert(mpu6050_sample(max) == 32767);
}

static void test_register_constants(void)
{
	assert(MPU6050_WHO_AM_I == 0x75);
	assert(MPU6050_WHO_AM_I_VALUE == 0x68);
	assert(MPU6050_ACCEL_XOUT_H == 0x3B);
	/* Temperature is in the MIDDLE of the burst, not at the end. */
	assert(MPU6050_TEMP_OFF == 6);
	assert(MPU6050_GYRO_OFF == 8);
	assert(MPU6050_BURST_LEN == 14);
	/* The gyro block starts at 0x43, which is XOUT_H + 8. */
	assert(MPU6050_ACCEL_XOUT_H + MPU6050_GYRO_OFF == 0x43);
	assert(MPU6050_FS_SHIFT == 3);
	assert(MPU6050_SLEEP == 0x40 && MPU6050_DEVICE_RESET == 0x80);
	assert(MPU6050_I2C_BYPASS_EN == 0x02);
	assert(MPU6050_ADDR_LOW == 0x68 && MPU6050_ADDR_HIGH == 0x69);
}

int main(void)
{
	test_sensitivity_tables();
	test_conversions();
	test_base_rate_follows_the_filter();
	test_sample_rate();
	test_divider_solver();
	test_sample_assembly();
	test_register_constants();
	printf("all mpu6050 math checks passed\n");
	return 0;
}
