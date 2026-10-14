/*
 * src/bmi270/bmi270_math.h against the numbers in the BMI270 datasheet,
 * BST-BMI270-DS000-08 rev. 1.6. No platform; runs once.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "bmi270_math.h"

static int near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* Table 2: 16384, 8192, 4096, 2048 LSB/g for 2, 4, 8, 16 g. */
static void test_accel_scale(void)
{
	assert(bmi270_accel_lsb_per_g(0) == 16384.0f);
	assert(bmi270_accel_lsb_per_g(1) == 8192.0f);
	assert(bmi270_accel_lsb_per_g(2) == 4096.0f);
	assert(bmi270_accel_lsb_per_g(3) == 2048.0f);
	assert(bmi270_accel_lsb_per_g(4) == 0.0f);
	assert(bmi270_accel_lsb_per_g(-1) == 0.0f);
	assert(bmi270_accel_range_g(0) == 2.0f);
	assert(bmi270_accel_range_g(3) == 16.0f);
	assert(bmi270_accel_range_g(4) == 0.0f);
}

/* Table 3: 16.384 .. 262.144 LSB/dps for 2000 .. 125 dps. */
static void test_gyro_scale(void)
{
	assert(near(bmi270_gyro_lsb_per_dps(0), 16.384f, 1e-6f));
	assert(near(bmi270_gyro_lsb_per_dps(1), 32.768f, 1e-6f));
	assert(near(bmi270_gyro_lsb_per_dps(2), 65.536f, 1e-5f));
	assert(near(bmi270_gyro_lsb_per_dps(3), 131.072f, 1e-5f));
	assert(near(bmi270_gyro_lsb_per_dps(4), 262.144f, 1e-4f));
	assert(bmi270_gyro_lsb_per_dps(5) == 0.0f);
	assert(bmi270_gyro_range_dps(0) == 2000.0f);
	assert(bmi270_gyro_range_dps(4) == 125.0f);
	assert(bmi270_gyro_range_dps(5) == 0.0f);
	/* full scale times sensitivity is 32768 counts */
	for (int r = 0; r <= 4; r++)
		assert(near(bmi270_gyro_range_dps(r) *
			    bmi270_gyro_lsb_per_dps(r), 32768.0f, 0.01f));
}

/* 5.2.41, 5.2.43: the rate codes; 4.6.4 and 4.6.11: which are valid. */
static void test_odr(void)
{
	assert(near(bmi270_odr_hz(0x01), 0.78125f, 1e-6f));	/* 25/32 */
	assert(bmi270_odr_hz(0x05) == 12.5f);
	assert(bmi270_odr_hz(0x06) == 25.0f);
	assert(bmi270_odr_hz(0x08) == 100.0f);
	assert(bmi270_odr_hz(0x0C) == 1600.0f);
	assert(bmi270_odr_hz(0x0D) == 3200.0f);
	assert(bmi270_odr_hz(0x00) == 0.0f && bmi270_odr_hz(0x10) == 0.0f);

	assert(!bmi270_accel_odr_valid(0x04) && bmi270_accel_odr_valid(0x05));
	assert(bmi270_accel_odr_valid(0x0C) && !bmi270_accel_odr_valid(0x0D));
	assert(!bmi270_gyro_odr_valid(0x05) && bmi270_gyro_odr_valid(0x06));
	assert(bmi270_gyro_odr_valid(0x0D) && !bmi270_gyro_odr_valid(0x0E));
}

/* 4.6.14, 5.2.34, Table 4: 0 is 23 degC, 1/512 K per LSB, 0x8000 invalid. */
static void test_temperature(void)
{
	assert(bmi270_temp_c(0) == 23.0f);
	assert(near(bmi270_temp_c(0x7FFF), 87.0f - 1.0f / 512.0f, 1e-4f));
	assert(near(bmi270_temp_c((int16_t)0x8001), -41.0f + 1.0f / 512.0f,
		    1e-4f));
	assert(bmi270_temp_c(512) == 24.0f);
	assert(isnan(bmi270_temp_c(INT16_MIN)));
}

/* 5.2.40 and 4.6.10: factor_zx is 7 bits, two's complement. */
static void test_cas(void)
{
	assert(bmi270_cas_factor(0x00) == 0);
	assert(bmi270_cas_factor(0x3F) == 63);
	assert(bmi270_cas_factor(0x40) == -64);
	assert(bmi270_cas_factor(0x7E) == -2);
	assert(bmi270_cas_factor(0x7F) == -1);
	assert(bmi270_cas_factor(0x81) == 1);	/* bit 7 is not part of it */

	/* x - factor * z / 2^9 */
	assert(bmi270_cas_correct(1000, 2048, -2) == 1008);
	assert(bmi270_cas_correct(1000, 2048, 2) == 992);
	assert(bmi270_cas_correct(1000, 0, 63) == 1000);
	/* integer division: -511 / 512 is 0, not -1 */
	assert(bmi270_cas_correct(100, -511, 1) == 100);
	assert(bmi270_cas_correct(100, -512, 1) == 101);
	/* held to 16 bits */
	assert(bmi270_cas_correct(32767, -32768, 63) == INT16_MAX);
	assert(bmi270_cas_correct(-32768, -32768, -64) == INT16_MIN);
}

/* LSB first (5.2.12-5.2.23). */
static void test_sample(void)
{
	const uint8_t b[] = { 0x34, 0x12, 0xFF, 0xFF, 0x00, 0x80 };

	assert(bmi270_sample(&b[0]) == 0x1234);
	assert(bmi270_sample(&b[2]) == -1);
	assert(bmi270_sample(&b[4]) == INT16_MIN);
}

int main(void)
{
	test_accel_scale();
	test_gyro_scale();
	test_odr();
	test_temperature();
	test_cas();
	test_sample();
	printf("test_bmi270_math: all passed\n");
	return 0;
}
