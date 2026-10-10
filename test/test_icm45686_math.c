/*
 * Host-side check of the register conversions:
 *
 *     make -C test test
 *
 * The full-scale tables are the thing worth guarding. Their codes run
 * backwards -- 0 is the widest range -- so a table written the intuitive way
 * round produces readings wrong by a power of two, and nothing anywhere
 * reports an error.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "icm45686_math.h"
#include "icm45686_regs.h"

#define NEAR(a, b) (fabs((double)(a) - (double)(b)) < 1e-4 * fabs(b) + 1e-9)

static void test_fullscale_tables(void)
{
	/* accel_ui_fs_sel, TDK's inv_imu_defs.h: 32 g is code 0, 2 g is 4. */
	assert(NEAR(icm45686_accel_fs_g(0), 32.0f));
	assert(NEAR(icm45686_accel_fs_g(1), 16.0f));
	assert(NEAR(icm45686_accel_fs_g(2), 8.0f));
	assert(NEAR(icm45686_accel_fs_g(3), 4.0f));
	assert(NEAR(icm45686_accel_fs_g(4), 2.0f));
	/* The codes run backwards -- guard the direction explicitly. */
	assert(icm45686_accel_fs_g(0) > icm45686_accel_fs_g(4));

	/* gyro_ui_fs_sel: 4000 dps is code 0, 15.625 dps is code 8. */
	assert(NEAR(icm45686_gyro_fs_dps(0), 4000.0f));
	assert(NEAR(icm45686_gyro_fs_dps(1), 2000.0f));
	assert(NEAR(icm45686_gyro_fs_dps(4), 250.0f));
	assert(NEAR(icm45686_gyro_fs_dps(6), 62.5f));
	assert(NEAR(icm45686_gyro_fs_dps(7), 31.25f));
	assert(NEAR(icm45686_gyro_fs_dps(8), 15.625f));
	assert(icm45686_gyro_fs_dps(0) > icm45686_gyro_fs_dps(8));

	/* Each step down the table halves the range. */
	for (int c = 0; c < 4; c++)
		assert(NEAR(icm45686_accel_fs_g(c),
			    icm45686_accel_fs_g(c + 1) * 2.0f));
	for (int c = 0; c < 8; c++)
		assert(NEAR(icm45686_gyro_fs_dps(c),
			    icm45686_gyro_fs_dps(c + 1) * 2.0f));

	/* Out of range codes must report 0, never a plausible number. */
	assert(icm45686_accel_fs_g(5) == 0.0f);
	assert(icm45686_accel_fs_g(-1) == 0.0f);
	assert(icm45686_accel_fs_g(7) == 0.0f);
	assert(icm45686_gyro_fs_dps(9) == 0.0f);
	assert(icm45686_gyro_fs_dps(-1) == 0.0f);
	assert(icm45686_gyro_fs_dps(15) == 0.0f);
}

static void test_odr_table(void)
{
	assert(NEAR(icm45686_odr_hz(0x3), 6400.0f));
	assert(NEAR(icm45686_odr_hz(0x6), 800.0f));
	assert(NEAR(icm45686_odr_hz(0x9), 100.0f));
	assert(NEAR(icm45686_odr_hz(0xF), 1.5625f));
	/* Codes 0..2 are reserved, not "very fast". */
	assert(icm45686_odr_hz(0) == 0.0f);
	assert(icm45686_odr_hz(1) == 0.0f);
	assert(icm45686_odr_hz(2) == 0.0f);
	assert(icm45686_odr_hz(16) == 0.0f);
	/* Each step down halves the rate. */
	for (int c = 3; c < 15; c++)
		assert(NEAR(icm45686_odr_hz(c), icm45686_odr_hz(c + 1) * 2.0f));
}

static void test_conversions(void)
{
	/* Full scale sits at the ends of a signed 16-bit range. */
	assert(NEAR(icm45686_accel_sensitivity_code(4), 16384.0f));
	assert(NEAR(icm45686_accel_g(16384, 4), 1.0f));
	assert(NEAR(icm45686_accel_g(-16384, 4), -1.0f));
	assert(NEAR(icm45686_accel_g(32767, 0), 32.0f * 32767.0f / 32768.0f));

	/* 250 dps is code 4: 32768/250 = 131.072 counts per dps. */
	assert(NEAR(icm45686_gyro_sensitivity_code(4), 131.072f));
	assert(NEAR(icm45686_gyro_dps(131, 4), 131.0f / 131.072f));
	assert(NEAR(icm45686_gyro_dps(0, 4), 0.0f));

	/* An invalid code must not divide by zero into a reading. */
	assert(icm45686_accel_g(1000, 9) == 0.0f);
	assert(icm45686_gyro_dps(1000, 9) == 0.0f);
	assert(icm45686_accel_sensitivity_code(9) == 0.0f);

	/* Temperature: 132.48 LSB per degree, zero at 25 C. */
	assert(NEAR(icm45686_temp_c(0), 25.0f));
	assert(NEAR(icm45686_temp_c(13248), 125.0f));
	assert(NEAR(icm45686_temp_c(-13248), -75.0f));
}

static void test_sample_assembly(void)
{
	const uint8_t p[2] = { 0x12, 0x34 };

	assert(icm45686_sample(p, true) == (int16_t)0x1234);
	assert(icm45686_sample(p, false) == (int16_t)0x3412);

	/* Sign has to survive both orders. */
	const uint8_t neg_be[2] = { 0xFF, 0xFF };
	const uint8_t neg_le[2] = { 0x00, 0x80 };
	assert(icm45686_sample(neg_be, true) == -1);
	assert(icm45686_sample(neg_le, false) == -32768);

	const uint8_t max_be[2] = { 0x7F, 0xFF };
	assert(icm45686_sample(max_be, true) == 32767);
	assert(icm45686_sample(max_be, false) == (int16_t)0xFF7F);
}

static void test_register_constants(void)
{
	/* Guard the constants the driver depends on against a careless edit. */
	assert(ICM_WHO_AM_I == 0x72);
	assert(ICM_WHO_AM_I_VALUE == 0xE9);
	assert(ICM_ACCEL_DATA_X1_UI == 0x00);
	assert(ICM_GYRO_DATA_X1_UI == 0x06);
	assert(ICM_TEMP_DATA1_UI == 0x0C);
	/* The burst has to span exactly accel, gyro and temperature. */
	assert(ICM_DATA_BURST_LEN == ICM_TEMP_DATA1_UI + 2);
	assert(ICM_GYRO_DATA_X1_UI == ICM_ACCEL_DATA_X1_UI + 6);
	/* Field shifts, cross-checked against Zephyr's macros. */
	assert(ICM_ACCEL_FS_SHIFT == 4 && ICM_ACCEL_ODR_SHIFT == 0);
	assert(ICM_GYRO_FS_SHIFT == 4 && ICM_GYRO_ODR_SHIFT == 0);
	assert(ICM_GYRO_MODE_SHIFT == 2 && ICM_ACCEL_MODE_SHIFT == 0);
	assert(ICM_SOFT_RST == 0x02 && ICM_IREG_DONE == 0x01);
	assert(ICM_INT_DRDY == 0x04);
}

int main(void)
{
	test_fullscale_tables();
	test_odr_table();
	test_conversions();
	test_sample_assembly();
	test_register_constants();
	printf("all icm45686 math checks passed\n");
	return 0;
}
