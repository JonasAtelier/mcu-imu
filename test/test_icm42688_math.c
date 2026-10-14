/*
 * src/icm42688/icm42688_math.h against the ICM-42688-P datasheet, DS-000347
 * rev. 1.6, the values typed in by hand from it. No platform; runs once.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "icm42688_math.h"

static int near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* Table 2 and 14.38: 2048 .. 16384 LSB/g for 16 .. 2 g. */
static void test_accel(void)
{
	assert(icm42688_accel_lsb_per_g(0) == 2048.0f);
	assert(icm42688_accel_lsb_per_g(1) == 4096.0f);
	assert(icm42688_accel_lsb_per_g(2) == 8192.0f);
	assert(icm42688_accel_lsb_per_g(3) == 16384.0f);
	assert(icm42688_accel_lsb_per_g(4) == 0.0f);	/* reserved */
	assert(icm42688_accel_fs_g(0) == 16.0f);
	assert(icm42688_accel_fs_g(3) == 2.0f);
	assert(icm42688_accel_fs_g(4) == 0.0f);
}

/* Table 1 and 14.37, as printed. */
static void test_gyro(void)
{
	static const float lsb[8] = {
		16.4f, 32.8f, 65.5f, 131.0f, 262.0f, 524.3f, 1048.6f, 2097.2f,
	};
	static const float dps[8] = {
		2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f, 62.5f, 31.25f,
		15.625f,
	};

	for (int fs = 0; fs < 8; fs++) {
		assert(icm42688_gyro_lsb_per_dps(fs) == lsb[fs]);
		assert(icm42688_gyro_fs_dps(fs) == dps[fs]);
		/* full scale times sensitivity is about 2^15 */
		assert(near(dps[fs] * lsb[fs], 32768.0f, 32768.0f * 0.002f));
	}
	assert(icm42688_gyro_lsb_per_dps(8) == 0.0f);
	assert(icm42688_gyro_fs_dps(-1) == 0.0f);
}

/* 14.37-14.38 and 5.6: the codes in low-noise mode. */
static void test_odr(void)
{
	assert(icm42688_odr_hz(0x0) == 0.0f);
	assert(icm42688_odr_hz(0x1) == 32000.0f);
	assert(icm42688_odr_hz(0x6) == 1000.0f);
	assert(icm42688_odr_hz(0x7) == 200.0f);
	assert(icm42688_odr_hz(0xB) == 12.5f);
	assert(icm42688_odr_hz(0xC) == 0.0f);		/* 6.25 Hz, LP only */
	assert(icm42688_odr_hz(0xE) == 0.0f);
	assert(icm42688_odr_hz(0xF) == 500.0f);
	assert(icm42688_odr_hz(16) == 0.0f);
}

/* 14.6: (TEMP_DATA / 132.48) + 25. */
static void test_temperature(void)
{
	assert(icm42688_temp_c(0) == 25.0f);
	assert(near(icm42688_temp_c(1325), 25.0f + 1325.0f / 132.48f, 1e-4f));
	assert(near(icm42688_temp_c(-3312), 0.0f, 0.01f));
}

/* 14.40, 14.39, 14.41. */
static void test_ui_filter(void)
{
	for (int c = 0; c <= 7; c++)
		assert(icm42688_ui_bw_valid(c));
	for (int c = 8; c <= 13; c++)
		assert(!icm42688_ui_bw_valid(c));
	assert(icm42688_ui_bw_valid(14) && icm42688_ui_bw_valid(15));
	assert(!icm42688_ui_bw_valid(16) && !icm42688_ui_bw_valid(-1));
	assert(icm42688_ui_order_valid(0) && icm42688_ui_order_valid(2));
	assert(!icm42688_ui_order_valid(3));
}

/* 5.3: rows typed in from the table, ends and odd ones. */
static void test_aaf(void)
{
	static const struct { int delt, bw, sq, shift; } rows[] = {
		{ 1, 42, 1, 15 }, { 2, 84, 4, 13 }, { 9, 394, 81, 9 },
		{ 11, 488, 122, 8 }, { 13, 585, 170, 8 }, { 22, 1051, 488, 6 },
		{ 24, 1163, 576, 6 }, { 27, 1336, 736, 5 },
		{ 37, 1962, 1376, 4 }, { 52, 3057, 2720, 4 },
		{ 53, 3137, 2816, 3 }, { 63, 3979, 3968, 3 },
	};

	for (size_t i = 0; i < sizeof(rows) / sizeof(rows[0]); i++) {
		const struct icm42688_aaf_row *r =
			icm42688_aaf_row(rows[i].delt);

		assert(r && r->bandwidth_hz == rows[i].bw);
		assert(r->deltsqr == rows[i].sq);
		assert(r->bitshift == rows[i].shift);
	}
	/* every row: bandwidth rising, DELTSQR fits 12 bits, BITSHIFT 4 */
	for (int d = 2; d <= 63; d++) {
		assert(icm42688_aaf_row(d)->bandwidth_hz >
		       icm42688_aaf_row(d - 1)->bandwidth_hz);
		assert(icm42688_aaf_row(d)->deltsqr < 4096);
		assert(icm42688_aaf_row(d)->bitshift < 16);
	}
	assert(icm42688_aaf_row(0) == NULL && icm42688_aaf_row(64) == NULL);
}

/* 14.34: big endian. */
static void test_sample(void)
{
	const uint8_t b[] = { 0x12, 0x34, 0xFF, 0xFF, 0x80, 0x00 };

	assert(icm42688_sample(&b[0]) == 0x1234);
	assert(icm42688_sample(&b[2]) == -1);
	assert(icm42688_sample(&b[4]) == INT16_MIN);
}

int main(void)
{
	test_accel();
	test_gyro();
	test_odr();
	test_temperature();
	test_ui_filter();
	test_aaf();
	test_sample();
	printf("test_icm42688_math: all passed\n");
	return 0;
}
