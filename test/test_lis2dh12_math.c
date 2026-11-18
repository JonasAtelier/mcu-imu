/*
 * src/lis2dh12/lis2dh12_math.h against the LIS2DH12 register map the
 * driver was written from (on the UNVERIFIED list of docs/chips.md), the
 * values typed in by hand. No platform; runs once.
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "lis2dh12_math.h"

static int near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* Table "Mechanical characteristics": mg/digit per mode and full scale. */
static void test_sensitivity(void)
{
	assert(lis2dh12_mg_per_count(2, 0) == 1.0f);	/* HR, 2 g */
	assert(lis2dh12_mg_per_count(2, 1) == 2.0f);
	assert(lis2dh12_mg_per_count(2, 2) == 4.0f);
	assert(lis2dh12_mg_per_count(2, 3) == 12.0f);	/* not 8 */
	assert(lis2dh12_mg_per_count(1, 0) == 4.0f);	/* normal */
	assert(lis2dh12_mg_per_count(1, 3) == 48.0f);
	assert(lis2dh12_mg_per_count(0, 0) == 16.0f);	/* low power */
	assert(lis2dh12_mg_per_count(0, 2) == 64.0f);
	assert(lis2dh12_mg_per_count(0, 3) == 192.0f);
	assert(lis2dh12_mg_per_count(3, 0) == 0.0f);
	assert(lis2dh12_mg_per_count(0, 4) == 0.0f);
	assert(lis2dh12_mg_per_count(-1, 0) == 0.0f);
	assert(lis2dh12_resolution_bits(0) == 8);
	assert(lis2dh12_resolution_bits(1) == 10);
	assert(lis2dh12_resolution_bits(2) == 12);
	assert(lis2dh12_resolution_bits(3) == 0);
	assert(lis2dh12_full_scale_g(0) == 2.0f);
	assert(lis2dh12_full_scale_g(3) == 16.0f);
	assert(lis2dh12_full_scale_g(4) == 0.0f);
}

/* Left-justified little-endian words, cut to the mode's resolution. */
static void test_counts(void)
{
	const uint8_t one_g_hr[] = { 0x80, 0x3E };	/* 16000 = 1000 << 4 */
	const uint8_t minus_one[] = { 0xF0, 0xFF };	/* -16 */
	const uint8_t lp[] = { 0x00, 0xFB };		/* -5 << 8 */
	const uint8_t lowest[] = { 0x00, 0x80 };
	const uint8_t highest[] = { 0xFF, 0x7F };

	assert(lis2dh12_counts(one_g_hr, 2) == 1000);
	assert(near(lis2dh12_counts_to_g(1000, 2, 0), 1.0f, 1e-6f));
	/* the same word is 250 counts in normal mode: 2 g at 4 g */
	assert(lis2dh12_counts(one_g_hr, 1) == 250);
	assert(near(lis2dh12_counts_to_g(250, 1, 1), 2.0f, 1e-6f));
	assert(lis2dh12_counts(one_g_hr, 0) == 62);
	assert(lis2dh12_counts(minus_one, 2) == -1);
	assert(lis2dh12_counts(minus_one, 1) == -1);	/* sign kept */
	assert(lis2dh12_counts(lp, 0) == -5);
	assert(near(lis2dh12_counts_to_g(-5, 0, 3), -0.96f, 1e-6f));
	assert(lis2dh12_counts(lowest, 2) == -2048);
	assert(lis2dh12_counts(lowest, 1) == -512);
	assert(lis2dh12_counts(lowest, 0) == -128);
	assert(lis2dh12_counts(highest, 2) == 2047);
	assert(lis2dh12_counts(highest, 1) == 511);
	assert(lis2dh12_counts(highest, 0) == 127);
	assert(lis2dh12_counts(highest, 3) == 0);
}

/* Table "Data rate configuration": 0x8 low power only, 0x9 per mode. */
static void test_odr(void)
{
	assert(lis2dh12_odr_valid(0x0, 1));		/* power-down */
	assert(lis2dh12_odr_hz(0x0, 1) == 0.0f);
	assert(lis2dh12_odr_hz(0x1, 2) == 1.0f);
	assert(lis2dh12_odr_hz(0x4, 0) == 50.0f);
	assert(lis2dh12_odr_hz(0x7, 1) == 400.0f);
	assert(lis2dh12_odr_valid(0x8, 0) && lis2dh12_odr_hz(0x8, 0) == 1620.0f);
	assert(!lis2dh12_odr_valid(0x8, 1) && !lis2dh12_odr_valid(0x8, 2));
	assert(lis2dh12_odr_hz(0x9, 0) == 5376.0f);
	assert(lis2dh12_odr_hz(0x9, 1) == 1344.0f);
	assert(lis2dh12_odr_hz(0x9, 2) == 1344.0f);
	assert(!lis2dh12_odr_valid(0xA, 0) && !lis2dh12_odr_valid(-1, 0));
	assert(!lis2dh12_odr_valid(0x4, 3));
}

/* INT1_THS: 16, 32, 62, 186 mg a step, 1..127 steps, rounded. */
static void test_threshold(void)
{
	uint8_t code = 0;

	assert(lis2dh12_threshold_code(1.5f, 0, &code) && code == 94);
	assert(lis2dh12_threshold_code(1.5f, 1, &code) && code == 47);
	assert(lis2dh12_threshold_code(1.5f, 2, &code) && code == 24);
	assert(lis2dh12_threshold_code(1.5f, 3, &code) && code == 8);
	assert(lis2dh12_threshold_code(0.016f, 0, &code) && code == 1);
	assert(lis2dh12_threshold_code(2.032f, 0, &code) && code == 127);
	code = 77;
	assert(!lis2dh12_threshold_code(2.1f, 0, &code) && code == 77);
	assert(!lis2dh12_threshold_code(0.005f, 2, &code));	/* 0 steps */
	assert(!lis2dh12_threshold_code(0.0f, 1, &code));
	assert(!lis2dh12_threshold_code(-1.0f, 1, &code));
	assert(!lis2dh12_threshold_code(NAN, 1, &code));
	assert(!lis2dh12_threshold_code(1.0f, 4, &code));
	assert(lis2dh12_threshold_step_mg(2) == 62.0f);
}

/* INT1_SRC and INT1_CFG bit by bit. */
static void test_motion_bits(void)
{
	assert(lis2dh12_motion_axes(0x00) == 0);
	assert(lis2dh12_motion_axes(0x40) == 0);		/* IA alone */
	assert(lis2dh12_motion_axes(0x62) == 0x5);		/* IA, ZH, XH */
	assert(lis2dh12_motion_axes(0x08) == 0x2);		/* YH */
	assert(lis2dh12_motion_axes(0x15) == 0);		/* the L bits */
	assert(lis2dh12_motion_axes(0x2A) == 0x7);
	assert(lis2dh12_motion_cfg(0x1) == 0x02);
	assert(lis2dh12_motion_cfg(0x2) == 0x08);
	assert(lis2dh12_motion_cfg(0x4) == 0x20);
	assert(lis2dh12_motion_cfg(0x7) == 0x2A);
}

int main(void)
{
	test_sensitivity();
	test_counts();
	test_odr();
	test_threshold();
	test_motion_bits();
	printf("test_lis2dh12_math: all passed\n");
	return 0;
}
