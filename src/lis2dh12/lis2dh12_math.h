/*
 * Pure LIS2DH12 conversions, split out so they compile and are asserted on
 * the host (test/test_lis2dh12_math.c) with no bus or platform. They stay
 * in g and mg, the datasheet's units; the read op converts to m/s^2 last.
 *
 * Every figure is on the UNVERIFIED list of docs/chips.md "LIS2DH12".
 *
 * Two things to know about this part:
 *
 *   - the samples are left-justified 16-bit words whatever the mode, and
 *     the mode decides how many of the top bits are real: 8 in low power,
 *     10 in normal, 12 in high resolution. The mg per count then depends on
 *     the mode as well as the full scale, and at 16 g the steps are 12, 48
 *     and 192 mg, not the doubling the other ranges suggest.
 *   - the motion threshold has its own step per full scale, 16, 32, 62 and
 *     186 mg, unrelated to the sample resolution.
 */
#ifndef LIS2DH12_MATH_H
#define LIS2DH12_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "lis2dh12_regs.h"

/* Bits of the sample word that are real: 8, 10, 12; 0 for a bad mode. */
static inline int lis2dh12_resolution_bits(int mode)
{
	static const int bits[3] = { 8, 10, 12 };

	return (mode >= 0 && mode <= 2) ? bits[mode] : 0;
}

/*
 * mg per count, [mode][full scale] (table "Mechanical characteristics",
 * So): high resolution 1/2/4/12, normal 4/8/16/48, low power 16/32/64/192.
 */
static inline float lis2dh12_mg_per_count(int mode, int full_scale)
{
	static const float mg[3][4] = {
		{ 16.0f, 32.0f, 64.0f, 192.0f },	/* low power */
		{ 4.0f, 8.0f, 16.0f, 48.0f },		/* normal */
		{ 1.0f, 2.0f, 4.0f, 12.0f },		/* high resolution */
	};

	if (mode < 0 || mode > 2 || full_scale < 0 || full_scale > 3)
		return 0.0f;
	return mg[mode][full_scale];
}

static inline float lis2dh12_full_scale_g(int full_scale)
{
	static const float g[4] = { 2.0f, 4.0f, 8.0f, 16.0f };

	return (full_scale >= 0 && full_scale <= 3) ? g[full_scale] : 0.0f;
}

/*
 * One axis from its two bytes, low first, as counts at the mode's
 * resolution. The right shift of a negative value is arithmetic on every
 * compiler this library builds with (GCC, Clang, for Xtensa, RISC-V, ARM
 * and the host).
 */
static inline int16_t lis2dh12_counts(const uint8_t *p, int mode)
{
	int16_t word = (int16_t)(uint16_t)((uint16_t)p[1] << 8 | p[0]);
	int bits = lis2dh12_resolution_bits(mode);

	return bits ? (int16_t)(word >> (16 - bits)) : 0;
}

static inline float lis2dh12_counts_to_g(int16_t counts, int mode,
					 int full_scale)
{
	return (float)counts * lis2dh12_mg_per_count(mode, full_scale) /
	       1000.0f;
}

/*
 * Is this ODR code one the mode can run? 0x8 (1620 Hz) is low-power only;
 * 0x9 runs in every mode, at a rate that depends on it; 0xA..0xF do not
 * exist (9.7).
 */
static inline bool lis2dh12_odr_valid(int odr, int mode)
{
	if (lis2dh12_resolution_bits(mode) == 0 || odr < 0 || odr > 0x9)
		return false;
	return odr != 0x8 || mode == 0;
}

/* The rate in Hz; 0 for power-down or a code the mode cannot run. */
static inline float lis2dh12_odr_hz(int odr, int mode)
{
	static const float hz[10] = {
		0.0f, 1.0f, 10.0f, 25.0f, 50.0f, 100.0f, 200.0f, 400.0f,
		1620.0f, 1344.0f,
	};

	if (!lis2dh12_odr_valid(odr, mode))
		return 0.0f;
	if (odr == 0x9 && mode == 0)
		return 5376.0f;
	return hz[odr];
}

/* The motion threshold's step in mg per full scale (9.25). */
static inline float lis2dh12_threshold_step_mg(int full_scale)
{
	static const float mg[4] = { 16.0f, 32.0f, 62.0f, 186.0f };

	return (full_scale >= 0 && full_scale <= 3) ? mg[full_scale] : 0.0f;
}

/*
 * INT1_THS for threshold_g, rounded to the nearest step. False when that
 * is below one step or above 127, the 7 bits the register has (9.25).
 */
static inline bool lis2dh12_threshold_code(float threshold_g, int full_scale,
					   uint8_t *code)
{
	float step = lis2dh12_threshold_step_mg(full_scale);
	float steps;

	if (step <= 0.0f || !(threshold_g > 0.0f))
		return false;
	steps = roundf(threshold_g * 1000.0f / step);
	if (steps < 1.0f || steps > (float)LIS2DH12_7BIT_MAX)
		return false;
	*code = (uint8_t)steps;
	return true;
}

/* INT1_SRC's high-event bits as LIS2DH12_AXIS_* (9.24). */
static inline uint8_t lis2dh12_motion_axes(uint8_t source)
{
	return (uint8_t)(((source & LIS2DH12_XH) ? 1u << 0 : 0) |
			 ((source & LIS2DH12_YH) ? 1u << 1 : 0) |
			 ((source & LIS2DH12_ZH) ? 1u << 2 : 0));
}

/* The axes mask as INT1_CFG's high-event enables (9.23). */
static inline uint8_t lis2dh12_motion_cfg(uint8_t axes)
{
	return (uint8_t)(((axes & 1u) ? LIS2DH12_XHIE : 0) |
			 ((axes & 2u) ? LIS2DH12_YHIE : 0) |
			 ((axes & 4u) ? LIS2DH12_ZHIE : 0));
}

#endif /* LIS2DH12_MATH_H */
