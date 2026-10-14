/*
 * BMI270 conversions, pure, so the host tests can assert them without a
 * bus. Every number is cited to the datasheet (BST-BMI270-DS000-08 rev. 1.6).
 */
#ifndef BMI270_MATH_H
#define BMI270_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Little endian, LSB first: every data register pair (5.2.12-5.2.35). */
static inline int16_t bmi270_sample(const uint8_t *b)
{
	return (int16_t)(uint16_t)(b[0] | (b[1] << 8));
}

/* LSB/g: 16384 at 2 g, halving per range step (Table 2). 0 if unknown. */
static inline float bmi270_accel_lsb_per_g(int range)
{
	return range >= 0 && range <= 3 ? (float)(16384 >> range) : 0.0f;
}

/* LSB/dps: 16.384 at 2000 dps, doubling per range step (Table 3). */
static inline float bmi270_gyro_lsb_per_dps(int range)
{
	return range >= 0 && range <= 4 ? 16.384f * (float)(1 << range) : 0.0f;
}

/* Full scale of a range code, in g and dps (Table 2, Table 3). */
static inline float bmi270_accel_range_g(int range)
{
	return range >= 0 && range <= 3 ? (float)(2 << range) : 0.0f;
}

static inline float bmi270_gyro_range_dps(int range)
{
	return range >= 0 && range <= 4 ? 2000.0f / (float)(1 << range) : 0.0f;
}

/* The rate codes, in Hz: 25/32 * 2^(code - 1), so 0x08 is 100 Hz (5.2.41). */
static inline float bmi270_odr_hz(int code)
{
	return code >= 1 && code <= 0x0F ?
	       25.0f / 32.0f * (float)(1 << (code - 1)) : 0.0f;
}

/* acc_odr codes with acc_filter_perf = 1: 12.5 Hz to 1.6 kHz (4.6.4). */
static inline bool bmi270_accel_odr_valid(int code)
{
	return code >= 0x05 && code <= 0x0C;
}

/* gyr_odr codes: 25 Hz to 3.2 kHz (5.2.43, 4.6.11). */
static inline bool bmi270_gyro_odr_valid(int code)
{
	return code >= 0x06 && code <= 0x0D;
}

/*
 * 0x0000 is 23 degC and one LSB is 1/512 K (Table 4, 4.6.14); 0x8000 marks
 * an invalid reading (5.2.34), NaN here.
 */
static inline float bmi270_temp_c(int16_t raw)
{
	if (raw == INT16_MIN)
		return NAN;
	return 23.0f + (float)raw / 512.0f;
}

/* GYR_CAS.factor_zx: 7 bits, two's complement (5.2.40, note in 4.6.10). */
static inline int8_t bmi270_cas_factor(uint8_t gyr_cas)
{
	gyr_cas &= 0x7F;
	return (int8_t)(gyr_cas & 0x40 ? (int)gyr_cas - 128 : (int)gyr_cas);
}

/*
 * Gyro X corrected for cross-axis sensitivity:
 * x - factor_zx * z / 2^9 (4.6.10). Integer division as in Bosch's
 * comp_gyro_cross_axis_sensitivity() (bmi2.c), then held to 16 bits.
 */
static inline int16_t bmi270_cas_correct(int16_t x, int16_t z, int8_t factor)
{
	int32_t v = (int32_t)x - ((int32_t)factor * (int32_t)z) / 512;

	if (v > INT16_MAX)
		return INT16_MAX;
	if (v < INT16_MIN)
		return INT16_MIN;
	return (int16_t)v;
}

#endif /* BMI270_MATH_H */
