/*
 * Pure raw-register <-> physical-unit conversions, split out so they can be
 * compiled and asserted on the host (see test/test_icm45686_math.c) without
 * an ARM toolchain. Every constant here is quoted from TDK's reference driver
 * or Zephyr's, named at the point of use. These stay in g and dps; the
 * driver's read op converts to SI at the very end.
 *
 * The full-scale codes are the trap: they run *backwards*. Code 0 is the
 * widest range on both sensors and the largest code is the narrowest, which
 * is the opposite of almost every other IMU. Pick the scale factor off the
 * wrong end and every reading is wrong by a power of two with nothing
 * reporting an error.
 */
#ifndef ICM45686_MATH_H
#define ICM45686_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Sensor data is 16-bit signed, full scale at the ends of the range. */
#define ICM_FS_COUNTS		32768.0f

/*
 * Temperature, from Zephyr's icm45686_temp_c(): 132.48 LSB per degree,
 * zero at 25 C.
 */
#define ICM_TEMP_SENS_LSB_C	132.48f
#define ICM_TEMP_OFFSET_C	25.0f

/* accel_ui_fs_sel, ACCEL_CONFIG0 bits 6:4. Index is the register code. */
static inline float icm45686_accel_fs_g(int code)
{
	static const float g[8] = { 32.0f, 16.0f, 8.0f, 4.0f, 2.0f, 0, 0, 0 };

	return (code >= 0 && code <= 4) ? g[code] : 0.0f;
}

/* gyro_ui_fs_sel, GYRO_CONFIG0 bits 7:4. Index is the register code. */
static inline float icm45686_gyro_fs_dps(int code)
{
	static const float d[16] = { 4000.0f, 2000.0f, 1000.0f, 500.0f,
				     250.0f, 125.0f, 62.5f, 31.25f, 15.625f };

	return (code >= 0 && code <= 8) ? d[code] : 0.0f;
}

/* accel_odr / gyro_odr, both configs bits 3:0. Codes 0..2 are reserved. */
static inline float icm45686_odr_hz(int code)
{
	static const float hz[16] = {
		0, 0, 0, 6400.0f, 3200.0f, 1600.0f, 800.0f, 400.0f,
		200.0f, 100.0f, 50.0f, 25.0f, 12.5f, 6.25f, 3.125f, 1.5625f
	};

	return (code >= 3 && code <= 15) ? hz[code] : 0.0f;
}

/*
 * Counts per g / per dps for a full-scale code. 0 means the code is invalid,
 * which the caller must treat as a configuration error rather than divide by.
 */
static inline float icm45686_accel_sensitivity_code(int code)
{
	float fs = icm45686_accel_fs_g(code);

	return fs > 0.0f ? ICM_FS_COUNTS / fs : 0.0f;
}

static inline float icm45686_gyro_sensitivity_code(int code)
{
	float fs = icm45686_gyro_fs_dps(code);

	return fs > 0.0f ? ICM_FS_COUNTS / fs : 0.0f;
}

static inline float icm45686_accel_g(int16_t raw, int code)
{
	float s = icm45686_accel_sensitivity_code(code);

	return s > 0.0f ? (float)raw / s : 0.0f;
}

static inline float icm45686_gyro_dps(int16_t raw, int code)
{
	float s = icm45686_gyro_sensitivity_code(code);

	return s > 0.0f ? (float)raw / s : 0.0f;
}

static inline float icm45686_temp_c(int16_t raw)
{
	return (float)raw / ICM_TEMP_SENS_LSB_C + ICM_TEMP_OFFSET_C;
}

/*
 * Assemble one signed 16-bit sample from two bytes.
 *
 * Which byte comes first is not fixed in silicon: SREG_CTRL bit 1 selects it,
 * and the register names (X1 then X0) describe the big-endian arrangement.
 * The driver pins this at probe rather than trusting a reset default, so this
 * takes the choice as an argument instead of guessing.
 */
static inline int16_t icm45686_sample(const uint8_t *p, bool big_endian)
{
	uint16_t v = big_endian ? ((uint16_t)p[0] << 8) | p[1] :
				  ((uint16_t)p[1] << 8) | p[0];

	return (int16_t)v;
}

#endif /* ICM45686_MATH_H */
