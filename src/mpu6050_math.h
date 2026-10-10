/*
 * Pure raw-register <-> physical-unit conversions, split out so they can be
 * compiled and asserted on the host (see test/test_mpu6050_math.c) without an
 * ARM toolchain. These stay in g and dps, the units the datasheet tables are
 * written in; the driver's read op converts to SI at the very end.
 *
 * Two things here are worth more attention than they look:
 *
 *   - the sample rate depends on a register this driver also sets for a
 *     different reason. DLPF_CFG picks the filter *and* the gyro output rate,
 *     8 kHz at settings 0 and 7 and 1 kHz everywhere else, and SMPLRT_DIV
 *     divides whichever one is in force. Change the filter and the rate moves
 *     under you.
 *
 *   - the temperature offset is 36.53, not 36. Zephyr rounds it; Linux carries
 *     it exactly as 12420/340 and so does the datasheet.
 */
#ifndef MPU6050_MATH_H
#define MPU6050_MATH_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Temperature: Linux stores offset 12420 and scale 1/340, i.e. 36.53 C. */
#define MPU6050_TEMP_LSB_C	340.0f
#define MPU6050_TEMP_OFFSET_C	36.53f

/* Gyro output rate feeding SMPLRT_DIV, chosen by DLPF_CFG. */
#define MPU6050_GYRO_RATE_FAST	8000.0f		/* DLPF_CFG 0 or 7 */
#define MPU6050_GYRO_RATE_SLOW	1000.0f		/* DLPF_CFG 1..6 */

/* AFS_SEL -> counts per g. 2 g is 16384 and each step halves it. */
static inline float mpu6050_accel_sensitivity_of(int afs_sel)
{
	static const float s[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };

	return (afs_sel >= 0 && afs_sel <= 3) ? s[afs_sel] : 0.0f;
}

/*
 * FS_SEL -> counts per degree per second. Zephyr carries these x10 as
 * 1310, 655, 328, 164; Linux's nano-rad/s scales agree.
 */
static inline float mpu6050_gyro_sensitivity_of(int fs_sel)
{
	static const float s[4] = { 131.0f, 65.5f, 32.8f, 16.4f };

	return (fs_sel >= 0 && fs_sel <= 3) ? s[fs_sel] : 0.0f;
}

static inline float mpu6050_accel_fs_g(int afs_sel)
{
	static const float g[4] = { 2.0f, 4.0f, 8.0f, 16.0f };

	return (afs_sel >= 0 && afs_sel <= 3) ? g[afs_sel] : 0.0f;
}

static inline float mpu6050_gyro_fs_dps(int fs_sel)
{
	static const float d[4] = { 250.0f, 500.0f, 1000.0f, 2000.0f };

	return (fs_sel >= 0 && fs_sel <= 3) ? d[fs_sel] : 0.0f;
}

static inline float mpu6050_accel_g(int16_t raw, int afs_sel)
{
	float s = mpu6050_accel_sensitivity_of(afs_sel);

	return s > 0.0f ? (float)raw / s : 0.0f;
}

static inline float mpu6050_gyro_dps(int16_t raw, int fs_sel)
{
	float s = mpu6050_gyro_sensitivity_of(fs_sel);

	return s > 0.0f ? (float)raw / s : 0.0f;
}

static inline float mpu6050_temp_c(int16_t raw)
{
	return (float)raw / MPU6050_TEMP_LSB_C + MPU6050_TEMP_OFFSET_C;
}

/* The gyro output rate SMPLRT_DIV divides, for a given DLPF_CFG. */
static inline float mpu6050_gyro_output_rate(int dlpf_cfg)
{
	if (dlpf_cfg < 0 || dlpf_cfg > 7)
		return 0.0f;
	return (dlpf_cfg == 0 || dlpf_cfg == 7) ? MPU6050_GYRO_RATE_FAST :
						  MPU6050_GYRO_RATE_SLOW;
}

/* Actual sample rate: gyro_output_rate / (1 + SMPLRT_DIV). */
static inline float mpu6050_sample_rate_of(int dlpf_cfg, uint8_t smplrt_div)
{
	float base = mpu6050_gyro_output_rate(dlpf_cfg);

	return base > 0.0f ? base / (1.0f + (float)smplrt_div) : 0.0f;
}

/*
 * The divider that lands closest to a wanted rate, given the filter in force.
 * SMPLRT_DIV is 8 bits, so the reachable range is base/256 to base -- ask for
 * 10 Hz with the filter off and 8000/256 = 31.25 Hz is as slow as it goes,
 * and 2 kHz with any other filter is above its 1 kHz base. Both ends are
 * refused rather than clamped: returns false for an unusable request, and the
 * caller reports that rather than programming a rate nobody asked for.
 * Exactly the base rate is fine: base / base - 1 is exactly 0.
 */
static inline bool mpu6050_divider_for(float want_hz, int dlpf_cfg,
				       uint8_t *out)
{
	float base = mpu6050_gyro_output_rate(dlpf_cfg);
	float div;

	if (!(want_hz > 0.0f) || base <= 0.0f)
		return false;

	div = base / want_hz - 1.0f;
	if (div < 0.0f)
		return false;		/* faster than base: unreachable */
	if (div > 255.0f)
		return false;		/* slower than base/256: unreachable */

	*out = (uint8_t)(div + 0.5f);
	return true;
}

/* Big endian, high byte first, and not configurable on this part. */
static inline int16_t mpu6050_sample(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

#endif /* MPU6050_MATH_H */
