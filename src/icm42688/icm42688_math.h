/*
 * ICM-42688-P conversions and tables, pure, so the host tests can assert
 * them without a bus. Every number is cited to the datasheet (DS-000347
 * rev. 1.6).
 */
#ifndef ICM42688_MATH_H
#define ICM42688_MATH_H

#include <stdbool.h>
#include <stdint.h>

/* Big endian, high byte first, as INTF_CONFIG0 resets it (14.34). */
static inline int16_t icm42688_sample(const uint8_t *b)
{
	return (int16_t)(uint16_t)((b[0] << 8) | b[1]);
}

/*
 * LSB/g for ACCEL_FS_SEL 0..3: 16, 8, 4, 2 g (Table 2, 14.38). 0 for a
 * reserved code.
 */
static inline float icm42688_accel_lsb_per_g(int fs)
{
	return fs >= 0 && fs <= 3 ? (float)(2048 << fs) : 0.0f;
}

static inline float icm42688_accel_fs_g(int fs)
{
	return fs >= 0 && fs <= 3 ? (float)(16 >> fs) : 0.0f;
}

/*
 * LSB/dps for GYRO_FS_SEL 0..7: 2000 down to 15.625 dps, as Table 1 prints
 * them -- rounded, so not exact powers of two of each other.
 */
static inline float icm42688_gyro_lsb_per_dps(int fs)
{
	static const float lsb[8] = {
		16.4f, 32.8f, 65.5f, 131.0f, 262.0f, 524.3f, 1048.6f, 2097.2f,
	};

	return fs >= 0 && fs <= 7 ? lsb[fs] : 0.0f;
}

static inline float icm42688_gyro_fs_dps(int fs)
{
	return fs >= 0 && fs <= 7 ? 2000.0f / (float)(1 << fs) : 0.0f;
}

/*
 * GYRO_ODR / ACCEL_ODR codes in Hz, in low-noise mode (14.37-14.38, 5.6):
 * 1..0xB and 0xF. 0x0 and 0xC..0xE are reserved for the gyro and low-power
 * only for the accelerometer: 0 here.
 */
static inline float icm42688_odr_hz(int code)
{
	static const float hz[16] = {
		[0x1] = 32000.0f, [0x2] = 16000.0f, [0x3] = 8000.0f,
		[0x4] = 4000.0f, [0x5] = 2000.0f, [0x6] = 1000.0f,
		[0x7] = 200.0f, [0x8] = 100.0f, [0x9] = 50.0f,
		[0xA] = 25.0f, [0xB] = 12.5f, [0xF] = 500.0f,
	};

	return code >= 0 && code <= 15 ? hz[code] : 0.0f;
}

/* (TEMP_DATA / 132.48) + 25 degC (14.6, 4.13). */
static inline float icm42688_temp_c(int16_t raw)
{
	return (float)raw / 132.48f + 25.0f;
}

/* *_UI_FILT_BW codes in low-noise mode: 0..7, 14, 15 (14.40). */
static inline bool icm42688_ui_bw_valid(int code)
{
	return (code >= 0 && code <= 7) || code == 14 || code == 15;
}

/* *_UI_FILT_ORD: 1st, 2nd or 3rd order, 3 reserved (14.39, 14.41, 5.5). */
static inline bool icm42688_ui_order_valid(int code)
{
	return code >= 0 && code <= 2;
}

/*
 * The anti-alias filter, one row per DELT code 1..63: its 3 dB bandwidth,
 * DELTSQR and BITSHIFT, transcribed from the table of section 5.3. DELTSQR
 * is not DELT squared in most rows; the table is what the chip wants. The
 * reset values are rows 13 (gyro, 15.3-15.5) and 24 (accel, 16.1-16.3).
 */
struct icm42688_aaf_row {
	uint16_t bandwidth_hz;
	uint16_t deltsqr;
	uint8_t bitshift;
};

static inline const struct icm42688_aaf_row *icm42688_aaf_row(int delt)
{
	static const struct icm42688_aaf_row rows[64] = {
		[1] = { 42, 1, 15 },
		[2] = { 84, 4, 13 },
		[3] = { 126, 9, 12 },
		[4] = { 170, 16, 11 },
		[5] = { 213, 25, 10 },
		[6] = { 258, 36, 10 },
		[7] = { 303, 49, 9 },
		[8] = { 348, 64, 9 },
		[9] = { 394, 81, 9 },
		[10] = { 441, 100, 8 },
		[11] = { 488, 122, 8 },
		[12] = { 536, 144, 8 },
		[13] = { 585, 170, 8 },
		[14] = { 634, 196, 7 },
		[15] = { 684, 224, 7 },
		[16] = { 734, 256, 7 },
		[17] = { 785, 288, 7 },
		[18] = { 837, 324, 7 },
		[19] = { 890, 360, 6 },
		[20] = { 943, 400, 6 },
		[21] = { 997, 440, 6 },
		[22] = { 1051, 488, 6 },
		[23] = { 1107, 528, 6 },
		[24] = { 1163, 576, 6 },
		[25] = { 1220, 624, 6 },
		[26] = { 1277, 680, 6 },
		[27] = { 1336, 736, 5 },
		[28] = { 1395, 784, 5 },
		[29] = { 1454, 848, 5 },
		[30] = { 1515, 896, 5 },
		[31] = { 1577, 960, 5 },
		[32] = { 1639, 1024, 5 },
		[33] = { 1702, 1088, 5 },
		[34] = { 1766, 1152, 5 },
		[35] = { 1830, 1232, 5 },
		[36] = { 1896, 1296, 5 },
		[37] = { 1962, 1376, 4 },
		[38] = { 2029, 1440, 4 },
		[39] = { 2097, 1536, 4 },
		[40] = { 2166, 1600, 4 },
		[41] = { 2235, 1696, 4 },
		[42] = { 2306, 1760, 4 },
		[43] = { 2377, 1856, 4 },
		[44] = { 2449, 1952, 4 },
		[45] = { 2522, 2016, 4 },
		[46] = { 2596, 2112, 4 },
		[47] = { 2671, 2208, 4 },
		[48] = { 2746, 2304, 4 },
		[49] = { 2823, 2400, 4 },
		[50] = { 2900, 2496, 4 },
		[51] = { 2978, 2592, 4 },
		[52] = { 3057, 2720, 4 },
		[53] = { 3137, 2816, 3 },
		[54] = { 3217, 2944, 3 },
		[55] = { 3299, 3008, 3 },
		[56] = { 3381, 3136, 3 },
		[57] = { 3464, 3264, 3 },
		[58] = { 3548, 3392, 3 },
		[59] = { 3633, 3456, 3 },
		[60] = { 3718, 3584, 3 },
		[61] = { 3805, 3712, 3 },
		[62] = { 3892, 3840, 3 },
		[63] = { 3979, 3968, 3 },
	};

	return delt >= 1 && delt <= 63 ? &rows[delt] : (void *)0;
}

#endif /* ICM42688_MATH_H */
