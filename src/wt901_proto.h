/*
 * WT901 serial protocol: frame bytes, scales and the checksum. Every value
 * cites WitMotion's "WT901 Attitude Angle Sensor SPECIFICATION" in
 * docs/WT901.pdf, by section and printed page. Pure, so it compiles and is
 * asserted on the host.
 *
 * The module streams 11-byte frames on its own (§6.1, p.30-35) at 10 Hz out
 * of the box (§6.2 reminder 1, p.35); commands go the other way as five
 * bytes, 0xFF 0xAA address dataL dataH (§6.2 reminder 2, p.35). There is no
 * serial command to read a register: chapter 6 has only writes.
 */
#ifndef WT901_PROTO_H
#define WT901_PROTO_H

#include <stdbool.h>
#include <stdint.h>

/* §6.1.1-6.1.11: 0x55, type, 8 data bytes, SUM. */
#define WT901_FRAME_HEADER	0x55
#define WT901_FRAME_LEN		11
#define WT901_FRAME_DATA	2	/* offset of the first data byte */

/* §6.1: frame types, 0x50 time to 0x5A satellite accuracy. */
#define WT901_TYPE_FIRST	0x50
#define WT901_TYPE_ACCEL	0x51	/* §6.1.2, p.31 */
#define WT901_TYPE_GYRO		0x52	/* §6.1.3, p.31 */
#define WT901_TYPE_ANGLE	0x53	/* §6.1.4, p.32; titled "Angular
					 * Velocity Output", but its bytes and
					 * formulas are the angles */
#define WT901_TYPE_LAST		0x5A	/* §6.1.11, p.35 */

/* §6.2 reminder 2, p.35: the first two bytes of every command. */
#define WT901_COMMAND_1		0xFF
#define WT901_COMMAND_2		0xAA

/*
 * Scales, as the formulas give them, for a signed 16-bit value:
 *   §6.1.2 p.31: a = raw / 32768 * 16 g
 *   §6.1.3 p.31: w = raw / 32768 * 2000 deg/s
 *   §6.1.4 p.32: angle = raw / 32768 * 180 deg
 *   §6.1.2-6.1.4: T = raw / 100 degC
 * They match §2 item 6's ranges, +/-16 g and +/-2000 deg/s (p.5).
 */
#define WT901_ACCEL_FULL_SCALE_G	16.0f
#define WT901_GYRO_FULL_SCALE_DPS	2000.0f
#define WT901_ANGLE_FULL_SCALE_DEG	180.0f
#define WT901_RAW_FULL_SCALE		32768.0f
#define WT901_TEMP_LSB_PER_C		100.0f

/*
 * §6.1.2 note 2, p.31: each value low byte first, combined as a signed
 * short.
 */
static inline int16_t wt901_value(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/*
 * The SUM byte. §6.1.2, 6.1.3, 6.1.4 and 6.1.10 all give it as 0x55 + the
 * frame's own type + its eight data bytes. Two readings of the document are
 * possible, and this follows the first:
 *
 *   - The rule is uniform, and the other sections' formulas are typos: they
 *     name the wrong type (0x51 for the 0x50 frame in §6.1.1, 0x53 for 0x54
 *     in §6.1.5, 0x54 for 0x56-0x58 in §6.1.7-6.1.9, 0x59 for 0x5A in
 *     §6.1.11) while listing that frame's own data bytes.
 *   - Taken literally, those frames would carry a sum over a different type
 *     byte than the one they send.
 *
 * The document never says the sum is cut to a byte; SUM is one byte, so
 * this keeps the low 8 bits.
 */
static inline uint8_t wt901_sum(const uint8_t *frame)
{
	uint8_t sum = 0;

	for (int i = 0; i < WT901_FRAME_LEN - 1; i++)
		sum = (uint8_t)(sum + frame[i]);
	return sum;
}

static inline bool wt901_known_type(uint8_t type)
{
	return type >= WT901_TYPE_FIRST && type <= WT901_TYPE_LAST;
}

/* A header, a known type and a matching SUM. */
static inline bool wt901_frame_valid(const uint8_t *frame)
{
	return frame[0] == WT901_FRAME_HEADER && wt901_known_type(frame[1]) &&
	       wt901_sum(frame) == frame[WT901_FRAME_LEN - 1];
}

static inline float wt901_scaled(int16_t raw, float full_scale)
{
	return (float)raw / WT901_RAW_FULL_SCALE * full_scale;
}

#endif /* WT901_PROTO_H */
