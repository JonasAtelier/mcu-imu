/*
 * JY901 register map and serial frames. The module has one address space,
 * read over I2C (chapter 7) and written over either bus; its UART also
 * streams frames on its own. Every value cites WitMotion's "WT901 Attitude
 * Angle Sensor SPECIFICATION" in docs/WT901.pdf, by section and printed
 * page: no JY901-specific manual is on disk, and that document itself names
 * the module JY-901 in §5.2 (p.26) and chapter 7 (p.46-49). Pure, so it
 * compiles and is asserted on the host.
 *
 * UART: 11-byte frames out (§6.1, p.30-35) at 10 Hz by default (§6.2
 * reminder 1, p.35); commands in as 0xFF 0xAA address dataL dataH (§6.2
 * reminder 2, p.35). I2C: every register is 16 bits, low byte first, and a
 * read or write runs on to the next register (§7.1 p.48, §7.2 p.49).
 */
#ifndef JY901_REGS_H
#define JY901_REGS_H

#include <stdbool.h>
#include <stdint.h>

/* ---------------------------------------------------------- registers --- */

/*
 * §6.2.1 p.35-36, the same table as chapter 7 p.46-48. The serial command
 * addresses are these registers.
 */
#define JY901_REG_SAVE		0x00
#define JY901_REG_RSW		0x02	/* return content, §6.2.8 p.38 */
#define JY901_REG_RATE		0x03	/* return rate, §6.2.9 p.39 */
#define JY901_REG_IICADDR	0x1a	/* §6.2.32 p.45 */
#define JY901_REG_AX		0x34	/* p.47: AX AY AZ */
#define JY901_REG_GX		0x37	/* p.47: GX GY GZ */
#define JY901_REG_ROLL		0x3d	/* p.47-48: Roll Pitch Yaw */
#define JY901_REG_TEMP		0x40	/* p.48 */

/* §7.2 p.49: two bytes per register, low byte first. */
#define JY901_REG_BYTES		2

/*
 * imu_read() on I2C: AX through TEMP in one read, so accelerometer,
 * gyroscope and temperature come from one moment. Offsets are in registers
 * from AX.
 */
#define JY901_BURST_FIRST	JY901_REG_AX
#define JY901_BURST_REGS	(JY901_REG_TEMP - JY901_REG_AX + 1)
#define JY901_BURST_LEN		(JY901_BURST_REGS * JY901_REG_BYTES)
#define JY901_BURST_ACCEL	(JY901_REG_AX - JY901_BURST_FIRST)
#define JY901_BURST_GYRO	(JY901_REG_GX - JY901_BURST_FIRST)
#define JY901_BURST_TEMP	(JY901_REG_TEMP - JY901_BURST_FIRST)

/* §6.2.32 p.45: 7-bit, default 0x50, at most 0x7f. */
#define JY901_I2C_ADDRESS	0x50
#define JY901_I2C_ADDRESS_MAX	0x7f

/* §1 p.4, §2 item 11 p.5, chapter 7 p.46: at most 400 kHz. */
#define JY901_I2C_MAX_HZ	400000

/* ------------------------------------------------------------- frames --- */

/* §6.1.1-6.1.11: 0x55, type, 8 data bytes, SUM. */
#define JY901_FRAME_HEADER	0x55
#define JY901_FRAME_LEN		11
#define JY901_FRAME_DATA	2	/* offset of the first data byte */

/* §6.1: frame types, 0x50 time to 0x5A satellite accuracy. */
#define JY901_TYPE_FIRST	0x50
#define JY901_TYPE_ACCEL	0x51	/* §6.1.2, p.31 */
#define JY901_TYPE_GYRO		0x52	/* §6.1.3, p.31 */
#define JY901_TYPE_ANGLE	0x53	/* §6.1.4, p.32; titled "Angular
					 * Velocity Output", but its bytes and
					 * formulas are the angles */
#define JY901_TYPE_LAST		0x5A	/* §6.1.11, p.35 */

/* §6.2 reminder 2, p.35: the first two bytes of every command. */
#define JY901_COMMAND_1		0xFF
#define JY901_COMMAND_2		0xAA

/*
 * Scales, as the frame formulas give them, for a signed 16-bit value:
 *   §6.1.2 p.31: a = raw / 32768 * 16 g
 *   §6.1.3 p.31: w = raw / 32768 * 2000 deg/s
 *   §6.1.4 p.32: angle = raw / 32768 * 180 deg
 *   §6.1.2-6.1.4: T = raw / 100 degC
 * They match §2 item 6's ranges, +/-16 g and +/-2000 deg/s (p.5). Chapter 7
 * gives no scale for the registers; §7.2's example (p.49) converts the
 * angle registers by the frame formula ("section 5.1.4", meaning 6.1.4), so
 * every register is taken to use its frame's scale.
 */
#define JY901_ACCEL_FULL_SCALE_G	16.0f
#define JY901_GYRO_FULL_SCALE_DPS	2000.0f
#define JY901_ANGLE_FULL_SCALE_DEG	180.0f
#define JY901_RAW_FULL_SCALE		32768.0f
#define JY901_TEMP_LSB_PER_C		100.0f

/*
 * A signed 16-bit value, low byte first: in a frame (§6.1.2 note 2, p.31)
 * and in a register (§7.2, p.49) alike.
 */
static inline int16_t jy901_value(const uint8_t *p)
{
	return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}

/*
 * The SUM byte: 0x55 + the frame's own type + its eight data bytes, as
 * §6.1.2, 6.1.3, 6.1.4 and 6.1.10 give it. The other sections' formulas
 * name the wrong type while listing that frame's own data bytes (0x51 for
 * 0x50 in §6.1.1, 0x53 for 0x54 in §6.1.5, ...); they are taken as typos.
 * The document never says the sum is cut to a byte; SUM is one byte, so
 * this keeps the low 8 bits.
 */
static inline uint8_t jy901_sum(const uint8_t *frame)
{
	uint8_t sum = 0;

	for (int i = 0; i < JY901_FRAME_LEN - 1; i++)
		sum = (uint8_t)(sum + frame[i]);
	return sum;
}

static inline bool jy901_known_type(uint8_t type)
{
	return type >= JY901_TYPE_FIRST && type <= JY901_TYPE_LAST;
}

/* A header, a known type and a matching SUM. */
static inline bool jy901_frame_valid(const uint8_t *frame)
{
	return frame[0] == JY901_FRAME_HEADER && jy901_known_type(frame[1]) &&
	       jy901_sum(frame) == frame[JY901_FRAME_LEN - 1];
}

static inline float jy901_scaled(int16_t raw, float full_scale)
{
	return (float)raw / JY901_RAW_FULL_SCALE * full_scale;
}

#endif /* JY901_REGS_H */
