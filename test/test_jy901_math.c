/*
 * The JY901 frame rules, register layout and scales against docs/WT901.pdf,
 * with one frame worked by hand rather than through the header's own sum,
 * and chapter 7's own worked register read.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "jy901_regs.h"

static int near(float a, float b, float tolerance)
{
	return fabsf(a - b) < tolerance;
}

int main(void)
{
	/*
	 * An accelerometer frame (§6.1.2): ax = 2048 (1 g), ay = az = 0,
	 * T = 2500 (25.00 degC). SUM by hand: 0x55 + 0x51 + 0x00 + 0x08 +
	 * 0xC4 + 0x09 = 0x17B, low byte 0x7B.
	 */
	uint8_t frame[JY901_FRAME_LEN] = {
		0x55, 0x51, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x09, 0x7B,
	};

	assert(jy901_sum(frame) == 0x7B);
	assert(jy901_frame_valid(frame));
	assert(jy901_value(frame + 2) == 2048);
	assert(near(jy901_scaled(jy901_value(frame + 2),
				 JY901_ACCEL_FULL_SCALE_G), 1.0f, 1e-3f));
	assert(near((float)jy901_value(frame + 8) / JY901_TEMP_LSB_PER_C,
		    25.0f, 1e-3f));

	/* Any one byte off breaks it: header, type, data or SUM. */
	frame[10] = 0x7C;
	assert(!jy901_frame_valid(frame));
	frame[10] = 0x7B;
	frame[3] = 0x09;
	assert(!jy901_frame_valid(frame));
	frame[3] = 0x08;
	frame[0] = 0x54;
	assert(!jy901_frame_valid(frame));
	frame[0] = 0x55;
	/* An unknown type fails even with its SUM made to match. */
	frame[1] = 0x5B;	/* past §6.1.11's 0x5A */
	frame[10] = jy901_sum(frame);
	assert(!jy901_frame_valid(frame));
	frame[1] = 0x4F;	/* before §6.1.1's 0x50 */
	frame[10] = jy901_sum(frame);
	assert(!jy901_frame_valid(frame));

	/*
	 * §7.2 p.49, read from 0x3d: 0x9C 0x82 0x28 0xFF 0xE6 0x24 is roll
	 * -176.33, pitch -1.19, yaw 51.89 degrees, as the document prints them.
	 */
	{
		const uint8_t angles[6] = { 0x9C, 0x82, 0x28, 0xFF, 0xE6, 0x24 };

		assert(jy901_value(angles) == (int16_t)0x829C);
		assert(near(jy901_scaled(jy901_value(angles),
					 JY901_ANGLE_FULL_SCALE_DEG),
			    -176.33f, 0.005f));
		assert(near(jy901_scaled(jy901_value(angles + 2),
					 JY901_ANGLE_FULL_SCALE_DEG),
			    -1.19f, 0.005f));
		assert(near(jy901_scaled(jy901_value(angles + 4),
					 JY901_ANGLE_FULL_SCALE_DEG),
			    51.89f, 0.005f));
	}

	/* §6.1.2 note 2: low byte first, signed. */
	{
		const uint8_t negative[2] = { 0x00, 0x80 };

		assert(jy901_value(negative) == -32768);
	}

	/* §6.1.2-6.1.4 full scales, and §2 item 6's ranges. */
	assert(near(jy901_scaled(-32768, JY901_ACCEL_FULL_SCALE_G), -16.0f,
		    1e-3f));
	assert(near(jy901_scaled(16384, JY901_GYRO_FULL_SCALE_DPS), 1000.0f,
		    1e-3f));
	assert(near(jy901_scaled(16384, JY901_ANGLE_FULL_SCALE_DEG), 90.0f,
		    1e-3f));

	/* The burst: AX to TEMP of the chapter 7 table, p.47-48. */
	assert(JY901_REG_AX == 0x34 && JY901_REG_GX == 0x37 &&
	       JY901_REG_ROLL == 0x3d && JY901_REG_TEMP == 0x40);
	assert(JY901_BURST_REGS == 13 && JY901_BURST_LEN == 26);
	assert(JY901_BURST_ACCEL == 0 && JY901_BURST_GYRO == 3 &&
	       JY901_BURST_TEMP == 12);
	assert(JY901_REG_IICADDR == 0x1a && JY901_I2C_ADDRESS == 0x50 &&
	       JY901_I2C_ADDRESS_MAX == 0x7f);

	/* Byte values typed from the document. */
	assert(JY901_FRAME_HEADER == 0x55 && JY901_FRAME_LEN == 11);
	assert(JY901_TYPE_ACCEL == 0x51 && JY901_TYPE_GYRO == 0x52 &&
	       JY901_TYPE_ANGLE == 0x53);
	assert(JY901_COMMAND_1 == 0xFF && JY901_COMMAND_2 == 0xAA);

	printf("all jy901 math checks passed\n");
	return 0;
}
