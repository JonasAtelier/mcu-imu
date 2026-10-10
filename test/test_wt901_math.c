/*
 * The WT901 frame rules and scales against docs/WT901.pdf, with one frame
 * worked by hand rather than through the header's own sum.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "wt901_proto.h"

static int near(float a, float b)
{
	return fabsf(a - b) < 1e-3f;
}

int main(void)
{
	/*
	 * An accelerometer frame (§6.1.2): ax = 2048 (1 g), ay = az = 0,
	 * T = 2500 (25.00 degC). SUM by hand: 0x55 + 0x51 + 0x00 + 0x08 +
	 * 0xC4 + 0x09 = 0x17B, low byte 0x7B.
	 */
	uint8_t frame[WT901_FRAME_LEN] = {
		0x55, 0x51, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x09, 0x7B,
	};

	assert(wt901_sum(frame) == 0x7B);
	assert(wt901_frame_valid(frame));
	assert(wt901_value(frame + 2) == 2048);
	assert(near(wt901_scaled(wt901_value(frame + 2),
				 WT901_ACCEL_FULL_SCALE_G), 1.0f));
	assert(near((float)wt901_value(frame + 8) / WT901_TEMP_LSB_PER_C,
		    25.0f));

	/* Any one byte off breaks it: header, type, data or SUM. */
	frame[10] = 0x7C;
	assert(!wt901_frame_valid(frame));
	frame[10] = 0x7B;
	frame[0] = 0x54;
	assert(!wt901_frame_valid(frame));
	frame[0] = 0x55;
	/* An unknown type fails even with its SUM made to match. */
	frame[1] = 0x5B;	/* past §6.1.11's 0x5A */
	frame[10] = wt901_sum(frame);
	assert(!wt901_frame_valid(frame));
	frame[1] = 0x4F;	/* before §6.1.1's 0x50 */
	frame[10] = wt901_sum(frame);
	assert(!wt901_frame_valid(frame));

	/* §6.1.2 note 2: low byte first, signed. */
	{
		const uint8_t neg[2] = { 0x00, 0x80 };

		assert(wt901_value(neg) == -32768);
	}

	/* §6.1.2-6.1.4 full scales, and §2 item 6's ranges. */
	assert(near(wt901_scaled(-32768, WT901_ACCEL_FULL_SCALE_G), -16.0f));
	assert(near(wt901_scaled(16384, WT901_GYRO_FULL_SCALE_DPS), 1000.0f));
	assert(near(wt901_scaled(16384, WT901_ANGLE_FULL_SCALE_DEG), 90.0f));
	assert(near(wt901_scaled(-32768, WT901_ANGLE_FULL_SCALE_DEG),
		    -180.0f));

	/* Byte values typed from the document. */
	assert(WT901_FRAME_HEADER == 0x55 && WT901_FRAME_LEN == 11);
	assert(WT901_TYPE_ACCEL == 0x51 && WT901_TYPE_GYRO == 0x52 &&
	       WT901_TYPE_ANGLE == 0x53);
	assert(WT901_COMMAND_1 == 0xFF && WT901_COMMAND_2 == 0xAA);

	printf("all wt901 math checks passed\n");
	return 0;
}
