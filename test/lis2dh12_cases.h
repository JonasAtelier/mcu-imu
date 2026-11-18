/*
 * The LIS2DH12's cases, shared by esp/test_lis2dh12.c and
 * stm/test_lis2dh12.c.
 *
 * A register-level fake LIS2DH12 with the rules of the register map the
 * driver was written from (docs/chips.md "LIS2DH12", all on its UNVERIFIED
 * list) that a driver can break without noticing:
 *
 *   - a read of more than one byte without bit 7 of the sub-address set
 *     repeats the first register (6.1.1);
 *   - LPen and HR set at once is not allowed (table "Operating mode
 *     selection"), counted the moment a write makes it so;
 *   - CTRL_REG0 bits 6:0 must stay 0010000, INT1_THS and INT1_DURATION bit
 *     7 must be 0, and read-only or reserved addresses take no writes;
 *   - no soft reset: only chip_power_on() resets, so a probe after an MCU
 *     reset finds whatever the last run left (chip_dirty());
 *   - INT1_SRC clears on read, and the INT1 pin is the event routed by
 *     CTRL_REG3.I1_IA1.
 *
 * Each platform file includes this after providing:
 *
 *   probe_on(imu, config)    the probe with a valid bus config
 *   allocs, frees            calloc() and free() calls by the driver
 * and hands its bus calls to chip_i2c_read() and chip_i2c_write().
 */

#define REG(name)	LIS2DH12_##name

static struct {
	uint8_t regs[128];
	uint8_t who;
	int fail_in;
	int transfers, writes, violations;
	int reference_reads;
	bool filter_on_at_reference;	/* HP_IA1 set when REFERENCE was read */
} chip;

/* Power-on values (chapter 9): CTRL_REG0 0x10, CTRL_REG1 0x07, else 0. */
static void chip_power_on(void)
{
	memset(&chip, 0, sizeof(chip));
	chip.who = LIS2DH12_WHO_AM_I_VALUE;
	chip.fail_in = -1;
	chip.regs[REG(WHO_AM_I)] = chip.who;
	chip.regs[REG(CTRL_REG0)] = 0x10;
	chip.regs[REG(CTRL_REG1)] = 0x07;
}

/* What an earlier run may have left: everything on, an event latched. */
static void chip_dirty(void)
{
	chip.regs[REG(TEMP_CFG_REG)] = 0xC0;
	chip.regs[REG(CTRL_REG1)] = 0x97;		/* 1344 Hz, HR below */
	chip.regs[REG(CTRL_REG2)] = 0x39;
	chip.regs[REG(CTRL_REG3)] = 0xC0;		/* click, IA1 on INT1 */
	chip.regs[REG(CTRL_REG4)] = 0xB8;		/* BDU, 16 g, HR */
	chip.regs[REG(CTRL_REG5)] = 0x48;		/* FIFO, latch */
	chip.regs[REG(CTRL_REG6)] = 0xFA;
	chip.regs[REG(FIFO_CTRL_REG)] = 0x80;
	chip.regs[REG(INT1_CFG)] = 0x2A;
	chip.regs[REG(INT2_CFG)] = 0x3F;
	chip.regs[REG(CLICK_CFG)] = 0x15;
	chip.regs[REG(ACT_THS)] = 0x10;
	chip.regs[REG(INT1_SRC)] = 0x62;		/* latched event */
}

static bool chip_pin(void)
{
	return (chip.regs[REG(INT1_SRC)] & LIS2DH12_IA) &&
	       (chip.regs[REG(CTRL_REG3)] & LIS2DH12_I1_IA1);
}

static bool chip_writable(uint8_t reg)
{
	return (reg >= 0x1E && reg <= 0x26) || reg == 0x2E || reg == 0x30 ||
	       (reg >= 0x32 && reg <= 0x34) || (reg >= 0x36 && reg <= 0x38) ||
	       (reg >= 0x3A && reg <= 0x3F);
}

static bool chip_fails(void)
{
	chip.transfers++;
	return chip.fail_in >= 0 && chip.fail_in-- == 0;
}

static void chip_write(uint8_t reg, uint8_t v)
{
	chip.writes++;
	if (!chip_writable(reg)) {
		chip.violations++;
		return;
	}
	if (reg == REG(CTRL_REG0) && (v & 0x7F) != 0x10)
		chip.violations++;
	if ((reg == REG(INT1_THS) || reg == REG(INT1_DURATION)) && (v & 0x80))
		chip.violations++;
	chip.regs[reg] = v;
	if ((chip.regs[REG(CTRL_REG1)] & LIS2DH12_LPEN) &&
	    (chip.regs[REG(CTRL_REG4)] & LIS2DH12_HR))
		chip.violations++;
}

static void chip_read(uint8_t sub, uint8_t *out, size_t len)
{
	bool increment = sub & LIS2DH12_AUTO_INCREMENT;
	uint8_t first = sub & 0x7F;

	for (size_t i = 0; i < len; i++) {
		uint8_t r = (uint8_t)(increment ? first + i : first);

		out[i] = chip.regs[r & 0x7F];
		if (r == REG(INT1_SRC))
			chip.regs[r] = 0;
		if (r == REG(REFERENCE)) {
			chip.reference_reads++;
			chip.filter_on_at_reference =
				chip.regs[REG(CTRL_REG2)] & LIS2DH12_HP_IA1;
		}
	}
}

static bool chip_i2c_read(uint8_t sub, uint8_t *out, size_t len)
{
	if (chip_fails())
		return false;
	chip_read(sub, out, len);
	return true;
}

static bool chip_i2c_write(uint8_t sub, const uint8_t *data, size_t len)
{
	if (chip_fails())
		return false;
	assert(len == 1);	/* the driver writes one register at a time */
	chip_write(sub & 0x7F, data[0]);
	return true;
}

/* ------------------------------------------------------------ helpers ---- */

static int probe_on(struct imu_dev *imu, const struct lis2dh12_config *config);

/* What the badge runs: low power, 50 Hz, 8 g. */
static const struct lis2dh12_config base = {
	.mode = LIS2DH12_MODE_LOW_POWER,
	.odr = LIS2DH12_ODR_50HZ,
	.full_scale = LIS2DH12_FS_8G,
};

static const struct lis2dh12_motion_config shake = {
	.axes = LIS2DH12_AXIS_ALL,
	.threshold_g = 1.5f,
	.high_pass = true,
	.latched = true,
};

/* One axis's word, left-justified, low byte first. */
static void put_axis(int axis, int16_t counts, int mode)
{
	uint16_t word = (uint16_t)(counts * (1 << (16 -
				   lis2dh12_resolution_bits(mode))));

	chip.regs[REG(OUT_X_L) + axis * 2] = (uint8_t)(word & 0xFF);
	chip.regs[REG(OUT_X_L) + axis * 2 + 1] = (uint8_t)(word >> 8);
}

static bool near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* ------------------------------------------------------------- cases ---- */

/* Every control register to a known state, over whatever was there. */
static void test_probe(void)
{
	struct lis2dh12_config c = base;
	struct imu_dev imu = { 0 };

	chip_power_on();
	chip_dirty();
	allocs = frees = 0;
	assert(probe_on(&imu, &base) == 0);
	assert(imu.ops && strcmp(imu.name, "lis2dh12") == 0);
	assert(allocs == 1 && frees == 0);
	/* coming from high resolution into low power: HR went first */
	assert(chip.violations == 0);
	assert(chip.regs[REG(CTRL_REG0)] == 0x10);
	assert(chip.regs[REG(TEMP_CFG_REG)] == 0x00);
	/* 50 Hz (0x4 << 4), LPen, X Y Z */
	assert(chip.regs[REG(CTRL_REG1)] == 0x4F);
	assert(chip.regs[REG(CTRL_REG2)] == 0x00);
	assert(chip.regs[REG(CTRL_REG3)] == 0x00);
	/* BDU, 8 g (2 << 4), HR off */
	assert(chip.regs[REG(CTRL_REG4)] == 0xA0);
	assert(chip.regs[REG(CTRL_REG5)] == 0x00);
	assert(chip.regs[REG(CTRL_REG6)] == 0x00);
	assert(chip.regs[REG(FIFO_CTRL_REG)] == 0x00);
	assert(chip.regs[REG(INT1_CFG)] == 0x00);
	assert(chip.regs[REG(INT2_CFG)] == 0x00);
	assert(chip.regs[REG(CLICK_CFG)] == 0x00);
	assert(chip.regs[REG(ACT_THS)] == 0x00);
	/* the old latched event is gone */
	assert(chip.regs[REG(INT1_SRC)] == 0 && !chip_pin());
	/* 64 mg a count: 15.625 counts per g */
	assert(lis2dh12_accel_sensitivity(&imu) == 15.625f);

	lis2dh12_remove(&imu);
	assert((chip.regs[REG(CTRL_REG1)] >> 4) == 0);	/* power-down */
	assert(chip.violations == 0);
	assert(!imu.ops && !imu.priv && frees == 1);
	lis2dh12_remove(&imu);

	/* The options: pull-up off, active low, slowest filter, HR at 16 g */
	c.sa0_pullup_off = true;
	c.int_active_low = true;
	c.high_pass_cutoff = 3;
	c.mode = LIS2DH12_MODE_HIGH_RES;
	c.odr = LIS2DH12_ODR_FASTEST;
	c.full_scale = LIS2DH12_FS_16G;
	chip_power_on();
	assert(probe_on(&imu, &c) == 0);
	assert(chip.regs[REG(CTRL_REG0)] == 0x90);
	assert(chip.regs[REG(CTRL_REG6)] == 0x02);
	assert(chip.regs[REG(CTRL_REG2)] == 0x30);
	assert(chip.regs[REG(CTRL_REG1)] == 0x97);	/* 0x9, no LPen */
	assert(chip.regs[REG(CTRL_REG4)] == 0xB8);	/* BDU, 16 g, HR */
	assert(near(lis2dh12_accel_sensitivity(&imu), 1000.0f / 12.0f,
		    1e-3f));
	assert(chip.violations == 0);
	lis2dh12_remove(&imu);

	/* Normal mode: neither bit */
	c = base;
	c.mode = LIS2DH12_MODE_NORMAL;
	c.odr = LIS2DH12_ODR_400HZ;
	c.full_scale = LIS2DH12_FS_2G;
	chip_power_on();
	chip_dirty();
	assert(probe_on(&imu, &c) == 0);
	assert(chip.regs[REG(CTRL_REG1)] == 0x77);
	assert(chip.regs[REG(CTRL_REG4)] == 0x80);
	assert(chip.violations == 0);
	lis2dh12_remove(&imu);
}

/* Every mode and full scale: one burst, decoded and scaled. */
static void test_read(void)
{
	static const int16_t counts[3][3] = {
		{ 100, -37, 1 },		/* low power, 8-bit */
		{ 400, -150, -512 },		/* normal, 10-bit */
		{ 1600, -600, 2047 },		/* high resolution, 12-bit */
	};
	struct imu_dev imu = { 0 };
	struct imu_sample s;
	int16_t raw[3];
	int transfers;

	chip_power_on();
	assert(probe_on(&imu, &base) == 0);
	for (int mode = 0; mode < 3; mode++) {
		for (int fs = 0; fs < 4; fs++) {
			float mg = lis2dh12_mg_per_count(mode, fs);

			assert(lis2dh12_set_mode(&imu, mode,
						 LIS2DH12_ODR_100HZ, fs) == 0);
			for (int axis = 0; axis < 3; axis++)
				put_axis(axis, counts[mode][axis], mode);
			transfers = chip.transfers;
			assert(lis2dh12_read_raw(&imu, raw) == 0);
			/* one burst, so the auto-increment bit was set */
			assert(chip.transfers == transfers + 1);
			assert(raw[0] == counts[mode][0] &&
			       raw[1] == counts[mode][1] &&
			       raw[2] == counts[mode][2]);
			assert(imu_read(&imu, &s) == 0);
			for (int axis = 0; axis < 3; axis++) {
				float want = counts[mode][axis] * mg / 1000.0f *
					     9.80665f;

				assert(near(s.accel[axis], want,
					    fabsf(want) * 1e-5f + 1e-5f));
				assert(isnan(s.gyro[axis]));
			}
			assert(isnan(s.temp_c));
		}
	}
	assert(chip.violations == 0);

	/* Typed in by hand: 0x3E80 is 1 g in HR at 2 g, 2 g in normal at
	 * 4 g; 0xFB00 is -0.96 g in low power at 16 g */
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_HIGH_RES,
				 LIS2DH12_ODR_100HZ, LIS2DH12_FS_2G) == 0);
	chip.regs[0x28] = 0x80;
	chip.regs[0x29] = 0x3E;
	assert(imu_read(&imu, &s) == 0 && near(s.accel[0], 9.80665f, 1e-4f));
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_NORMAL,
				 LIS2DH12_ODR_100HZ, LIS2DH12_FS_4G) == 0);
	assert(imu_read(&imu, &s) == 0 &&
	       near(s.accel[0], 2.0f * 9.80665f, 1e-4f));
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_100HZ, LIS2DH12_FS_16G) == 0);
	chip.regs[0x28] = 0x00;
	chip.regs[0x29] = 0xFB;
	assert(imu_read(&imu, &s) == 0 &&
	       near(s.accel[0], -0.96f * 9.80665f, 1e-4f));

	/* A failed read leaves the sample alone */
	s.accel[0] = 99.0f;
	chip.fail_in = 0;
	assert(imu_read(&imu, &s) != 0 && s.accel[0] == 99.0f);
	lis2dh12_remove(&imu);
}

/* Mode changes never set LPen and HR together; bad codes refused. */
static void test_modes(void)
{
	struct imu_dev imu = { 0 };
	int transfers;

	chip_power_on();
	assert(probe_on(&imu, &base) == 0);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_HIGH_RES,
				 LIS2DH12_ODR_25HZ, LIS2DH12_FS_4G) == 0);
	assert(chip.regs[REG(CTRL_REG1)] == 0x37);
	assert(chip.regs[REG(CTRL_REG4)] == 0x98);
	assert(lis2dh12_accel_sensitivity(&imu) == 500.0f);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_1620HZ, LIS2DH12_FS_4G) == 0);
	assert(chip.regs[REG(CTRL_REG1)] == 0x8F);
	assert(chip.regs[REG(CTRL_REG4)] == 0x90);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_HIGH_RES,
				 LIS2DH12_ODR_FASTEST, LIS2DH12_FS_4G) == 0);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_NORMAL,
				 LIS2DH12_ODR_1HZ, LIS2DH12_FS_4G) == 0);
	assert(chip.regs[REG(CTRL_REG1)] == 0x17);
	assert(chip.regs[REG(CTRL_REG4)] == 0x90);
	assert(lis2dh12_accel_sensitivity(&imu) == 125.0f);
	/* power-down is a rate like the others after probe */
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_POWER_DOWN, LIS2DH12_FS_4G) == 0);
	assert(chip.regs[REG(CTRL_REG1)] == 0x0F);
	assert(chip.violations == 0);

	/* refused before any transfer */
	transfers = chip.transfers;
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_NORMAL,
				 LIS2DH12_ODR_1620HZ, LIS2DH12_FS_4G) == -EINVAL);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_HIGH_RES,
				 LIS2DH12_ODR_1620HZ, LIS2DH12_FS_4G) == -EINVAL);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_NORMAL, 0xA,
				 LIS2DH12_FS_4G) == -EINVAL);
	assert(lis2dh12_set_mode(&imu, 3, LIS2DH12_ODR_50HZ,
				 LIS2DH12_FS_4G) == -EINVAL);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_NORMAL,
				 LIS2DH12_ODR_50HZ, 4) == -EINVAL);
	assert(chip.transfers == transfers);
	lis2dh12_remove(&imu);
}

/* The motion detector: registers, order, the event and its release. */
static void test_motion(void)
{
	struct lis2dh12_motion_config m = shake;
	struct imu_dev imu = { 0 };
	uint8_t axes = 0xFF;
	bool active = true;
	int transfers, references;

	chip_power_on();
	assert(probe_on(&imu, &base) == 0);
	chip.regs[REG(INT1_SRC)] = 0x42;	/* stale event, before */
	assert(lis2dh12_set_motion(&imu, &shake) == 0);
	assert(chip.violations == 0);
	assert(chip.regs[REG(INT1_THS)] == 24);		/* 1.5 g / 62 mg */
	assert(chip.regs[REG(INT1_DURATION)] == 0);
	assert(chip.regs[REG(INT1_CFG)] == 0x2A);	/* XHIE YHIE ZHIE, OR */
	assert(chip.regs[REG(CTRL_REG2)] == 0x01);	/* HP_IA1 */
	assert(chip.regs[REG(CTRL_REG5)] == 0x08);	/* LIR_INT1 */
	assert(chip.regs[REG(CTRL_REG3)] == 0x40);	/* I1_IA1 */
	/* the filter reset after it was switched on, the stale event gone */
	assert(chip.reference_reads == 1 && chip.filter_on_at_reference);
	assert(!chip_pin());

	/* An event on X and Z: read, decoded, released */
	chip.regs[REG(INT1_SRC)] = 0x62;
	assert(chip_pin());
	assert(lis2dh12_read_motion(&imu, &active, &axes) == 0);
	assert(active && axes == (LIS2DH12_AXIS_X | LIS2DH12_AXIS_Z));
	assert(!chip_pin());
	assert(lis2dh12_read_motion(&imu, &active, NULL) == 0 && !active);
	assert(lis2dh12_read_motion(&imu, NULL, &axes) == -EINVAL);

	/* A new full scale rewrites the threshold for its step */
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_50HZ, LIS2DH12_FS_2G) == 0);
	assert(chip.regs[REG(INT1_THS)] == 94);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_50HZ, LIS2DH12_FS_16G) == 0);
	assert(chip.regs[REG(INT1_THS)] == 8);
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_50HZ, LIS2DH12_FS_4G) == 0);
	assert(chip.regs[REG(INT1_THS)] == 47);
	/* one that would not fit the new step is refused, nothing sent */
	m.threshold_g = 2.5f;
	assert(lis2dh12_set_motion(&imu, &m) == 0);
	assert(chip.regs[REG(INT1_THS)] == 78);
	transfers = chip.transfers;
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_50HZ, LIS2DH12_FS_2G) == -EINVAL);
	assert(chip.transfers == transfers);
	assert(chip.regs[REG(CTRL_REG4)] == 0x90);

	/* Not latched, no filter, one axis, a duration */
	m = (struct lis2dh12_motion_config){
		.axes = LIS2DH12_AXIS_Y, .threshold_g = 0.5f, .duration = 5,
	};
	references = chip.reference_reads;
	assert(lis2dh12_set_motion(&imu, &m) == 0);
	assert(chip.regs[REG(INT1_CFG)] == 0x08);
	assert(chip.regs[REG(INT1_THS)] == 16);		/* 0.5 g / 32 mg */
	assert(chip.regs[REG(INT1_DURATION)] == 5);
	assert(chip.regs[REG(CTRL_REG2)] == 0x00);
	assert(chip.regs[REG(CTRL_REG5)] == 0x00);
	assert(chip.regs[REG(CTRL_REG3)] == 0x40);
	assert(chip.reference_reads == references);	/* no filter, no reset */

	/* Refused before any transfer */
	transfers = chip.transfers;
	m = shake;
	m.threshold_g = 0.0f;
	assert(lis2dh12_set_motion(&imu, &m) == -EINVAL);
	m.threshold_g = 0.01f;				/* 0 steps at 4 g */
	assert(lis2dh12_set_motion(&imu, &m) == -EINVAL);
	m.threshold_g = 4.1f;				/* 128 steps */
	assert(lis2dh12_set_motion(&imu, &m) == -EINVAL);
	m = shake;
	m.duration = 128;
	assert(lis2dh12_set_motion(&imu, &m) == -EINVAL);
	m = shake;
	m.axes = 0x08;
	assert(lis2dh12_set_motion(&imu, &m) == -EINVAL);
	assert(lis2dh12_set_motion(&imu, NULL) == -EINVAL);
	assert(chip.transfers == transfers);

	/* Off: unrouted, disabled, unlatched, filter off, event read away */
	assert(lis2dh12_set_motion(&imu, &shake) == 0);
	chip.regs[REG(INT1_SRC)] = 0x42;
	m = (struct lis2dh12_motion_config){ .axes = 0 };
	assert(lis2dh12_set_motion(&imu, &m) == 0);
	assert(chip.regs[REG(CTRL_REG3)] == 0 && chip.regs[REG(INT1_CFG)] == 0);
	assert(chip.regs[REG(CTRL_REG5)] == 0 && chip.regs[REG(CTRL_REG2)] == 0);
	assert(chip.regs[REG(INT1_SRC)] == 0);
	/* and a full scale it would not have fitted is now fine */
	assert(lis2dh12_set_mode(&imu, LIS2DH12_MODE_LOW_POWER,
				 LIS2DH12_ODR_50HZ, LIS2DH12_FS_16G) == 0);
	assert(chip.violations == 0);
	lis2dh12_remove(&imu);
}

/* STATUS_REG.ZYXDA alone, and the escape hatch. */
static void test_status_and_registers(void)
{
	struct imu_dev imu = { 0 };
	uint8_t b[6] = { 0 };
	bool ready = false;

	chip_power_on();
	assert(probe_on(&imu, &base) == 0);
	chip.regs[REG(STATUS_REG)] = 0x08;
	assert(lis2dh12_data_ready(&imu, &ready) == 0 && ready);
	chip.regs[REG(STATUS_REG)] = 0xF7;		/* every bit but ZYXDA */
	assert(lis2dh12_data_ready(&imu, &ready) == 0 && !ready);
	assert(lis2dh12_data_ready(&imu, NULL) == -EINVAL);

	assert(lis2dh12_reg_read(&imu, REG(WHO_AM_I), b, 1) == 0 &&
	       b[0] == 0x33);
	put_axis(0, 1, 2);
	put_axis(1, 2, 2);
	put_axis(2, 3, 2);
	assert(lis2dh12_reg_read(&imu, REG(OUT_X_L), b, 6) == 0);
	assert(b[1] == 0x00 && b[0] == 0x10 && b[2] == 0x20 && b[4] == 0x30);
	assert(lis2dh12_reg_write(&imu, 0x3A, 0x05) == 0);	/* CLICK_THS */
	assert(chip.regs[0x3A] == 0x05);
	assert(lis2dh12_reg_read(&imu, 0xA8, b, 6) == -EINVAL);
	assert(lis2dh12_reg_read(&imu, 0x28, b, 0) == -EINVAL);
	assert(lis2dh12_reg_read(&imu, 0x28, NULL, 1) == -EINVAL);
	assert(lis2dh12_reg_write(&imu, 0xBA, 0) == -EINVAL);
	assert(chip.violations == 0);
	lis2dh12_remove(&imu);
}

/* Refused before anything is allocated or sent. */
static void test_refused(void)
{
	static const struct imu_ops other_ops = { 0 };
	struct lis2dh12_config c;
	struct imu_dev imu = { 0 };
	struct imu_dev other = { .ops = &other_ops };
	int16_t raw[3];
	uint8_t axes;
	bool b;

	chip_power_on();
	allocs = 0;
#define REFUSED(field, value) do { \
		c = base; \
		c.field = value; \
		assert(probe_on(&imu, &c) == -EINVAL); \
	} while (0)
	REFUSED(mode, 3);
	REFUSED(odr, LIS2DH12_ODR_POWER_DOWN);		/* set_mode() for that */
	REFUSED(odr, 0xA);
	REFUSED(full_scale, 4);
	REFUSED(high_pass_cutoff, 4);
	c = base;
	c.mode = LIS2DH12_MODE_NORMAL;
	c.odr = LIS2DH12_ODR_1620HZ;			/* low power only */
	assert(probe_on(&imu, &c) == -EINVAL);
#undef REFUSED
	assert(probe_on(&imu, NULL) == -EINVAL);
	assert(allocs == 0 && chip.transfers == 0 && !imu.ops);

	/* Already bound */
	assert(probe_on(&imu, &base) == 0);
	allocs = 0;
	assert(probe_on(&imu, &base) == -EBUSY && allocs == 0);
	lis2dh12_remove(&imu);

	/* Another driver's imu_dev */
	assert(lis2dh12_read_raw(&other, raw) == -EINVAL);
	assert(lis2dh12_data_ready(&other, &b) == -EINVAL);
	assert(lis2dh12_set_mode(&other, 0, 1, 0) == -EINVAL);
	assert(lis2dh12_set_motion(&other, &shake) == -EINVAL);
	assert(lis2dh12_read_motion(&other, &b, &axes) == -EINVAL);
	assert(lis2dh12_reg_read(&other, 0x0F, &b, 1) == -EINVAL);
	assert(lis2dh12_reg_write(&other, 0x20, 0) == -EINVAL);
	assert(lis2dh12_accel_sensitivity(&other) == 0.0f);
	assert(lis2dh12_accel_sensitivity(NULL) == 0.0f);
	lis2dh12_remove(&other);
	assert(other.ops == &other_ops);
}

static void test_failures(void)
{
	struct imu_dev imu = { 0 };
	int t;

	/* Someone else answers: nothing written to it */
	chip_power_on();
	chip.who = chip.regs[REG(WHO_AM_I)] = 0x44;
	allocs = frees = 0;
	assert(probe_on(&imu, &base) == -ENODEV);
	assert(!imu.ops && allocs == frees && chip.writes == 0);

	/* A bus error at every transfer of a full probe: nothing bound or
	 * leaked, and the next probe, chip still powered, works */
	chip_power_on();
	chip_dirty();
	assert(probe_on(&imu, &base) == 0);
	t = chip.transfers;
	lis2dh12_remove(&imu);
	for (int i = 0; i < t; i++) {
		chip_power_on();
		chip_dirty();
		chip.fail_in = i;
		allocs = frees = 0;
		assert(probe_on(&imu, &base) < 0);
		assert(!imu.ops && !imu.priv && allocs == frees);
		assert(probe_on(&imu, &base) == 0);
		assert(chip.violations == 0);
		lis2dh12_remove(&imu);
	}

	/* The motion detector set up again after a failure at each step */
	chip_power_on();
	assert(probe_on(&imu, &base) == 0);
	t = chip.transfers;
	assert(lis2dh12_set_motion(&imu, &shake) == 0);
	t = chip.transfers - t;
	for (int i = 0; i < t; i++) {
		chip.fail_in = i;
		assert(lis2dh12_set_motion(&imu, &shake) < 0);
		assert(lis2dh12_set_motion(&imu, &shake) == 0);
		assert(chip.regs[REG(CTRL_REG3)] == 0x40 &&
		       chip.regs[REG(INT1_THS)] == 24);
	}
	assert(chip.violations == 0);
	lis2dh12_remove(&imu);
}

static void run_lis2dh12_cases(void)
{
	test_probe();
	test_read();
	test_modes();
	test_motion();
	test_status_and_registers();
	test_refused();
	test_failures();
}
