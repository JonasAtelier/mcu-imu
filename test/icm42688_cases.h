/*
 * The ICM-42688-P's cases, shared by esp/test_icm42688.c and
 * stm/test_icm42688.c, each run on I2C and on SPI.
 *
 * A register-level fake ICM-42688-P with its register banks and the rules
 * of the datasheet (DS-000347 rev. 1.6) a driver can break without noticing:
 *
 *   - registers in banks 0, 1, 2 and 4, chosen by REG_BANK_SEL at 0x76 in
 *     every bank (13, 14.59): a write to the wrong bank lands in the wrong
 *     register, and the tests read each bank's own;
 *   - after a soft reset nothing is heard for 1 ms (14.1);
 *   - while a sensor runs, only the rate, full scale and power mode may
 *     change (12.9); after a sensor comes on, no write for 200 us (14.36);
 *   - reserved bits keep their reset values (the tests check them);
 *   - INT_STATUS clears on read (14.21); a latched data-ready pin also on a
 *     sensor data read when INT_CONFIG0 says so (14.49).
 *
 * Each platform file includes this after providing:
 *
 *   enum bus { BUS_I2C, BUS_SPI }
 *   probe_on(imu, bus, config)   that bus's probe with a valid bus config
 *   allocs, frees                calloc() and free() calls by the driver
 * and hands its bus calls to chip_i2c_read(), chip_i2c_write() and
 * chip_spi_frame(), and its waits to now_us.
 */

static struct {
	uint8_t regs[5][128];		/* banks 0..4 */
	uint8_t bank;
	int64_t quiet_until_us;		/* nothing heard before then */
	int64_t no_write_until_us;	/* no writes before then (14.36) */
	bool pin;			/* INT1 asserted, latched mode */
	uint8_t who;
	int fail_in;
	int transfers, resets, violations;
	int bank_writes;		/* REG_BANK_SEL writes heard */
} chip;

static int64_t now_us;

/* Reset values (chapters 14-16) of every register the driver touches. */
static void chip_reset_registers(void)
{
	memset(chip.regs, 0, sizeof(chip.regs));
	chip.bank = 0;
	chip.regs[0][ICM42688_DRIVE_CONFIG] = 0x05;
	chip.regs[0][ICM42688_TEMP_DATA1] = 0x80;
	chip.regs[0][ICM42688_INT_STATUS] = 0x10;
	chip.regs[0][ICM42688_INTF_CONFIG0] = 0x30;
	chip.regs[0][0x4D] = 0x91;			/* INTF_CONFIG1 */
	chip.regs[0][ICM42688_GYRO_CONFIG0] = 0x06;
	chip.regs[0][ICM42688_ACCEL_CONFIG0] = 0x06;
	chip.regs[0][ICM42688_GYRO_CONFIG1] = 0x16;
	chip.regs[0][ICM42688_GYRO_ACCEL_CONFIG0] = 0x11;
	chip.regs[0][ICM42688_ACCEL_CONFIG1] = 0x0D;
	chip.regs[0][ICM42688_INT_CONFIG1] = 0x10;
	chip.regs[0][ICM42688_INT_SOURCE0] = 0x10;
	chip.regs[0][ICM42688_WHO_AM_I] = chip.who;
	chip.regs[1][ICM42688_GYRO_CONFIG_STATIC2] = 0xA0;
	chip.regs[1][ICM42688_GYRO_CONFIG_STATIC3] = 0x0D;
	chip.regs[1][ICM42688_GYRO_CONFIG_STATIC4] = 0xAA;
	chip.regs[1][ICM42688_GYRO_CONFIG_STATIC5] = 0x80;
	chip.regs[1][0x7A] = 0x83;			/* INTF_CONFIG4 */
	chip.regs[1][ICM42688_INTF_CONFIG6] = 0x5F;
	chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC2] = 0x30;
	chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC3] = 0x40;
	chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC4] = 0x62;
	chip.pin = false;
}

static void chip_power_on(void)
{
	memset(&chip, 0, sizeof(chip));
	chip.who = ICM42688_WHO_AM_I_VALUE;
	chip.fail_in = -1;
	chip.quiet_until_us = INT64_MIN / 2;
	chip.no_write_until_us = INT64_MIN / 2;
	chip_reset_registers();
}

static bool chip_fails(void)
{
	chip.transfers++;
	return chip.fail_in >= 0 && chip.fail_in-- == 0;
}

static bool chip_running(void)
{
	return chip.regs[0][ICM42688_PWR_MGMT0] & 0x0F;
}

/* The registers 12.9 lets change while a sensor runs. */
static bool chip_runtime_register(uint8_t reg)
{
	return chip.bank == 0 && (reg == ICM42688_GYRO_CONFIG0 ||
				  reg == ICM42688_ACCEL_CONFIG0 ||
				  reg == ICM42688_PWR_MGMT0);
}

static void chip_write(uint8_t reg, uint8_t v)
{
	if (now_us < chip.quiet_until_us || now_us < chip.no_write_until_us) {
		chip.violations++;
		return;
	}
	if (reg == ICM42688_REG_BANK_SEL) {
		assert(v <= 2 || v == 4);	/* the user banks, 13 */
		chip.bank = v;
		chip.bank_writes++;
		return;
	}
	if (chip_running() && !chip_runtime_register(reg))
		chip.violations++;
	if (chip.bank == 0 && reg == ICM42688_DEVICE_CONFIG &&
	    (v & ICM42688_SOFT_RESET)) {
		chip_reset_registers();
		chip.resets++;
		chip.quiet_until_us = now_us + 1000;
		return;
	}
	if (chip.bank == 0 && reg == ICM42688_PWR_MGMT0) {
		uint8_t old = chip.regs[0][reg];

		/* a sensor from off to on: 200 us of no writes */
		if (((v & 0x3) >= 2 && (old & 0x3) < 2) ||
		    ((v & 0xC) && !(old & 0xC)))
			chip.no_write_until_us = now_us + 200;
	}
	chip.regs[chip.bank][reg & 0x7F] = v;
}

static void chip_read(uint8_t reg, uint8_t *out, size_t len)
{
	bool data = false;

	if (now_us < chip.quiet_until_us) {
		chip.violations++;
		memset(out, 0xFF, len);
		return;
	}
	for (size_t i = 0; i < len; i++) {
		uint8_t r = (uint8_t)((reg + i) & 0x7F);

		if (r == ICM42688_REG_BANK_SEL) {
			out[i] = chip.bank;
			continue;
		}
		out[i] = chip.regs[chip.bank][r];
		if (chip.bank == 0 && r == ICM42688_INT_STATUS) {
			chip.regs[0][r] = 0;		/* R/C, 14.21 */
			chip.pin = false;
		}
		if (chip.bank == 0 && r >= 0x1D && r <= 0x2A)
			data = true;
	}
	/* UI_DRDY_INT_CLEAR 10 or 11: a sensor register read clears it */
	if (data && (chip.regs[0][ICM42688_INT_CONFIG0] & 0x20))
		chip.pin = false;
}

static bool chip_i2c_read(uint8_t reg, uint8_t *out, size_t len)
{
	if (chip_fails())
		return false;
	chip_read(reg, out, len);
	return true;
}

static bool chip_i2c_write(uint8_t reg, const uint8_t *data, size_t len)
{
	if (chip_fails())
		return false;
	for (size_t i = 0; i < len; i++)
		chip_write((uint8_t)(reg + i), data[i]);
	return true;
}

/* One SPI frame: tx[0] the address byte, read bit 7; no dummy byte. */
static bool chip_spi_frame(const uint8_t *tx, uint8_t *rx, size_t n)
{
	if (chip_fails())
		return false;
	if (tx[0] & ICM42688_SPI_READ) {
		assert(rx && n >= 2);
		rx[0] = 0xFF;
		chip_read(tx[0] & 0x7F, &rx[1], n - 1);
	} else {
		for (size_t i = 1; i < n; i++)
			chip_write((uint8_t)((tx[0] & 0x7F) + i - 1), tx[i]);
	}
	return true;
}

/* ------------------------------------------------------------ helpers ---- */

static int probe_on(struct imu_dev *imu, enum bus bus,
		    const struct icm42688_config *config);

static const struct icm42688_config base = {
	.accel_fs = ICM42688_ACCEL_FS_4G,
	.gyro_fs = ICM42688_GYRO_FS_500DPS,
	.odr = ICM42688_ODR_1KHZ,
	.accel_ui_bw = ICM42688_UI_BW_M_4,
	.gyro_ui_bw = ICM42688_UI_BW_M_4,
	.accel_ui_order = ICM42688_UI_ORDER_2,
	.gyro_ui_order = ICM42688_UI_ORDER_2,
};

static void put16(uint8_t reg, int16_t v)
{
	chip.regs[0][reg] = (uint8_t)((uint16_t)v >> 8);	/* big endian */
	chip.regs[0][reg + 1] = (uint8_t)((uint16_t)v & 0xFF);
}

static bool near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* ------------------------------------------------------------- cases ---- */

/* A plain config: what lands where, and nothing written out of turn. */
static void test_probe(enum bus bus)
{
	struct imu_dev imu = { 0 };

	chip_power_on();
	allocs = frees = 0;
	assert(probe_on(&imu, bus, &base) == 0);
	assert(imu.ops && strcmp(imu.name, "icm42688") == 0);
	assert(allocs == 1 && frees == 0);
	assert(chip.resets == 1 && chip.violations == 0);
	assert(chip.bank == 0);

	/* 12.3: I2C's settings on I2C, the reset values left on SPI */
	if (bus == BUS_I2C) {
		assert(chip.regs[0][ICM42688_DRIVE_CONFIG] == 0x09);
		assert(chip.regs[1][ICM42688_INTF_CONFIG6] == 0x5C);
	} else {
		assert(chip.regs[0][ICM42688_DRIVE_CONFIG] == 0x05);
		assert(chip.regs[1][ICM42688_INTF_CONFIG6] == 0x5F);
	}
	/* big endian, the rest of INTF_CONFIG0 as reset */
	assert(chip.regs[0][ICM42688_INTF_CONFIG0] == 0x30);
	/* 4 g and 500 dps (code 2 << 5) at 1 kHz (6) */
	assert(chip.regs[0][ICM42688_ACCEL_CONFIG0] == 0x46);
	assert(chip.regs[0][ICM42688_GYRO_CONFIG0] == 0x46);
	/* 2nd order and M/4 are the reset values: unchanged */
	assert(chip.regs[0][ICM42688_GYRO_CONFIG1] == 0x16);
	assert(chip.regs[0][ICM42688_ACCEL_CONFIG1] == 0x0D);
	assert(chip.regs[0][ICM42688_GYRO_ACCEL_CONFIG0] == 0x11);
	/* AAF left at reset; interrupt left at reset */
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC3] == 0x0D);
	assert(chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC2] == 0x30);
	assert(chip.regs[0][ICM42688_INT_SOURCE0] == 0x10);
	assert(chip.regs[0][ICM42688_INT_CONFIG1] == 0x10);
	/* both sensors on, low noise */
	assert(chip.regs[0][ICM42688_PWR_MGMT0] == 0x0F);

	assert(icm42688_accel_sensitivity(&imu) == 8192.0f);
	assert(icm42688_gyro_sensitivity(&imu) == 65.5f);

	icm42688_remove(&imu);
	assert(chip.regs[0][ICM42688_PWR_MGMT0] == 0);
	assert(!imu.ops && !imu.priv && frees == 1);
	icm42688_remove(&imu);
}

/* Anti-alias and UI filters: the right bank, bits and reserved bits. */
static void test_filters(enum bus bus)
{
	struct icm42688_config c = base;
	struct imu_dev imu = { 0 };

	/* Rows 1 and 63 of the table in 5.3, 3rd and 1st order, ODR/2 and
	 * low latency */
	c.gyro_aaf = 1;
	c.accel_aaf = 63;
	c.gyro_ui_order = ICM42688_UI_ORDER_3;
	c.accel_ui_order = ICM42688_UI_ORDER_1;
	c.gyro_ui_bw = ICM42688_UI_BW_ODR_2;
	c.accel_ui_bw = ICM42688_UI_BW_LOW_LATENCY_8ODR;
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.violations == 0 && chip.bank == 0);
	/* gyro: enabled, reserved 0xA0 kept; DELT 1, DELTSQR 1, BITSHIFT 15 */
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC2] == 0xA0);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC3] == 0x01);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC4] == 0x01);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC5] == 0xF0);
	/* accel: DELT 63 in bits 6:1, DELTSQR 3968 = 0xF80, BITSHIFT 3 */
	assert(chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC2] == 0x7E);
	assert(chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC3] == 0x80);
	assert(chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC4] == 0x3F);
	/* and nothing landed in bank 0 at those addresses */
	assert(chip.regs[0][0x03] == 0 && chip.regs[0][0x0C] == 0);
	/* GYRO_CONFIG1: order 2 in bits 3:2, the rest (0x12) kept */
	assert(chip.regs[0][ICM42688_GYRO_CONFIG1] == 0x1A);
	/* ACCEL_CONFIG1: order 0 in bits 4:3, the rest (0x05) kept */
	assert(chip.regs[0][ICM42688_ACCEL_CONFIG1] == 0x05);
	assert(chip.regs[0][ICM42688_GYRO_ACCEL_CONFIG0] == 0xF0);
	icm42688_remove(&imu);

	/* Row 13, the gyro's reset row, and row 24, the accel's, written out */
	c = base;
	c.gyro_aaf = 13;
	c.accel_aaf = 24;
	chip_power_on();
	chip.regs[1][ICM42688_GYRO_CONFIG_STATIC4] = 0;
	chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC3] = 0;
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC3] == 0x0D &&
	       chip.regs[1][ICM42688_GYRO_CONFIG_STATIC4] == 0xAA &&
	       chip.regs[1][ICM42688_GYRO_CONFIG_STATIC5] == 0x80);
	assert(chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC2] == 0x30 &&
	       chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC3] == 0x40 &&
	       chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC4] == 0x62);
	icm42688_remove(&imu);

	/* Row 40 on the gyro: DELTSQR 1600 = 0x640 needs its high bits */
	c = base;
	c.gyro_aaf = 40;
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC3] == 40 &&
	       chip.regs[1][ICM42688_GYRO_CONFIG_STATIC4] == 0x40 &&
	       chip.regs[1][ICM42688_GYRO_CONFIG_STATIC5] == 0x46);
	icm42688_remove(&imu);

	/* Bypassed: only the DIS bits change */
	c = base;
	c.gyro_aaf = ICM42688_AAF_OFF;
	c.accel_aaf = ICM42688_AAF_OFF;
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC2] == 0xA2);
	assert(chip.regs[1][ICM42688_GYRO_CONFIG_STATIC3] == 0x0D);
	assert(chip.regs[2][ICM42688_ACCEL_CONFIG_STATIC2] == 0x31);
	assert(chip.violations == 0 && chip.bank == 0);
	icm42688_remove(&imu);

	assert(icm42688_aaf_bandwidth_hz(1) == 42);
	assert(icm42688_aaf_bandwidth_hz(13) == 585);
	assert(icm42688_aaf_bandwidth_hz(63) == 3979);
	assert(icm42688_aaf_bandwidth_hz(0) == 0);
	assert(icm42688_aaf_bandwidth_hz(64) == 0);
}

/* The data-ready interrupt both ways, and the latched pin's release. */
static void test_interrupt(enum bus bus)
{
	struct icm42688_config c = base;
	struct imu_dev imu = { 0 };
	struct imu_sample s;
	bool ready = true;

	c.data_ready_int1 = (struct icm42688_int_config){
		.enable = true, .active_high = true, .latched = true,
	};
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.violations == 0);
	/* latched, push-pull, active high */
	assert(chip.regs[0][ICM42688_INT_CONFIG] == 0x07);
	/* clear on status read or sensor read */
	assert(chip.regs[0][ICM42688_INT_CONFIG0] == 0x30);
	/* 8 us pulse, no de-assert time, INT_ASYNC_RESET cleared (12.6) */
	assert(chip.regs[0][ICM42688_INT_CONFIG1] == 0x60);
	/* data ready alone on INT1 */
	assert(chip.regs[0][ICM42688_INT_SOURCE0] == 0x08);

	/* A latched pin is released by the sample read */
	chip.pin = true;
	chip.regs[0][ICM42688_INT_STATUS] = ICM42688_DATA_RDY_INT;
	assert(imu_read(&imu, &s) == 0 && !chip.pin);
	/* INT_STATUS clears on read */
	assert(icm42688_data_ready(&imu, &ready) == 0 && ready);
	assert(icm42688_data_ready(&imu, &ready) == 0 && !ready);
	assert(icm42688_data_ready(&imu, NULL) == -EINVAL);
	icm42688_remove(&imu);

	/* Open drain, active low, pulsed */
	c.data_ready_int1 = (struct icm42688_int_config){
		.enable = true, .open_drain = true,
	};
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.regs[0][ICM42688_INT_CONFIG] == 0x00);
	assert(chip.regs[0][ICM42688_INT_SOURCE0] == 0x08);
	icm42688_remove(&imu);
}

/* One big-endian burst, scaled; temperature by 14.6's formula. */
static void test_read(enum bus bus)
{
	struct icm42688_config c = base;
	struct imu_dev imu = { 0 };
	struct imu_sample s;
	int16_t a[3], g[3], t;

	c.accel_fs = ICM42688_ACCEL_FS_16G;
	c.gyro_fs = ICM42688_GYRO_FS_2000DPS;
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);

	put16(ICM42688_TEMP_DATA1, 1325);	/* 25 + 1325/132.48 degC */
	put16(0x1F, 2048);			/* +1 g at 16 g */
	put16(0x21, -1024);
	put16(0x23, 0x0102);
	put16(0x25, 164);			/* +10 dps at 2000 dps */
	put16(0x27, -328);
	put16(0x29, 0x7FFF);

	assert(icm42688_read_raw(&imu, a, g, &t) == 0);
	assert(t == 1325);
	assert(a[0] == 2048 && a[1] == -1024 && a[2] == 0x0102);
	assert(g[0] == 164 && g[1] == -328 && g[2] == 0x7FFF);
	assert(icm42688_read_raw(&imu, a, g, NULL) == 0);

	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[0], 9.80665f, 1e-4f));
	assert(near(s.accel[1], -4.903325f, 1e-4f));
	assert(near(s.gyro[0], 10.0f * 0.0174532925f, 1e-5f));
	assert(near(s.gyro[1], -20.0f * 0.0174532925f, 1e-5f));
	assert(near(s.temp_c, 25.0f + 1325.0f / 132.48f, 1e-4f));

	s.temp_c = 99.0f;
	chip.fail_in = 0;
	assert(imu_read(&imu, &s) != 0 && s.temp_c == 99.0f);
	icm42688_remove(&imu);
}

/* Changes after probe: only what 12.9 allows while running. */
static void test_runtime(enum bus bus)
{
	struct imu_dev imu = { 0 };
	uint8_t v = 0;

	chip_power_on();
	assert(probe_on(&imu, bus, &base) == 0);

	assert(icm42688_set_ranges(&imu, ICM42688_ACCEL_FS_2G,
				   ICM42688_GYRO_FS_15_625DPS) == 0);
	assert(chip.regs[0][ICM42688_ACCEL_CONFIG0] == 0x66);
	assert(chip.regs[0][ICM42688_GYRO_CONFIG0] == 0xE6);
	assert(icm42688_accel_sensitivity(&imu) == 16384.0f);
	assert(icm42688_gyro_sensitivity(&imu) == 2097.2f);
	assert(icm42688_set_odr(&imu, ICM42688_ODR_32KHZ) == 0);
	assert(chip.regs[0][ICM42688_ACCEL_CONFIG0] == 0x61);
	assert(chip.regs[0][ICM42688_GYRO_CONFIG0] == 0xE1);
	assert(icm42688_set_odr(&imu, ICM42688_ODR_500HZ) == 0);
	assert(chip.regs[0][ICM42688_GYRO_CONFIG0] == 0xEF);
	assert(chip.violations == 0);

	assert(icm42688_set_ranges(&imu, 4, ICM42688_GYRO_FS_2000DPS) ==
	       -EINVAL);
	assert(icm42688_set_ranges(&imu, ICM42688_ACCEL_FS_2G, 8) == -EINVAL);
	assert(icm42688_set_odr(&imu, 0) == -EINVAL);
	assert(icm42688_set_odr(&imu, 0xC) == -EINVAL);	/* LP only */
	assert(chip.regs[0][ICM42688_ACCEL_CONFIG0] == 0x6F);

	/* Power: off, then on, then a write at once -- the driver waited */
	assert(icm42688_set_power(&imu, false, false) == 0);
	assert(chip.regs[0][ICM42688_PWR_MGMT0] == 0x00);
	assert(icm42688_set_power(&imu, true, false) == 0);
	assert(chip.regs[0][ICM42688_PWR_MGMT0] == 0x03);
	assert(icm42688_set_power(&imu, true, true) == 0);
	assert(chip.regs[0][ICM42688_PWR_MGMT0] == 0x0F);
	assert(icm42688_set_odr(&imu, ICM42688_ODR_1KHZ) == 0);
	assert(chip.violations == 0);

	/* The escape hatch, by bank, back in bank 0 after */
	assert(icm42688_reg_read(&imu, 1, ICM42688_GYRO_CONFIG_STATIC3, &v,
				 1) == 0 && v == 0x0D && chip.bank == 0);
	assert(icm42688_reg_read(&imu, 0, ICM42688_WHO_AM_I, &v, 1) == 0 &&
	       v == 0x47);
	assert(icm42688_reg_read(&imu, 5, 0, &v, 1) == -EINVAL);
	assert(icm42688_reg_read(&imu, 3, 0, &v, 1) == -EINVAL);
	assert(icm42688_reg_write(&imu, 3, 0, 0) == -EINVAL);
	assert(icm42688_reg_read(&imu, 0, 0, &v, 15) == -EINVAL);
	assert(icm42688_reg_write(&imu, 5, 0, 0) == -EINVAL);
	icm42688_remove(&imu);

	/* Sensors off: a bank-4 write lands in bank 4, then bank 0 again */
	chip_power_on();
	assert(probe_on(&imu, bus, &base) == 0);
	assert(icm42688_set_power(&imu, false, false) == 0);
	assert(icm42688_reg_write(&imu, 4, 0x40, 0x5A) == 0);
	assert(chip.regs[4][0x40] == 0x5A && chip.bank == 0);
	assert(chip.violations == 0);
	icm42688_remove(&imu);
}

/* Refused before anything is allocated or sent. */
static void test_refused(enum bus bus)
{
	static const struct imu_ops other_ops = { 0 };
	struct icm42688_config c;
	struct imu_dev imu = { 0 };
	struct imu_dev other = { .ops = &other_ops };
	int16_t a[3], g[3];
	bool b;

	chip_power_on();
	allocs = 0;
#define REFUSED(field, value) do { \
		c = base; \
		c.field = value; \
		assert(probe_on(&imu, bus, &c) == -EINVAL); \
	} while (0)
	REFUSED(accel_fs, 4);			/* reserved */
	REFUSED(gyro_fs, 8);
	REFUSED(odr, 0);			/* reserved */
	REFUSED(odr, 0xC);			/* 6.25 Hz: LP only */
	REFUSED(odr, 0xE);
	REFUSED(accel_ui_bw, 8);		/* reserved 8..13 */
	REFUSED(gyro_ui_bw, 13);
	REFUSED(gyro_ui_bw, 16);
	REFUSED(accel_ui_order, 3);		/* reserved */
	REFUSED(gyro_ui_order, 3);
	REFUSED(accel_aaf, 64);
	REFUSED(gyro_aaf, 0xFE);
#undef REFUSED
	assert(probe_on(&imu, bus, NULL) == -EINVAL);
	assert(allocs == 0 && chip.transfers == 0 && !imu.ops);

	/* Already bound */
	assert(probe_on(&imu, bus, &base) == 0);
	allocs = 0;
	assert(probe_on(&imu, bus, &base) == -EBUSY && allocs == 0);
	icm42688_remove(&imu);

	/* Another driver's imu_dev */
	assert(icm42688_read_raw(&other, a, g, NULL) == -EINVAL);
	assert(icm42688_data_ready(&other, &b) == -EINVAL);
	assert(icm42688_set_ranges(&other, 0, 0) == -EINVAL);
	assert(icm42688_set_odr(&other, ICM42688_ODR_1KHZ) == -EINVAL);
	assert(icm42688_set_power(&other, true, true) == -EINVAL);
	assert(icm42688_reg_read(&other, 0, 0, &b, 1) == -EINVAL);
	assert(icm42688_reg_write(&other, 0, 0, 0) == -EINVAL);
	assert(icm42688_accel_sensitivity(&other) == 0.0f);
	assert(icm42688_gyro_sensitivity(NULL) == 0.0f);
	icm42688_remove(&other);
	assert(other.ops == &other_ops);
}

/*
 * Fail transfer n of a probe, then probe again with the chip still powered,
 * in whatever bank the failure left it: the second probe must work.
 */
static void probe_fails_at(enum bus bus, const struct icm42688_config *c,
			   int n)
{
	struct imu_dev imu = { 0 };

	chip_power_on();
	chip.fail_in = n;
	allocs = frees = 0;
	assert(probe_on(&imu, bus, c) < 0);
	assert(!imu.ops && !imu.priv && allocs == frees);
	assert(probe_on(&imu, bus, c) == 0);
	assert(chip.bank == 0 && chip.violations == 0);
	icm42688_remove(&imu);
}

static void test_failures(enum bus bus)
{
	struct icm42688_config c = base;
	struct imu_dev imu = { 0 };
	int t;

	/* Someone else answers: no reset and no config written to it, only
	 * the REG_BANK_SEL = 0 at 0x76 that comes before WHO_AM_I */
	chip_power_on();
	chip.who = 0x98;
	chip_reset_registers();
	allocs = frees = 0;
	assert(probe_on(&imu, bus, &base) == -ENODEV);
	assert(!imu.ops && allocs == frees && chip.resets == 0);

	/* A bus error at each step, with every optional part in */
	c.gyro_aaf = 20;
	c.accel_aaf = ICM42688_AAF_OFF;
	c.data_ready_int1.enable = true;
	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	t = chip.transfers;
	icm42688_remove(&imu);
	for (int i = 0; i < t; i++)
		probe_fails_at(bus, &c, i);

	/* The MCU reset in the middle of a banked access: chip left in bank 2 */
	chip_power_on();
	chip.bank = 2;
	assert(probe_on(&imu, bus, &base) == 0 && chip.bank == 0);

	/* A bank select that fails: a best-effort bank 0 follows it */
	{
		uint8_t v;
		int before = chip.bank_writes;

		chip.fail_in = 0;
		assert(icm42688_reg_read(&imu, 1, 0x0C, &v, 1) < 0);
		assert(chip.bank_writes == before + 1 && chip.bank == 0);
		before = chip.bank_writes;
		chip.fail_in = 0;
		assert(icm42688_reg_write(&imu, 2, 0x10, 1) < 0);
		assert(chip.bank_writes == before + 1 && chip.bank == 0);
	}
	icm42688_remove(&imu);
}

static void run_icm42688_cases(void)
{
	static const enum bus buses[] = { BUS_I2C, BUS_SPI };

	for (size_t i = 0; i < 2; i++) {
		printf("-- on %s\n", buses[i] == BUS_I2C ? "I2C" : "SPI");
		test_probe(buses[i]);
		test_filters(buses[i]);
		test_interrupt(buses[i]);
		test_read(buses[i]);
		test_runtime(buses[i]);
		test_refused(buses[i]);
		test_failures(buses[i]);
	}
}
