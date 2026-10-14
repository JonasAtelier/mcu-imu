/*
 * The BMI270's cases, shared by esp/test_bmi270.c and stm/test_bmi270.c,
 * each run on I2C and on SPI.
 *
 * A register-level fake BMI270 that keeps the rules of the datasheet
 * (BST-BMI270-DS000-08 rev. 1.6) that a driver can get wrong without
 * noticing on the bench until it fails:
 *
 *   - the interface is I2C after power-up and every soft reset, and turns
 *     SPI 200 us after a rising edge of CSB (6.3); an SPI frame before that
 *     is not understood;
 *   - an SPI read answers one dummy byte before the data (6.4);
 *   - after a write, the next one must wait 450 us while advanced power
 *     save is on, 2 us once it is off (4.5, Table 19); a write too soon is
 *     lost;
 *   - the configuration goes to INIT_DATA only with INIT_CTRL at 0 and
 *     advanced power save off, at the word offset in INIT_ADDR (4.4,
 *     5.2.65); INIT_CTRL = 1 more than once after a reset fails the load;
 *   - INTERNAL_STATUS reads init_ok some time after INIT_CTRL = 1 (4.4),
 *     and only if what was uploaded is Bosch's file byte for byte;
 *   - reading INT_STATUS_1 clears it (5.2.29).
 *
 * Each platform file includes this after providing:
 *
 *   enum bus { BUS_I2C, BUS_SPI }
 *   probe_on(imu, bus, config)   that bus's probe with a valid bus config
 *   allocs, frees                calloc() and free() calls by the driver
 *   now_us                       the fake clock its delay functions move
 * and hands its bus calls to chip_i2c_read(), chip_i2c_write(),
 * chip_spi_frame() and chip_cs_rise().
 */

#define INIT_DELAY_US	15000	/* when the fake reports the load done */

static struct {
	uint8_t regs[128];
	uint8_t image[BMI270_CONFIG_SIZE];
	bool spi;			/* interface switched to SPI */
	int64_t spi_from_us;		/* SPI from then; -1: no edge yet */
	int64_t write_ok_us;		/* the next write is heard from then */
	int64_t init_at_us;		/* INIT_CTRL = 1; -1: not yet */
	bool prepared;			/* INIT_CTRL = 0 since reset */
	int init_done_writes;		/* INIT_CTRL = 1 since reset */
	uint16_t init_word;		/* INIT_ADDR, in words */
	uint8_t chip_id;
	uint8_t cas;			/* GYR_CAS once loaded */
	uint8_t message;		/* INTERNAL_STATUS once loaded */
	bool never_ready;
	bool corrupt_upload;		/* flip one uploaded byte */
	bool resetting;			/* the CSB edge ending a soft-reset
					 * frame comes before the reset is
					 * done, so it selects nothing */
	int fail_in;			/* fail the Nth transfer from now */
	int resets, lost_writes, chunks, longest_chunk, frames_in_i2c;
	int transfers;
} chip;

static int64_t now_us;

/* Power-on values (5.2): what a reset leaves. */
static void chip_reset_registers(void)
{
	memset(chip.regs, 0, sizeof(chip.regs));
	memset(chip.image, 0, sizeof(chip.image));
	chip.regs[BMI270_CHIP_ID] = chip.chip_id;
	chip.regs[BMI270_STATUS] = 0x10;
	chip.regs[0x1B] = 0x01;			/* EVENT.por_detected */
	chip.regs[BMI270_TEMPERATURE_0 + 1] = 0x80;
	chip.regs[BMI270_ACC_CONF] = 0xA8;
	chip.regs[BMI270_ACC_RANGE] = 0x02;
	chip.regs[BMI270_GYR_CONF] = 0xA9;
	chip.regs[BMI270_PWR_CONF] = 0x03;
	chip.spi = false;
	chip.spi_from_us = -1;
	chip.init_at_us = -1;
	chip.prepared = false;
	chip.init_done_writes = 0;
	chip.init_word = 0;
}

static void chip_power_on(void)
{
	memset(&chip, 0, sizeof(chip));
	chip.chip_id = BMI270_CHIP_ID_VALUE;
	chip.cas = 0x7E;			/* factor_zx = -2 */
	chip.message = BMI270_MESSAGE_INIT_OK;
	chip.fail_in = -1;
	chip.write_ok_us = INT64_MIN / 2;
	chip_reset_registers();
}

/* Counts down the transfers to a failure; true when this one fails. */
static bool chip_fails(void)
{
	chip.transfers++;
	return chip.fail_in >= 0 && chip.fail_in-- == 0;
}

static bool chip_aps(void)
{
	return chip.regs[BMI270_PWR_CONF] & BMI270_ADV_POWER_SAVE;
}

static void chip_write_one(uint8_t reg, uint8_t v)
{
	switch (reg) {
	case BMI270_CMD:
		if (v == BMI270_CMD_SOFTRESET) {
			chip_reset_registers();
			chip.resets++;
			chip.resetting = true;
		}
		return;
	case BMI270_INIT_CTRL:
		if (v == BMI270_INIT_CTRL_PREPARE) {
			chip.prepared = true;
		} else if (v == BMI270_INIT_CTRL_DONE) {
			bool good = chip.prepared && !chip_aps() &&
				    !memcmp(chip.image, bmi270_config_data,
					    sizeof(chip.image));

			chip.init_done_writes++;
			chip.init_at_us = now_us;
			/* init_err (5.2.33) for a bad file or a second go */
			if (chip.message == BMI270_MESSAGE_INIT_OK &&
			    (!good || chip.init_done_writes > 1))
				chip.message = 0x02;
		}
		break;
	case BMI270_INIT_ADDR_0:
		v &= BMI270_INIT_ADDR_0_MASK;
		chip.init_word = (uint16_t)((chip.init_word & 0xFFF0) | v);
		break;
	case BMI270_INIT_ADDR_0 + 1:
		chip.init_word = (uint16_t)((chip.init_word & 0x000F) |
					    (v << 4));
		break;
	default:
		break;
	}
	chip.regs[reg & 0x7F] = v;
}

/*
 * One write transaction: reg, then len bytes. The idle time it needs after
 * it is set by the power mode it was made in, so the write that turns
 * advanced power save off still needs the 450 us.
 */
static void chip_write(uint8_t reg, const uint8_t *data, size_t len)
{
	if (now_us < chip.write_ok_us) {
		chip.lost_writes++;
		return;
	}
	chip.write_ok_us = now_us + (chip_aps() ? 450 : 2);

	if (reg == BMI270_INIT_DATA) {
		size_t at = (size_t)chip.init_word * 2;

		if (!chip.prepared || chip_aps() ||
		    at + len > sizeof(chip.image)) {
			chip.lost_writes++;
			return;
		}
		memcpy(&chip.image[at], data, len);
		if (chip.corrupt_upload && chip.chunks == 100)
			chip.image[at] ^= 0xFF;
		chip.chunks++;
		if ((int)len > chip.longest_chunk)
			chip.longest_chunk = (int)len;
		return;
	}
	for (size_t i = 0; i < len; i++)
		chip_write_one((uint8_t)(reg + i), data[i]);
}

/* One read transaction: len bytes from reg on. */
static void chip_read(uint8_t reg, uint8_t *out, size_t len)
{
	if (chip.init_at_us >= 0 && !chip.never_ready &&
	    now_us - chip.init_at_us >= INIT_DELAY_US) {
		chip.regs[BMI270_INTERNAL_STATUS] = chip.message;
		if (chip.message == BMI270_MESSAGE_INIT_OK &&
		    chip.regs[BMI270_FEAT_PAGE] == 0)
			chip.regs[BMI270_GYR_CAS] = chip.cas;
	}
	for (size_t i = 0; i < len; i++) {
		uint8_t r = (uint8_t)((reg + i) & 0x7F);

		out[i] = chip.regs[r];
		if (r == 0x1D)			/* INT_STATUS_1 */
			chip.regs[r] = 0;
	}
}

/* An I2C transaction; false when the bus should report a failure. */
static bool chip_i2c_read(uint8_t reg, uint8_t *out, size_t len)
{
	if (chip_fails())
		return false;
	assert(!chip.spi && "I2C after the interface turned SPI");
	chip_read(reg, out, len);
	return true;
}

static bool chip_i2c_write(uint8_t reg, const uint8_t *data, size_t len)
{
	if (chip_fails())
		return false;
	assert(!chip.spi && "I2C after the interface turned SPI");
	chip_write(reg, data, len);
	return true;
}

/*
 * One SPI frame, CSB low to high: tx and rx are n bytes each, tx[0] the
 * address byte. The rising edge at its end is chip_cs_rise()'s.
 */
static bool chip_spi_frame(const uint8_t *tx, uint8_t *rx, size_t n)
{
	if (chip_fails())
		return false;
	if (chip.spi_from_us >= 0 && now_us >= chip.spi_from_us)
		chip.spi = true;
	if (!chip.spi) {
		/* Still I2C: nothing here is understood. */
		chip.frames_in_i2c++;
		if (rx)
			memset(rx, 0xFF, n);
		return true;
	}
	if (tx[0] & BMI270_SPI_READ) {
		assert(rx && n >= 3);
		rx[0] = 0xFF;			/* under the address */
		rx[1] = 0xA5;			/* the dummy byte */
		chip_read(tx[0] & 0x7F, &rx[2], n - 2);
	} else {
		chip_write(tx[0], &tx[1], n - 1);
	}
	return true;
}

static void chip_cs_rise(void)
{
	if (chip.resetting) {
		chip.resetting = false;
		return;
	}
	if (!chip.spi && chip.spi_from_us < 0)
		chip.spi_from_us = now_us + 200;
}

/* ------------------------------------------------------------ helpers ---- */

static int probe_on(struct imu_dev *imu, enum bus bus,
		    const struct bmi270_config *config);

static const char *bus_name(enum bus bus)
{
	return bus == BUS_I2C ? "I2C" : "SPI";
}

static void put16(uint8_t reg, int16_t v)
{
	chip.regs[reg] = (uint8_t)((uint16_t)v & 0xFF);
	chip.regs[reg + 1] = (uint8_t)((uint16_t)v >> 8);
}

static bool near(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* ------------------------------------------------------------- cases ---- */

/* Defaults: the whole of section 4.4, then the reset values written back. */
static void test_probe(enum bus bus)
{
	struct imu_dev imu = { 0 };
	int64_t start;

	chip_power_on();
	allocs = frees = 0;
	start = now_us;
	assert(probe_on(&imu, bus, NULL) == 0);
	assert(imu.ops && imu.priv && strcmp(imu.name, "bmi270") == 0);
	assert(allocs == 1 && frees == 0);

	/* One soft reset, then the file, intact, loaded once */
	assert(chip.resets == 1);
	assert(memcmp(chip.image, bmi270_config_data,
		      sizeof(chip.image)) == 0);
	assert(chip.init_done_writes == 1);
	assert(chip.regs[BMI270_INTERNAL_STATUS] == BMI270_MESSAGE_INIT_OK);
	/* 8192 bytes in 30-byte pieces: 273 of them and one of 2 */
	assert(chip.chunks == 274 && chip.longest_chunk == 30);
	/* Every write landed: idle times kept, nothing sent too soon */
	assert(chip.lost_writes == 0);
	assert(now_us - start >= INIT_DELAY_US);

	/* Normal mode, the chip's reset values (section 3, step 3) */
	assert(chip.regs[BMI270_PWR_CTRL] == 0x0E);
	assert(chip.regs[BMI270_ACC_CONF] == 0xA8);
	assert(chip.regs[BMI270_ACC_RANGE] == 0x02);
	assert(chip.regs[BMI270_GYR_CONF] == 0xA9);
	assert(chip.regs[BMI270_GYR_RANGE] == 0x00);
	assert(chip.regs[BMI270_PWR_CONF] == 0x02);
	/* No interrupt asked for: the pin stays off */
	assert(chip.regs[BMI270_INT1_IO_CTRL] == 0);
	assert(chip.regs[BMI270_INT_MAP_DATA] == 0);

	/* SPI: one throw-away frame at power-up, one after the reset */
	if (bus == BUS_SPI)
		assert(chip.frames_in_i2c == 2 && chip.spi);
	else
		assert(chip.frames_in_i2c == 0 && !chip.spi);

	assert(bmi270_accel_sensitivity(&imu) == 4096.0f);
	assert(bmi270_gyro_sensitivity(&imu) == 16.384f);

	bmi270_remove(&imu);
	assert(chip.regs[BMI270_PWR_CTRL] == 0);
	assert(!imu.ops && !imu.priv && frees == 1);
	bmi270_remove(&imu);	/* twice is harmless */

	/* Probed again: a fresh reset, so INIT_CTRL = 1 once per reset */
	assert(probe_on(&imu, bus, NULL) == 0);
	assert(chip.resets == 2 && chip.init_done_writes == 1);
	bmi270_remove(&imu);
}

/* Every register a full config touches, and the changes after probe. */
static void test_config(enum bus bus)
{
	struct bmi270_config c = {
		.accel_range = BMI270_ACCEL_RANGE_16G,
		.accel_odr = BMI270_ACCEL_ODR_1600_HZ,
		.accel_filter = BMI270_ACCEL_FILTER_OSR2,
		.gyro_range = BMI270_GYRO_RANGE_125_DPS,
		.gyro_odr = BMI270_GYRO_ODR_3200_HZ,
		.gyro_filter = BMI270_GYRO_FILTER_OSR4,
		.gyro_low_noise = true,
		.data_ready_int1 = {
			.enable = true, .active_high = true,
			.open_drain = true, .latched = true,
		},
	};
	struct imu_dev imu = { 0 };
	uint8_t flags = 0xFF;

	chip_power_on();
	assert(probe_on(&imu, bus, &c) == 0);
	/* filter_perf | bwp 1 << 4 | odr 0x0C */
	assert(chip.regs[BMI270_ACC_CONF] == 0x9C);
	assert(chip.regs[BMI270_ACC_RANGE] == 0x03);
	/* filter_perf | noise_perf | bwp 0 | odr 0x0D */
	assert(chip.regs[BMI270_GYR_CONF] == 0xCD);
	assert(chip.regs[BMI270_GYR_RANGE] == 0x04);
	/* output_en | lvl | od; latched; drdy on INT1 */
	assert(chip.regs[BMI270_INT1_IO_CTRL] == 0x0E);
	assert(chip.regs[BMI270_INT_LATCH] == 0x01);
	assert(chip.regs[BMI270_INT_MAP_DATA] == 0x04);
	assert(bmi270_accel_sensitivity(&imu) == 2048.0f);
	assert(near(bmi270_gyro_sensitivity(&imu), 262.144f, 1e-3f));

	/* Push-pull, active low, not latched */
	bmi270_remove(&imu);
	c.data_ready_int1.active_high = false;
	c.data_ready_int1.open_drain = false;
	c.data_ready_int1.latched = false;
	c.gyro_low_noise = false;
	assert(probe_on(&imu, bus, &c) == 0);
	assert(chip.regs[BMI270_INT1_IO_CTRL] == 0x08);
	assert(chip.regs[BMI270_INT_LATCH] == 0x00);
	assert(chip.regs[BMI270_GYR_CONF] == 0x8D);

	/* Ranges and rates after probe keep the filter bits */
	assert(bmi270_set_ranges(&imu, BMI270_ACCEL_RANGE_2G,
				 BMI270_GYRO_RANGE_1000_DPS) == 0);
	assert(chip.regs[BMI270_ACC_RANGE] == 0 &&
	       chip.regs[BMI270_GYR_RANGE] == 1);
	assert(bmi270_accel_sensitivity(&imu) == 16384.0f);
	assert(near(bmi270_gyro_sensitivity(&imu), 32.768f, 1e-4f));
	assert(bmi270_set_odr(&imu, BMI270_ACCEL_ODR_12_5_HZ,
			      BMI270_GYRO_ODR_25_HZ) == 0);
	assert(chip.regs[BMI270_ACC_CONF] == 0x95);
	assert(chip.regs[BMI270_GYR_CONF] == 0x86);
	assert(bmi270_set_ranges(&imu, 4, BMI270_GYRO_RANGE_125_DPS) ==
	       -EINVAL);
	assert(bmi270_set_ranges(&imu, BMI270_ACCEL_RANGE_2G, 5) == -EINVAL);
	assert(bmi270_set_odr(&imu, 0x04, BMI270_GYRO_ODR_25_HZ) == -EINVAL);
	assert(bmi270_set_odr(&imu, BMI270_ACCEL_ODR_12_5_HZ, 0x0E) ==
	       -EINVAL);
	assert(chip.regs[BMI270_ACC_RANGE] == 0 &&
	       chip.regs[BMI270_ACC_CONF] == 0x95);

	/* STATUS polling */
	chip.regs[BMI270_STATUS] = 0x10 | BMI270_STATUS_DRDY_ACC;
	assert(bmi270_data_ready(&imu, &flags) == 0 &&
	       flags == BMI270_DATA_READY_ACCEL);
	chip.regs[BMI270_STATUS] = 0x10 | BMI270_STATUS_DRDY_GYR |
				   BMI270_STATUS_DRDY_ACC;
	assert(bmi270_data_ready(&imu, &flags) == 0 &&
	       flags == (BMI270_DATA_READY_ACCEL | BMI270_DATA_READY_GYRO));
	chip.regs[BMI270_STATUS] = 0x10;
	assert(bmi270_data_ready(&imu, &flags) == 0 && flags == 0);
	assert(bmi270_data_ready(&imu, NULL) == -EINVAL);
	bmi270_remove(&imu);
}

/* One burst, decoded and scaled, the cross-axis correction applied. */
static void test_read(enum bus bus)
{
	struct imu_dev imu = { 0 };
	struct imu_sample s;
	int16_t a[3], g[3], t;

	chip_power_on();
	assert(probe_on(&imu, bus, NULL) == 0);	/* 8 g, 2000 dps */

	put16(BMI270_DATA_8, 4096);		/* +1 g */
	put16(BMI270_DATA_8 + 2, -2048);	/* -0.5 g */
	put16(BMI270_DATA_8 + 4, 0);
	put16(BMI270_DATA_8 + 6, 1000);		/* gyro X */
	put16(BMI270_DATA_8 + 8, -1638);	/* about -100 dps */
	put16(BMI270_DATA_8 + 10, 2048);	/* gyro Z */
	put16(BMI270_TEMPERATURE_0, 1024);	/* 23 + 2 degC */
	chip.regs[0x1D] = 0xC0;			/* both drdy flags */

	assert(bmi270_read_raw(&imu, a, g, &t) == 0);
	assert(a[0] == 4096 && a[1] == -2048 && a[2] == 0);
	/* X - (-2) * 2048 / 512 = 1000 + 8 */
	assert(g[0] == 1008 && g[1] == -1638 && g[2] == 2048);
	assert(t == 1024);
	/* The burst ran through INT_STATUS_1 and cleared it */
	assert(chip.regs[0x1D] == 0);
	assert(bmi270_read_raw(&imu, a, g, NULL) == 0);

	assert(imu_read(&imu, &s) == 0);
	assert(near(s.accel[0], 9.80665f, 1e-4f));
	assert(near(s.accel[1], -4.903325f, 1e-4f));
	assert(near(s.gyro[0], 1008.0f / 16.384f * 0.0174532925f, 1e-5f));
	assert(near(s.gyro[1], -1638.0f / 16.384f * 0.0174532925f, 1e-5f));
	assert(near(s.temp_c, 25.0f, 1e-4f));

	/* An invalid temperature is NaN; the rest of the sample stands */
	put16(BMI270_TEMPERATURE_0, INT16_MIN);
	assert(imu_read(&imu, &s) == 0 && isnan(s.temp_c));
	assert(near(s.accel[0], 9.80665f, 1e-4f));

	/* A failed read leaves the sample alone */
	s.temp_c = 99.0f;
	chip.fail_in = 0;
	assert(imu_read(&imu, &s) != 0 && s.temp_c == 99.0f);
	bmi270_remove(&imu);
}

/* Refused before anything is allocated or sent. */
static void test_refused(enum bus bus)
{
	static const struct bmi270_config good = {
		.accel_range = BMI270_ACCEL_RANGE_4G,
		.accel_odr = BMI270_ACCEL_ODR_100_HZ,
		.accel_filter = BMI270_ACCEL_FILTER_CIC,
		.gyro_range = BMI270_GYRO_RANGE_250_DPS,
		.gyro_odr = BMI270_GYRO_ODR_400_HZ,
		.gyro_filter = BMI270_GYRO_FILTER_NORMAL,
	};
	struct bmi270_config c;
	static const struct imu_ops other_ops = { 0 };
	struct imu_dev imu = { 0 };
	struct imu_dev bound = { 0 };

	chip_power_on();
	allocs = 0;
#define REFUSED(field, value) do { \
		c = good; \
		c.field = value; \
		assert(probe_on(&imu, bus, &c) == -EINVAL); \
	} while (0)
	REFUSED(accel_range, 4);
	REFUSED(accel_odr, 0x04);		/* 6.25 Hz: low power only */
	REFUSED(accel_odr, 0x0D);		/* reserved */
	REFUSED(accel_filter, 4);		/* reserved with filter_perf */
	REFUSED(gyro_range, 5);
	REFUSED(gyro_odr, 0x05);		/* reserved */
	REFUSED(gyro_odr, 0x0E);		/* reserved */
	REFUSED(gyro_filter, 3);		/* reserved */
#undef REFUSED
	assert(allocs == 0 && chip.transfers == 0);
	assert(!imu.ops && !imu.priv);

	/* The good one goes through */
	assert(probe_on(&imu, bus, &good) == 0);
	assert(chip.regs[BMI270_ACC_CONF] == 0xB8);

	/* Already bound: refused, the first binding untouched */
	allocs = 0;
	assert(probe_on(&imu, bus, NULL) == -EBUSY);
	assert(allocs == 0 && imu.ops);
	bmi270_remove(&imu);

	assert(probe_on(NULL, bus, NULL) == -EINVAL);

	/* Another driver's imu_dev: every BMI270 call refuses it */
	bound.ops = &other_ops;
	{
		int16_t a[3], g[3];
		uint8_t b;

		assert(bmi270_read_raw(&bound, a, g, NULL) == -EINVAL);
		assert(bmi270_data_ready(&bound, &b) == -EINVAL);
		assert(bmi270_set_ranges(&bound, 0, 0) == -EINVAL);
		assert(bmi270_set_odr(&bound, 0x08, 0x08) == -EINVAL);
		assert(bmi270_reg_read(&bound, 0, &b, 1) == -EINVAL);
		assert(bmi270_reg_write(&bound, 0, 0) == -EINVAL);
		assert(bmi270_accel_sensitivity(&bound) == 0.0f);
		assert(bmi270_gyro_sensitivity(NULL) == 0.0f);
		bmi270_remove(&bound);
		assert(bound.ops);
	}
}

/* A probe whose transfer number n fails: an error, nothing left behind. */
static void probe_fails_at(enum bus bus, int n)
{
	struct imu_dev imu = { 0 };

	chip_power_on();
	chip.fail_in = n;
	allocs = frees = 0;
	assert(probe_on(&imu, bus, NULL) < 0);
	assert(!imu.ops && !imu.priv && allocs == frees);
}

/* Each way the bring-up can fail: its errno, and nothing left behind. */
static void test_failures(enum bus bus)
{
	struct imu_dev imu = { 0 };
	int64_t start;
	int t;

#define FAILS(expect) do { \
		allocs = frees = 0; \
		assert(probe_on(&imu, bus, NULL) == (expect)); \
		assert(!imu.ops && !imu.priv); \
		assert(allocs == frees); \
	} while (0)

	/* Someone else answers */
	chip_power_on();
	chip.chip_id = 0x26;
	chip_reset_registers();
	FAILS(-ENODEV);
	assert(chip.resets == 0);	/* nothing written to a stranger */

	/* The load reports an error code */
	chip_power_on();
	chip.message = 0x02;		/* init_err */
	FAILS(-EPROTO);

	/* The load never reports: give up after 20 ms, not before */
	chip_power_on();
	chip.never_ready = true;
	start = now_us;
	FAILS(-ETIMEDOUT);
	assert(now_us - chip.init_at_us >= 20000);
	assert(now_us - start >= 20000);

	/* An uploaded byte that arrives wrong: the chip reports init_err */
	chip_power_on();
	chip.corrupt_upload = true;
	FAILS(-EPROTO);

	/*
	 * A bus error at each step of the way -- every 37th transfer of a
	 * good probe, and its last -- ends the probe with an error, and
	 * leaves nothing bound or allocated.
	 */
	chip_power_on();
	assert(probe_on(&imu, bus, NULL) == 0);
	t = chip.transfers;
	bmi270_remove(&imu);
	for (int i = 0; i < t; i += 37)
		probe_fails_at(bus, i);
	probe_fails_at(bus, t - 1);
#undef FAILS
}

static void run_bmi270_cases(void)
{
	static const enum bus buses[] = { BUS_I2C, BUS_SPI };

	for (size_t i = 0; i < 2; i++) {
		printf("-- on %s\n", bus_name(buses[i]));
		test_probe(buses[i]);
		test_config(buses[i]);
		test_read(buses[i]);
		test_refused(buses[i]);
		test_failures(buses[i]);
	}
}
