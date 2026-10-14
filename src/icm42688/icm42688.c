#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "imu_priv.h"

#if CONFIG_IMU_ICM42688

#include "icm42688.h"
#include "icm42688_math.h"
#include "icm42688_priv.h"
#include "icm42688_regs.h"
#include "imu_log.h"

#define ICM42688_TIMEOUT_MS	100
#ifdef ESP_PLATFORM
#define ICM42688_I2C_HZ		400000		/* default */
#define ICM42688_I2C_MAX_HZ	1000000		/* 9.3 */
#define ICM42688_SPI_HZ		8000000		/* default; wiring often
						 * takes less than the part */
#define ICM42688_SPI_MAX_HZ	24000000	/* 9.6 */
#endif

/* The user banks: 0, 1, 2 and 4 (chapter 13). */
static bool icm42688_bank_valid(uint8_t bank)
{
	return bank <= 2 || bank == 4;
}

static const struct imu_ops icm42688_ops;

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct icm42688 *to_icm42688(struct imu_dev *imu)
{
	if (!imu || imu->ops != &icm42688_ops)
		return NULL;
	return imu->priv;
}

/* ---------------------------------------------------------------- bus ---- */

/*
 * len bytes from consecutive registers of the bank selected. On SPI bit 7
 * of the address byte reads and the data follows the address with no dummy
 * byte (9.6, item 5); on I2C a write of the address, then a repeated start
 * (9.4).
 */
static int icm42688_read(struct icm42688 *d, uint8_t reg, uint8_t *buf,
			 size_t len)
{
	if (len == 0 || len > ICM42688_BURST_LEN)
		return -EINVAL;
	if (d->bus.ops->kind == IMU_BUS_SPI)
		reg |= ICM42688_SPI_READ;
	return imu_reg_read(&d->bus, reg, buf, len);
}

static int icm42688_write(struct icm42688 *d, uint8_t reg, uint8_t value)
{
	return imu_reg_write(&d->bus, (uint8_t)(reg & ~ICM42688_SPI_READ),
			     value);
}

/* Only the bits in mask change; the reserved ones keep what they read. */
static int icm42688_update(struct icm42688 *d, uint8_t reg, uint8_t mask,
			   uint8_t value)
{
	uint8_t old;
	int ret;

	ret = icm42688_read(d, reg, &old, 1);
	if (ret)
		return ret;
	return icm42688_write(d, reg,
			      (uint8_t)((old & (uint8_t)~mask) |
					(value & mask)));
}

/*
 * Every register outside bank 0 is reached by selecting its bank in
 * REG_BANK_SEL, which sits at 0x76 in every bank (14.59), and the driver
 * goes back to bank 0 at once, failure or not: everything else it does is
 * in bank 0.
 */
static int icm42688_select_bank(struct icm42688 *d, uint8_t bank)
{
	return icm42688_write(d, ICM42688_REG_BANK_SEL, bank);
}

/*
 * Into bank, or, if that write failed, back to bank 0 as well as can be:
 * a select that failed may still have landed. The first error stands.
 */
static int icm42688_enter_bank(struct icm42688 *d, uint8_t bank)
{
	int ret = icm42688_select_bank(d, bank);

	if (ret)
		icm42688_select_bank(d, ICM42688_BANK_0);
	return ret;
}

static int icm42688_bank_update(struct icm42688 *d, uint8_t bank,
				uint8_t reg, uint8_t mask, uint8_t value)
{
	int ret, back;

	ret = icm42688_enter_bank(d, bank);
	if (ret)
		return ret;
	ret = icm42688_update(d, reg, mask, value);
	back = icm42688_select_bank(d, ICM42688_BANK_0);
	return ret ? ret : back;
}

int icm42688_reg_read(struct imu_dev *imu, uint8_t bank, uint8_t reg,
		      void *buf, size_t len)
{
	struct icm42688 *d = to_icm42688(imu);
	int ret, back;

	if (!d || !buf || !icm42688_bank_valid(bank))
		return -EINVAL;
	if (bank == ICM42688_BANK_0)
		return icm42688_read(d, reg, buf, len);
	ret = icm42688_enter_bank(d, bank);
	if (ret)
		return ret;
	ret = icm42688_read(d, reg, buf, len);
	back = icm42688_select_bank(d, ICM42688_BANK_0);
	return ret ? ret : back;
}

int icm42688_reg_write(struct imu_dev *imu, uint8_t bank, uint8_t reg,
		       uint8_t value)
{
	struct icm42688 *d = to_icm42688(imu);

	if (!d || !icm42688_bank_valid(bank))
		return -EINVAL;
	if (bank == ICM42688_BANK_0)
		return icm42688_write(d, reg, value);
	return icm42688_bank_update(d, bank, reg, 0xFF, value);
}

/* ------------------------------------------------------------ config ----- */

static bool icm42688_aaf_valid(uint8_t aaf)
{
	return aaf == ICM42688_AAF_DEFAULT || aaf == ICM42688_AAF_OFF ||
	       icm42688_aaf_row(aaf) != NULL;
}

static bool icm42688_config_valid(const struct icm42688_config *c)
{
	return icm42688_accel_fs_g(c->accel_fs) > 0.0f &&
	       icm42688_gyro_fs_dps(c->gyro_fs) > 0.0f &&
	       icm42688_odr_hz(c->odr) > 0.0f &&
	       icm42688_ui_bw_valid(c->accel_ui_bw) &&
	       icm42688_ui_bw_valid(c->gyro_ui_bw) &&
	       icm42688_ui_order_valid(c->accel_ui_order) &&
	       icm42688_ui_order_valid(c->gyro_ui_order) &&
	       icm42688_aaf_valid(c->accel_aaf) &&
	       icm42688_aaf_valid(c->gyro_aaf);
}

unsigned int icm42688_aaf_bandwidth_hz(uint8_t delt)
{
	const struct icm42688_aaf_row *row = icm42688_aaf_row(delt);

	return row ? row->bandwidth_hz : 0;
}

/*
 * The gyroscope's anti-alias filter, bank 1 (5.3; 15.2-15.5): enabled or
 * bypassed by GYRO_AAF_DIS, then the row's DELT, DELTSQR and BITSHIFT.
 */
static int icm42688_set_gyro_aaf(struct icm42688 *d, uint8_t aaf)
{
	const struct icm42688_aaf_row *row = icm42688_aaf_row(aaf);
	int ret, back;

	if (aaf == ICM42688_AAF_DEFAULT)
		return 0;
	ret = icm42688_enter_bank(d, ICM42688_BANK_1);
	if (ret)
		return ret;
	ret = icm42688_update(d, ICM42688_GYRO_CONFIG_STATIC2,
			      ICM42688_GYRO_AAF_DIS,
			      row ? 0 : ICM42688_GYRO_AAF_DIS);
	if (!ret && row)
		ret = icm42688_update(d, ICM42688_GYRO_CONFIG_STATIC3,
				      ICM42688_AAF_DELT_MASK, aaf);
	if (!ret && row)
		ret = icm42688_write(d, ICM42688_GYRO_CONFIG_STATIC4,
				     (uint8_t)(row->deltsqr & 0xFF));
	if (!ret && row)
		ret = icm42688_write(d, ICM42688_GYRO_CONFIG_STATIC5,
				     (uint8_t)((row->bitshift <<
						ICM42688_AAF_BITSHIFT_SHIFT) |
					       (row->deltsqr >> 8)));
	back = icm42688_select_bank(d, ICM42688_BANK_0);
	return ret ? ret : back;
}

/*
 * The accelerometer's, bank 2 (5.3; 16.1-16.3): ACCEL_AAF_DIS and DELT
 * share ACCEL_CONFIG_STATIC2.
 */
static int icm42688_set_accel_aaf(struct icm42688 *d, uint8_t aaf)
{
	const struct icm42688_aaf_row *row = icm42688_aaf_row(aaf);
	int ret, back;

	if (aaf == ICM42688_AAF_DEFAULT)
		return 0;
	ret = icm42688_enter_bank(d, ICM42688_BANK_2);
	if (ret)
		return ret;
	if (row)
		ret = icm42688_update(d, ICM42688_ACCEL_CONFIG_STATIC2,
				      (uint8_t)((ICM42688_AAF_DELT_MASK <<
					ICM42688_ACCEL_AAF_DELT_SHIFT) |
					ICM42688_ACCEL_AAF_DIS),
				      (uint8_t)(aaf <<
					ICM42688_ACCEL_AAF_DELT_SHIFT));
	else
		ret = icm42688_update(d, ICM42688_ACCEL_CONFIG_STATIC2,
				      ICM42688_ACCEL_AAF_DIS,
				      ICM42688_ACCEL_AAF_DIS);
	if (!ret && row)
		ret = icm42688_write(d, ICM42688_ACCEL_CONFIG_STATIC3,
				     (uint8_t)(row->deltsqr & 0xFF));
	if (!ret && row)
		ret = icm42688_write(d, ICM42688_ACCEL_CONFIG_STATIC4,
				     (uint8_t)((row->bitshift <<
						ICM42688_AAF_BITSHIFT_SHIFT) |
					       (row->deltsqr >> 8)));
	back = icm42688_select_bank(d, ICM42688_BANK_0);
	return ret ? ret : back;
}

/*
 * The UI filters (5.5): order in GYRO_CONFIG1 and ACCEL_CONFIG1, keeping
 * the DEC2_M2 order and the temperature filter around it (14.39, 14.41),
 * bandwidth for both in GYRO_ACCEL_CONFIG0 (14.40).
 */
static int icm42688_set_ui_filters(struct icm42688 *d,
				   const struct icm42688_config *c)
{
	int ret;

	ret = icm42688_update(d, ICM42688_GYRO_CONFIG1,
			      ICM42688_ORDER_MASK << ICM42688_GYRO_ORDER_SHIFT,
			      (uint8_t)(c->gyro_ui_order <<
					ICM42688_GYRO_ORDER_SHIFT));
	if (!ret)
		ret = icm42688_update(d, ICM42688_ACCEL_CONFIG1,
				      ICM42688_ORDER_MASK <<
				      ICM42688_ACCEL_ORDER_SHIFT,
				      (uint8_t)(c->accel_ui_order <<
						ICM42688_ACCEL_ORDER_SHIFT));
	if (!ret)
		ret = icm42688_write(d, ICM42688_GYRO_ACCEL_CONFIG0,
				     (uint8_t)((c->accel_ui_bw <<
						ICM42688_ACCEL_BW_SHIFT) |
					       (c->gyro_ui_bw <<
						ICM42688_GYRO_BW_SHIFT)));
	return ret;
}

/*
 * One register and the cached fields it holds, together: the cache changes
 * only once the write has landed, so imu_read() always scales by what the
 * chip does.
 */
static int icm42688_write_gyro(struct icm42688 *d, int fs, int odr)
{
	int ret = icm42688_write(d, ICM42688_GYRO_CONFIG0,
				 (uint8_t)((fs << ICM42688_FS_SHIFT) | odr));

	if (ret)
		return ret;
	d->gyro_fs = fs;
	d->odr = odr;
	return 0;
}

static int icm42688_write_accel(struct icm42688 *d, int fs, int odr)
{
	int ret = icm42688_write(d, ICM42688_ACCEL_CONFIG0,
				 (uint8_t)((fs << ICM42688_FS_SHIFT) | odr));

	if (ret)
		return ret;
	d->accel_fs = fs;
	d->odr = odr;
	return 0;
}

/*
 * The data-ready interrupt on INT1: pin behaviour (14.3), clear on either
 * an INT_STATUS read or a sensor data read (14.49), the 8 us pulse and no
 * de-assert time, right at every rate, with INT_ASYNC_RESET cleared as 12.6
 * asks (14.50), then the routing (14.51).
 */
static int icm42688_set_int1(struct icm42688 *d,
			     const struct icm42688_int_config *c)
{
	int ret;

	if (!c->enable)
		return 0;
	ret = icm42688_write(d, ICM42688_INT_CONFIG,
			     (uint8_t)((c->active_high ?
					ICM42688_INT1_POLARITY_HIGH : 0) |
				       (c->open_drain ? 0 :
					ICM42688_INT1_PUSH_PULL) |
				       (c->latched ? ICM42688_INT1_LATCHED :
					0)));
	if (!ret)
		ret = icm42688_write(d, ICM42688_INT_CONFIG0,
				     ICM42688_DRDY_CLEAR_ON_EITHER);
	if (!ret)
		ret = icm42688_write(d, ICM42688_INT_CONFIG1,
				     ICM42688_INT_CONFIG1_VALUE);
	if (!ret)
		ret = icm42688_write(d, ICM42688_INT_SOURCE0,
				     ICM42688_UI_DRDY_INT1_EN);
	return ret;
}

int icm42688_set_ranges(struct imu_dev *imu, enum icm42688_accel_fs a,
			enum icm42688_gyro_fs g)
{
	struct icm42688 *d = to_icm42688(imu);
	int ret;

	if (!d || icm42688_accel_fs_g(a) <= 0.0f ||
	    icm42688_gyro_fs_dps(g) <= 0.0f)
		return -EINVAL;
	ret = icm42688_write_accel(d, a, d->odr);
	if (ret)
		return ret;
	return icm42688_write_gyro(d, g, d->odr);
}

int icm42688_set_odr(struct imu_dev *imu, enum icm42688_odr odr)
{
	struct icm42688 *d = to_icm42688(imu);
	int ret;

	if (!d || icm42688_odr_hz(odr) <= 0.0f)
		return -EINVAL;
	ret = icm42688_write_accel(d, d->accel_fs, odr);
	if (ret)
		return ret;
	return icm42688_write_gyro(d, d->gyro_fs, odr);
}

/*
 * PWR_MGMT0 (14.36), low noise or off for each. A sensor coming on from
 * off takes no register writes for 200 us.
 */
static int icm42688_power(struct icm42688 *d, bool accel_on, bool gyro_on)
{
	uint8_t v = (uint8_t)(((accel_on ? ICM42688_MODE_LN :
				ICM42688_MODE_OFF) <<
			       ICM42688_ACCEL_MODE_SHIFT) |
			      ((gyro_on ? ICM42688_MODE_LN :
				ICM42688_MODE_OFF) <<
			       ICM42688_GYRO_MODE_SHIFT));
	bool waking = (v & ~d->pwr_mgmt0) != 0;
	int ret;

	ret = icm42688_write(d, ICM42688_PWR_MGMT0, v);
	if (ret)
		return ret;
	d->pwr_mgmt0 = v;
	if (waking)
		imu_delay_us(ICM42688_MODE_WAIT_US);
	return 0;
}

int icm42688_set_power(struct imu_dev *imu, bool accel_on, bool gyro_on)
{
	struct icm42688 *d = to_icm42688(imu);

	if (!d)
		return -EINVAL;
	return icm42688_power(d, accel_on, gyro_on);
}

float icm42688_accel_sensitivity(struct imu_dev *imu)
{
	struct icm42688 *d = to_icm42688(imu);

	return d ? icm42688_accel_lsb_per_g(d->accel_fs) : 0.0f;
}

float icm42688_gyro_sensitivity(struct imu_dev *imu)
{
	struct icm42688 *d = to_icm42688(imu);

	return d ? icm42688_gyro_lsb_per_dps(d->gyro_fs) : 0.0f;
}

/* -------------------------------------------------------------- read ----- */

/*
 * One burst, TEMP_DATA1 to GYRO_DATA_Z0: temperature, accel and gyro of
 * the same sample. With a latched data-ready interrupt it also releases the
 * pin (14.49).
 */
int icm42688_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		      int16_t *temp)
{
	struct icm42688 *d = to_icm42688(imu);
	uint8_t b[ICM42688_BURST_LEN];
	int ret, i;

	if (!d || !accel || !gyro)
		return -EINVAL;

	ret = icm42688_read(d, ICM42688_TEMP_DATA1, b, sizeof(b));
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		accel[i] = icm42688_sample(&b[ICM42688_BURST_ACCEL + i * 2]);
		gyro[i] = icm42688_sample(&b[ICM42688_BURST_GYRO + i * 2]);
	}
	if (temp)
		*temp = icm42688_sample(&b[0]);
	return 0;
}

/* imu_ops.read: the same burst, in m/s^2, rad/s and degC. */
static int icm42688_read_sample(struct imu_dev *imu, struct imu_sample *s)
{
	struct icm42688 *d = imu->priv;
	float accel_lsb = icm42688_accel_lsb_per_g(d->accel_fs);
	float gyro_lsb = icm42688_gyro_lsb_per_dps(d->gyro_fs);
	int16_t a[3], g[3], t;
	int ret, i;

	ret = icm42688_read_raw(imu, a, g, &t);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		s->accel[i] = (float)a[i] / accel_lsb * IMU_G;
		s->gyro[i] = (float)g[i] / gyro_lsb * IMU_DEG_TO_RAD;
	}
	s->temp_c = icm42688_temp_c(t);
	return 0;
}

int icm42688_data_ready(struct imu_dev *imu, bool *ready)
{
	struct icm42688 *d = to_icm42688(imu);
	uint8_t status = 0;
	int ret;

	if (!d || !ready)
		return -EINVAL;
	ret = icm42688_read(d, ICM42688_INT_STATUS, &status, 1);
	if (ret)
		return ret;
	*ready = (status & ICM42688_DATA_RDY_INT) != 0;
	return 0;
}

static const struct imu_ops icm42688_ops = {
	.read = icm42688_read_sample,
};

/* ------------------------------------------------------- probe/remove ---- */

/*
 * The chip half of both probes, on a bus already attached. Everything but
 * the power mode is set while both sensors are off, as section 12.9
 * requires; they are off out of reset (14.36).
 */
static int icm42688_start(struct imu_dev *imu, struct icm42688 *d,
			  const struct icm42688_config *c)
{
	uint8_t who = 0;
	int ret;

	/*
	 * Bank 0 first: a probe that failed in bank 1 or 2, or an MCU reset in
	 * the middle of a banked access, leaves the chip in that bank while it
	 * stays powered, and WHO_AM_I would read as a stranger. REG_BANK_SEL
	 * is at 0x76 in every bank (14.59).
	 */
	ret = icm42688_select_bank(d, ICM42688_BANK_0);
	if (!ret)
		ret = icm42688_read(d, ICM42688_WHO_AM_I, &who, 1);
	if (ret)
		return ret;
	if (who != ICM42688_WHO_AM_I_VALUE) {
		imu_err(imu, "no ICM-42688-P answering: WHO_AM_I=0x%02x, "
			"expected 0x%02x", who, ICM42688_WHO_AM_I_VALUE);
		return -ENODEV;
	}

	/* A known state, whatever an earlier run left: bank 0, reset values */
	ret = icm42688_write(d, ICM42688_DEVICE_CONFIG, ICM42688_SOFT_RESET);
	if (ret)
		return ret;
	imu_sleep_ms(ICM42688_RESET_WAIT_MS);
	d->pwr_mgmt0 = 0;

	/*
	 * The chip comes up set for SPI; on I2C, section 12.3 gives the
	 * driver settings for that bus: both slew rates 1 (DRIVE_CONFIG,
	 * 14.2) and I3C SDR and DDR off (INTF_CONFIG6, bank 1, 15.19). On SPI
	 * the reset values are already the table's.
	 */
	if (d->bus.ops->kind == IMU_BUS_I2C) {
		ret = icm42688_write(d, ICM42688_DRIVE_CONFIG,
				     ICM42688_DRIVE_CONFIG_I2C);
		if (!ret)
			ret = icm42688_bank_update(d, ICM42688_BANK_1,
						   ICM42688_INTF_CONFIG6,
						   ICM42688_I3C_SDR_EN |
						   ICM42688_I3C_DDR_EN, 0);
		if (ret)
			return ret;
	}

	/* Pin the byte order the burst is decoded in: big endian (14.34). */
	ret = icm42688_update(d, ICM42688_INTF_CONFIG0,
			      ICM42688_SENSOR_DATA_BIG_ENDIAN,
			      ICM42688_SENSOR_DATA_BIG_ENDIAN);
	if (!ret)
		ret = icm42688_set_gyro_aaf(d, c->gyro_aaf);
	if (!ret)
		ret = icm42688_set_accel_aaf(d, c->accel_aaf);
	if (!ret)
		ret = icm42688_set_ui_filters(d, c);
	if (!ret)
		ret = icm42688_write_gyro(d, c->gyro_fs, c->odr);
	if (!ret)
		ret = icm42688_write_accel(d, c->accel_fs, c->odr);
	if (!ret)
		ret = icm42688_set_int1(d, &c->data_ready_int1);
	if (!ret)
		ret = icm42688_power(d, true, true);
	return ret;
}

/* Allocate state and bind it to imu; the bus starts unattached. */
static struct icm42688 *icm42688_bind(struct imu_dev *imu)
{
	struct icm42688 *d = calloc(1, sizeof(*d));

	if (!d)
		return NULL;
	imu->ops = &icm42688_ops;
	imu->priv = d;
	return d;
}

/* Detach the bus if attached, free d, and unbind imu. */
static void icm42688_release(struct imu_dev *imu, struct icm42688 *d)
{
	if (d->bus.ops)
		imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

/* The checks both probes make before touching anything. */
static int icm42688_check(struct imu_dev *imu, const void *bus_config,
			  const struct icm42688_config *config)
{
	if (!imu || !bus_config || !config)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "icm42688";
	if (!icm42688_config_valid(config)) {
		imu_err(imu, "full scale, rate, filter or AAF code outside the "
			"supported set");
		return -EINVAL;
	}
	return 0;
}

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int icm42688_probe_i2c(struct imu_dev *imu,
		       const struct icm42688_i2c_config *bus_config,
		       const struct icm42688_config *config)
{
	struct icm42688 *d;
	uint8_t address;
	uint32_t timeout_ms;
	uint32_t scl_hz = 0;
	int ret;

	ret = icm42688_check(imu, bus_config, config);
	if (ret)
		return ret;
	if (!bus_config->bus)
		return -EINVAL;

	address = bus_config->address ? bus_config->address :
		  ICM42688_I2C_ADDR_LOW;
	if (address != ICM42688_I2C_ADDR_LOW &&
	    address != ICM42688_I2C_ADDR_HIGH) {
		imu_err(imu, "address must be 0x68 or 0x69, not 0x%02x",
			address);
		return -EINVAL;
	}
#ifdef ESP_PLATFORM
	if (bus_config->clock_hz > ICM42688_I2C_MAX_HZ) {
		imu_err(imu, "I2C clock %u Hz is above 1 MHz",
			(unsigned int)bus_config->clock_hz);
		return -EINVAL;
	}
	scl_hz = bus_config->clock_hz ? bus_config->clock_hz :
		 ICM42688_I2C_HZ;
#endif
	timeout_ms = bus_config->timeout_ms > 0 ?
		     (uint32_t)bus_config->timeout_ms : ICM42688_TIMEOUT_MS;

	/* There at all? See imu_i2c_probe(); decided before allocating. */
	ret = imu_i2c_probe(bus_config->bus, address, timeout_ms);
	if (ret) {
		imu_err(imu, "nothing answering at 0x%02x (%d)", address, ret);
		return ret;
	}

	d = icm42688_bind(imu);
	if (!d)
		return -ENOMEM;
	ret = imu_i2c_attach(&d->bus, bus_config->bus, address, scl_hz,
			     timeout_ms);
	if (!ret)
		ret = icm42688_start(imu, d, config);
	if (ret) {
		imu_err(imu, "probe on I2C failed (%d)", ret);
		icm42688_release(imu, d);
		return ret;
	}
	imu_info(imu, "up on I2C at 0x%02x: %.0f g, %.0f dps, %.1f Hz",
		 address, (double)icm42688_accel_fs_g(d->accel_fs),
		 (double)icm42688_gyro_fs_dps(d->gyro_fs),
		 (double)icm42688_odr_hz(d->odr));
	return 0;
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_SPI_MODULE_ENABLED)
int icm42688_probe_spi(struct imu_dev *imu,
		       const struct icm42688_spi_config *bus_config,
		       const struct icm42688_config *config)
{
	struct icm42688 *d;
	int ret;
#ifndef ESP_PLATFORM
	int mode;
#endif

	ret = icm42688_check(imu, bus_config, config);
	if (ret)
		return ret;

#ifdef ESP_PLATFORM
	if (bus_config->cs_pin < 0 ||
	    bus_config->clock_hz > ICM42688_SPI_MAX_HZ ||
	    (bus_config->mode != 0 && bus_config->mode != 3)) {
		imu_err(imu, "SPI needs a CS pin, at most 24 MHz and mode 0 "
			"or 3");
		return -EINVAL;
	}
#else
	mode = bus_config->bus ? imu_spi_mode(bus_config->bus) : -1;
	if ((mode != 0 && mode != 3) || !bus_config->cs_port ||
	    !bus_config->cs_pin) {
		imu_err(imu, "SPI needs mode 0 or 3 with 8-bit frames, and a "
			"CS pin");
		return -EINVAL;
	}
#endif

	d = icm42688_bind(imu);
	if (!d)
		return -ENOMEM;
#ifdef ESP_PLATFORM
	ret = imu_spi_attach(&d->bus, bus_config->host, bus_config->cs_pin,
			     bus_config->clock_hz ? bus_config->clock_hz :
						    ICM42688_SPI_HZ,
			     bus_config->mode);
#else
	/* Deselects CS, so the first transfer starts on a falling edge. */
	ret = imu_spi_attach(&d->bus, bus_config->bus, bus_config->cs_port,
			     bus_config->cs_pin, ICM42688_TIMEOUT_MS);
#endif
	if (!ret)
		ret = icm42688_start(imu, d, config);
	if (ret) {
		imu_err(imu, "probe on SPI failed (%d)", ret);
		icm42688_release(imu, d);
		return ret;
	}
	imu_info(imu, "up on SPI: %.0f g, %.0f dps, %.1f Hz",
		 (double)icm42688_accel_fs_g(d->accel_fs),
		 (double)icm42688_gyro_fs_dps(d->gyro_fs),
		 (double)icm42688_odr_hz(d->odr));
	return 0;
}
#endif

void icm42688_remove(struct imu_dev *imu)
{
	struct icm42688 *d = to_icm42688(imu);

	if (!d)
		return;
	icm42688_power(d, false, false);
	icm42688_release(imu, d);
}

#endif /* CONFIG_IMU_ICM42688 */
