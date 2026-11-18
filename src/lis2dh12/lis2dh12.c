#include <errno.h>
#include <math.h>
#include <stdlib.h>

#include "imu_priv.h"

#if CONFIG_IMU_LIS2DH12

#include "imu_log.h"
#include "lis2dh12.h"
#include "lis2dh12_math.h"
#include "lis2dh12_priv.h"
#include "lis2dh12_regs.h"

#define LIS2DH12_TIMEOUT_MS	100
#ifdef ESP_PLATFORM
/* Default and most: I2C fast mode (table "I2C slave timing values"). */
#define LIS2DH12_I2C_HZ		400000
#endif

static const struct imu_ops lis2dh12_ops;

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct lis2dh12 *to_lis2dh12(struct imu_dev *imu)
{
	if (!imu || imu->ops != &lis2dh12_ops)
		return NULL;
	return imu->priv;
}

/* ---------------------------------------------------------------- bus ---- */

/* More than one byte needs the auto-increment bit (6.1.1). */
static int lis2dh12_read(struct lis2dh12 *d, uint8_t reg, uint8_t *buf,
			 size_t len)
{
	if (len > 1)
		reg |= LIS2DH12_AUTO_INCREMENT;
	return imu_reg_read(&d->bus, reg, buf, len);
}

static int lis2dh12_write(struct lis2dh12 *d, uint8_t reg, uint8_t value)
{
	return imu_reg_write(&d->bus, reg, value);
}

int lis2dh12_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len)
{
	struct lis2dh12 *d = to_lis2dh12(imu);

	if (!d || !buf || len == 0 || (reg & LIS2DH12_AUTO_INCREMENT))
		return -EINVAL;
	return lis2dh12_read(d, reg, buf, len);
}

int lis2dh12_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value)
{
	struct lis2dh12 *d = to_lis2dh12(imu);

	if (!d || (reg & LIS2DH12_AUTO_INCREMENT))
		return -EINVAL;
	return lis2dh12_write(d, reg, value);
}

/* ------------------------------------------------------------ config ----- */

static bool lis2dh12_mode_valid(int mode, int odr, int full_scale)
{
	return lis2dh12_odr_valid(odr, mode) &&
	       lis2dh12_full_scale_g(full_scale) > 0.0f;
}

/* CTRL_REG4 (9.10): block data update always, so a burst never mixes the
 * high byte of one sample with the low byte of the next. */
static uint8_t lis2dh12_ctrl_reg4(int full_scale, bool high_res)
{
	return (uint8_t)(LIS2DH12_BDU | (full_scale << LIS2DH12_FS_SHIFT) |
			 (high_res ? LIS2DH12_HR : 0));
}

/*
 * Mode, rate and full scale. LPen = 1 with HR = 1 is not allowed (table
 * "Operating mode selection"), so: HR off, then CTRL_REG1 with the rate and
 * LPen, then HR on if wanted. The two are never set together, whichever
 * mode the chip comes from. The cache follows each write as it lands, so a
 * failure in between leaves it telling the truth about the chip.
 */
static int lis2dh12_write_mode(struct lis2dh12 *d, int mode, int odr,
			       int full_scale)
{
	uint8_t ctrl_reg1 = (uint8_t)((odr << LIS2DH12_ODR_SHIFT) |
				      (mode == LIS2DH12_MODE_LOW_POWER ?
				       LIS2DH12_LPEN : 0) |
				      LIS2DH12_XYZ_EN);
	int ret;

	ret = lis2dh12_write(d, LIS2DH12_CTRL_REG4,
			     lis2dh12_ctrl_reg4(full_scale, false));
	if (ret)
		return ret;
	d->full_scale = full_scale;
	if (d->mode == LIS2DH12_MODE_HIGH_RES)
		d->mode = LIS2DH12_MODE_NORMAL;

	ret = lis2dh12_write(d, LIS2DH12_CTRL_REG1, ctrl_reg1);
	if (ret)
		return ret;
	d->odr = odr;
	d->mode = mode == LIS2DH12_MODE_LOW_POWER ? LIS2DH12_MODE_LOW_POWER :
						    LIS2DH12_MODE_NORMAL;

	if (mode != LIS2DH12_MODE_HIGH_RES)
		return 0;
	ret = lis2dh12_write(d, LIS2DH12_CTRL_REG4,
			     lis2dh12_ctrl_reg4(full_scale, true));
	if (ret)
		return ret;
	d->mode = mode;
	return 0;
}

int lis2dh12_set_mode(struct imu_dev *imu, enum lis2dh12_mode mode,
		      enum lis2dh12_odr odr,
		      enum lis2dh12_full_scale full_scale)
{
	struct lis2dh12 *d = to_lis2dh12(imu);
	uint8_t threshold = 0;
	int ret;

	if (!d || !lis2dh12_mode_valid(mode, odr, full_scale))
		return -EINVAL;
	/* The motion threshold has to fit the new full scale's step. */
	if (d->motion.axes &&
	    !lis2dh12_threshold_code(d->motion.threshold_g, full_scale,
				     &threshold)) {
		imu_err(imu, "motion threshold %.3f g does not fit %.0f g",
			(double)d->motion.threshold_g,
			(double)lis2dh12_full_scale_g(full_scale));
		return -EINVAL;
	}

	ret = lis2dh12_write_mode(d, mode, odr, full_scale);
	if (!ret && d->motion.axes)
		ret = lis2dh12_write(d, LIS2DH12_INT1_THS, threshold);
	return ret;
}

float lis2dh12_accel_sensitivity(struct imu_dev *imu)
{
	struct lis2dh12 *d = to_lis2dh12(imu);
	float mg = d ? lis2dh12_mg_per_count(d->mode, d->full_scale) : 0.0f;

	return mg > 0.0f ? 1000.0f / mg : 0.0f;
}

/* ------------------------------------------------------------ motion ----- */

/*
 * Off: nothing routed to INT1, generator 1 disabled, not latched, its
 * filter off, and a latched event read away so the pin is released.
 */
static int lis2dh12_motion_off(struct lis2dh12 *d)
{
	uint8_t source;
	int ret;

	ret = lis2dh12_write(d, LIS2DH12_CTRL_REG3, 0);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_INT1_CFG, 0);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG5, 0);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG2,
				     (uint8_t)(d->ctrl_reg2 &
					       ~LIS2DH12_HP_IA1));
	if (ret)
		return ret;
	d->ctrl_reg2 &= (uint8_t)~LIS2DH12_HP_IA1;
	d->motion.axes = 0;
	return lis2dh12_read(d, LIS2DH12_INT1_SRC, &source, 1);
}

int lis2dh12_set_motion(struct imu_dev *imu,
			const struct lis2dh12_motion_config *c)
{
	struct lis2dh12 *d = to_lis2dh12(imu);
	uint8_t threshold = 0, reference, source;
	uint8_t ctrl_reg2;
	int ret;

	if (!d || !c || (c->axes & ~LIS2DH12_AXIS_ALL))
		return -EINVAL;
	if (!c->axes)
		return lis2dh12_motion_off(d);
	if (c->duration > LIS2DH12_7BIT_MAX ||
	    !lis2dh12_threshold_code(c->threshold_g, d->full_scale,
				     &threshold)) {
		imu_err(imu, "motion threshold %.3f g or duration %u outside "
			"1..127 steps at %.0f g", (double)c->threshold_g,
			(unsigned int)c->duration,
			(double)lis2dh12_full_scale_g(d->full_scale));
		return -EINVAL;
	}

	/*
	 * Nothing on the pin while the generator is rebuilt (CTRL_REG3, 9.9),
	 * then threshold and duration (9.25, 9.26) and the filter (9.8).
	 * Reading REFERENCE in HPM = 00 resets the filter to the present
	 * acceleration (9.13), so gravity is not a step the generator sees.
	 */
	ctrl_reg2 = (uint8_t)((d->ctrl_reg2 & ~LIS2DH12_HP_IA1) |
			      (c->high_pass ? LIS2DH12_HP_IA1 : 0));
	ret = lis2dh12_write(d, LIS2DH12_CTRL_REG3, 0);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_INT1_THS, threshold);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_INT1_DURATION, c->duration);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG2, ctrl_reg2);
	if (ret)
		return ret;
	d->ctrl_reg2 = ctrl_reg2;
	if (c->high_pass) {
		ret = lis2dh12_read(d, LIS2DH12_REFERENCE, &reference, 1);
		if (ret)
			return ret;
	}

	/* Latch (9.11), the axes' high events OR'ed (9.23), whatever latched
	 * meanwhile read away (9.24), then routed to INT1 (9.9). */
	ret = lis2dh12_write(d, LIS2DH12_CTRL_REG5,
			     c->latched ? LIS2DH12_LIR_INT1 : 0);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_INT1_CFG,
				     lis2dh12_motion_cfg(c->axes));
	if (!ret)
		ret = lis2dh12_read(d, LIS2DH12_INT1_SRC, &source, 1);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG3, LIS2DH12_I1_IA1);
	if (ret)
		return ret;
	d->motion = *c;
	return 0;
}

int lis2dh12_read_motion(struct imu_dev *imu, bool *active, uint8_t *axes)
{
	struct lis2dh12 *d = to_lis2dh12(imu);
	uint8_t source = 0;
	int ret;

	if (!d || !active)
		return -EINVAL;
	ret = lis2dh12_read(d, LIS2DH12_INT1_SRC, &source, 1);
	if (ret)
		return ret;
	*active = (source & LIS2DH12_IA) != 0;
	if (axes)
		*axes = lis2dh12_motion_axes(source);
	return 0;
}

/* -------------------------------------------------------------- read ----- */

/* OUT_X_L to OUT_Z_H, one burst, at the mode's resolution (9.15-9.20). */
int lis2dh12_read_raw(struct imu_dev *imu, int16_t accel[3])
{
	struct lis2dh12 *d = to_lis2dh12(imu);
	uint8_t b[LIS2DH12_BURST_LEN];
	int ret, i;

	if (!d || !accel)
		return -EINVAL;
	ret = lis2dh12_read(d, LIS2DH12_OUT_X_L, b, sizeof(b));
	if (ret)
		return ret;
	for (i = 0; i < 3; i++)
		accel[i] = lis2dh12_counts(&b[i * 2], d->mode);
	return 0;
}

/* imu_ops.read: the burst in m/s^2; no gyroscope and no absolute
 * temperature on this part, so both are NaN. */
static int lis2dh12_read_sample(struct imu_dev *imu, struct imu_sample *s)
{
	struct lis2dh12 *d = imu->priv;
	int16_t a[3];
	int ret, i;

	ret = lis2dh12_read_raw(imu, a);
	if (ret)
		return ret;
	for (i = 0; i < 3; i++) {
		s->accel[i] = lis2dh12_counts_to_g(a[i], d->mode,
						   d->full_scale) * IMU_G;
		s->gyro[i] = NAN;
	}
	s->temp_c = NAN;
	return 0;
}

int lis2dh12_data_ready(struct imu_dev *imu, bool *ready)
{
	struct lis2dh12 *d = to_lis2dh12(imu);
	uint8_t status = 0;
	int ret;

	if (!d || !ready)
		return -EINVAL;
	ret = lis2dh12_read(d, LIS2DH12_STATUS_REG, &status, 1);
	if (ret)
		return ret;
	*ready = (status & LIS2DH12_ZYXDA) != 0;
	return 0;
}

static const struct imu_ops lis2dh12_ops = {
	.read = lis2dh12_read_sample,
};

/* ------------------------------------------------------- probe/remove ---- */

/*
 * The chip half of the probe. There is no soft reset of the user
 * registers, so every one this driver relies on is written: routing off
 * first, then the pull-up (9.5), temperature sensor off (9.6), the filter
 * (9.8), no latch and FIFO off (9.11), the pin polarity (9.12), FIFO
 * bypass (9.21), both generators, click and sleep-to-wake off (9.23,
 * 9.27, 9.31, 9.37), the mode, and last a latched event of an earlier run
 * read away (9.24).
 */
static int lis2dh12_start(struct imu_dev *imu, struct lis2dh12 *d,
			  const struct lis2dh12_config *c)
{
	static const uint8_t zeroed[] = {
		LIS2DH12_CTRL_REG3, LIS2DH12_TEMP_CFG_REG, LIS2DH12_CTRL_REG5,
		LIS2DH12_FIFO_CTRL_REG, LIS2DH12_INT1_CFG, LIS2DH12_INT2_CFG,
		LIS2DH12_CLICK_CFG, LIS2DH12_ACT_THS,
	};
	uint8_t who = 0, source;
	size_t i;
	int ret;

	ret = lis2dh12_read(d, LIS2DH12_WHO_AM_I, &who, 1);
	if (ret)
		return ret;
	if (who != LIS2DH12_WHO_AM_I_VALUE) {
		imu_err(imu, "no LIS2DH12 answering: WHO_AM_I=0x%02x, "
			"expected 0x%02x", who, LIS2DH12_WHO_AM_I_VALUE);
		return -ENODEV;
	}

	for (i = 0; i < sizeof(zeroed) && !ret; i++)
		ret = lis2dh12_write(d, zeroed[i], 0);
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG0,
				     (uint8_t)(LIS2DH12_CTRL_REG0_FIXED |
					       (c->sa0_pullup_off ?
						LIS2DH12_SDO_PU_DISC : 0)));
	if (!ret) {
		d->ctrl_reg2 = (uint8_t)(c->high_pass_cutoff <<
					 LIS2DH12_HPCF_SHIFT);
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG2, d->ctrl_reg2);
	}
	if (!ret)
		ret = lis2dh12_write(d, LIS2DH12_CTRL_REG6,
				     c->int_active_low ?
				     LIS2DH12_INT_POLARITY : 0);
	if (!ret)
		ret = lis2dh12_write_mode(d, c->mode, c->odr, c->full_scale);
	if (!ret)
		ret = lis2dh12_read(d, LIS2DH12_INT1_SRC, &source, 1);
	return ret;
}

/* Detach the bus if attached, free d, and unbind imu. */
static void lis2dh12_release(struct imu_dev *imu, struct lis2dh12 *d)
{
	if (d->bus.ops)
		imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int lis2dh12_probe_i2c(struct imu_dev *imu,
		       const struct lis2dh12_i2c_config *bus_config,
		       const struct lis2dh12_config *config)
{
	struct lis2dh12 *d;
	uint8_t address;
	uint32_t timeout_ms;
	uint32_t scl_hz = 0;
	int ret;

	if (!imu || !bus_config || !config)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;
	imu->name = "lis2dh12";

	if (!bus_config->bus ||
	    !lis2dh12_mode_valid(config->mode, config->odr,
				 config->full_scale) ||
	    config->odr == LIS2DH12_ODR_POWER_DOWN ||
	    config->high_pass_cutoff > 3) {
		imu_err(imu, "mode, rate, full scale or filter code outside "
			"the supported set");
		return -EINVAL;
	}
	address = bus_config->address ? bus_config->address :
		  LIS2DH12_I2C_ADDR_LOW;
	if (address != LIS2DH12_I2C_ADDR_LOW &&
	    address != LIS2DH12_I2C_ADDR_HIGH) {
		imu_err(imu, "address must be 0x18 or 0x19, not 0x%02x",
			address);
		return -EINVAL;
	}
#ifdef ESP_PLATFORM
	if (bus_config->clock_hz > LIS2DH12_I2C_HZ) {
		imu_err(imu, "I2C clock %u Hz is above 400 kHz",
			(unsigned int)bus_config->clock_hz);
		return -EINVAL;
	}
	scl_hz = bus_config->clock_hz ? bus_config->clock_hz :
		 LIS2DH12_I2C_HZ;
#endif
	timeout_ms = bus_config->timeout_ms > 0 ?
		     (uint32_t)bus_config->timeout_ms : LIS2DH12_TIMEOUT_MS;

	/* There at all? See imu_i2c_probe(); decided before allocating. */
	ret = imu_i2c_probe(bus_config->bus, address, timeout_ms);
	if (ret) {
		imu_err(imu, "nothing answering at 0x%02x (%d)", address, ret);
		return ret;
	}

	d = calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;
	imu->ops = &lis2dh12_ops;
	imu->priv = d;
	ret = imu_i2c_attach(&d->bus, bus_config->bus, address, scl_hz,
			     timeout_ms);
	if (!ret)
		ret = lis2dh12_start(imu, d, config);
	if (ret) {
		imu_err(imu, "probe failed (%d)", ret);
		lis2dh12_release(imu, d);
		return ret;
	}
	imu_info(imu, "up on I2C at 0x%02x: %d-bit, %.0f g, %.0f Hz",
		 address, lis2dh12_resolution_bits(d->mode),
		 (double)lis2dh12_full_scale_g(d->full_scale),
		 (double)lis2dh12_odr_hz(d->odr, d->mode));
	return 0;
}
#endif

void lis2dh12_remove(struct imu_dev *imu)
{
	struct lis2dh12 *d = to_lis2dh12(imu);

	if (!d)
		return;
	/* Power-down (9.7): the ODR bits 0, about 0.5 uA. */
	lis2dh12_write(d, LIS2DH12_CTRL_REG1, LIS2DH12_XYZ_EN);
	lis2dh12_release(imu, d);
}

#endif /* CONFIG_IMU_LIS2DH12 */
