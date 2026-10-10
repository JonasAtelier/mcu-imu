#include <errno.h>
#include <math.h>
#include <stdlib.h>

#include "imu_priv.h"

#if CONFIG_IMU_MPU6050

#include "imu_log.h"
#include "mpu6050.h"
#include "mpu6050_math.h"
#include "mpu6050_priv.h"
#include "mpu6050_regs.h"

#define DEFAULT_RATE_HZ		100.0f

/* ESP-IDF adds the device at the config's clock; the HAL's is the handle's. */
#ifdef ESP_PLATFORM
#define DEFAULT_SCL_HZ		400000
#define mpu6050_scl_hz(cfg)	((cfg)->scl_hz ? (cfg)->scl_hz : DEFAULT_SCL_HZ)
#else
#define mpu6050_scl_hz(cfg)	0
#endif
#define I2C_TIMEOUT_MS		100
#define RESET_WAIT_MS		100	/* the usual figure for the reset */
#define WAKE_WAIT_MS		10

static const struct imu_ops mpu6050_ops;

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct mpu6050 *to_mpu6050(struct imu_dev *imu)
{
	if (!imu || imu->ops != &mpu6050_ops)
		return NULL;
	return imu->priv;
}

/* ---------------------------------------------------------------- I2C ---- */

int mpu6050_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len)
{
	struct mpu6050 *d = to_mpu6050(imu);

	if (!d || !buf || len == 0)
		return -EINVAL;

	/* Register address, repeated start, then the burst. */
	return imu_reg_read(&d->bus, reg, buf, len);
}

int mpu6050_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value)
{
	struct mpu6050 *d = to_mpu6050(imu);

	if (!d)
		return -EINVAL;

	return imu_reg_write(&d->bus, reg, value);
}

static int mpu6050_reg_update(struct imu_dev *imu, uint8_t reg, uint8_t mask,
			      uint8_t value)
{
	uint8_t v = 0;
	int ret;

	ret = mpu6050_reg_read(imu, reg, &v, 1);
	if (ret)
		return ret;
	v = (uint8_t)((v & ~mask) | (value & mask));
	return mpu6050_reg_write(imu, reg, v);
}

/* ------------------------------------------------------------ config ----- */

int mpu6050_set_ranges(struct imu_dev *imu, enum mpu6050_accel_fs a,
		       enum mpu6050_gyro_fs g)
{
	struct mpu6050 *d = to_mpu6050(imu);
	int ret;

	if (!d)
		return -EINVAL;
	if (mpu6050_accel_fs_g(a) <= 0.0f || mpu6050_gyro_fs_dps(g) <= 0.0f)
		return -EINVAL;

	/*
	 * Each cached field follows its own register the moment that write
	 * lands. A failure between the two leaves the cache telling the truth
	 * about a half-applied change instead of mis-scaling every sample.
	 */
	ret = mpu6050_reg_write(imu, MPU6050_ACCEL_CONFIG,
				(uint8_t)(a << MPU6050_FS_SHIFT));
	if (ret)
		return ret;
	d->accel_fs = a;
	ret = mpu6050_reg_write(imu, MPU6050_GYRO_CONFIG,
				(uint8_t)(g << MPU6050_FS_SHIFT));
	if (ret)
		return ret;
	d->gyro_fs = g;
	return 0;
}

int mpu6050_set_rate(struct imu_dev *imu, enum mpu6050_dlpf dlpf, float hz)
{
	struct mpu6050 *d = to_mpu6050(imu);
	uint8_t div;
	int ret;

	if (!d)
		return -EINVAL;
	if (mpu6050_gyro_output_rate(dlpf) <= 0.0f)
		return -EINVAL;

	/*
	 * The filter picks the base rate, so solve the divider against the
	 * filter being installed, not the one currently in force.
	 */
	if (!mpu6050_divider_for(hz, dlpf, &div)) {
		imu_err(imu, "%.2f Hz is out of reach with this filter: "
			"%.0f Hz base divides to %.2f..%.0f Hz",
			(double)hz, (double)mpu6050_gyro_output_rate(dlpf),
			(double)(mpu6050_gyro_output_rate(dlpf) / 256.0f),
			(double)mpu6050_gyro_output_rate(dlpf));
		return -EINVAL;
	}

	/* Lockstep, as in set_ranges: sample_rate() is what the chip does. */
	ret = mpu6050_reg_write(imu, MPU6050_CONFIG, (uint8_t)dlpf);
	if (ret)
		return ret;
	d->dlpf = dlpf;
	ret = mpu6050_reg_write(imu, MPU6050_SMPLRT_DIV, div);
	if (ret)
		return ret;
	d->smplrt_div = div;

	if (fabsf(mpu6050_sample_rate(imu) - hz) > 0.02f * hz)
		imu_warn(imu, "asked %.2f Hz, the divider gives %.2f Hz",
			 (double)hz, (double)mpu6050_sample_rate(imu));
	return 0;
}

float mpu6050_sample_rate(struct imu_dev *imu)
{
	struct mpu6050 *d = to_mpu6050(imu);

	return d ? mpu6050_sample_rate_of(d->dlpf, d->smplrt_div) : 0.0f;
}

int mpu6050_sleep(struct imu_dev *imu, bool sleep)
{
	int ret;

	if (!to_mpu6050(imu))
		return -EINVAL;

	ret = mpu6050_reg_update(imu, MPU6050_PWR_MGMT_1, MPU6050_SLEEP,
				 sleep ? MPU6050_SLEEP : 0);
	if (ret)
		return ret;
	if (!sleep)
		imu_sleep_ms(WAKE_WAIT_MS);
	return 0;
}

int mpu6050_i2c_bypass(struct imu_dev *imu, bool enable)
{
	int ret;

	if (!to_mpu6050(imu))
		return -EINVAL;

	/*
	 * Bypass only takes effect while the internal master is off. This
	 * driver never turns it on, but a previous owner of the chip might
	 * have.
	 */
	ret = mpu6050_reg_update(imu, MPU6050_USER_CTRL, MPU6050_I2C_MST_EN, 0);
	if (ret)
		return ret;
	return mpu6050_reg_update(imu, MPU6050_INT_PIN_CFG,
				  MPU6050_I2C_BYPASS_EN,
				  enable ? MPU6050_I2C_BYPASS_EN : 0);
}

float mpu6050_accel_sensitivity(struct imu_dev *imu)
{
	struct mpu6050 *d = to_mpu6050(imu);

	return d ? mpu6050_accel_sensitivity_of(d->accel_fs) : 0.0f;
}

float mpu6050_gyro_sensitivity(struct imu_dev *imu)
{
	struct mpu6050 *d = to_mpu6050(imu);

	return d ? mpu6050_gyro_sensitivity_of(d->gyro_fs) : 0.0f;
}

/* -------------------------------------------------------------- read ----- */

int mpu6050_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		     int16_t *temp)
{
	uint8_t b[MPU6050_BURST_LEN];
	int ret, i;

	if (!to_mpu6050(imu) || !accel || !gyro)
		return -EINVAL;

	/*
	 * One burst from ACCEL_XOUT_H. Temperature sits between the two
	 * sensors, which is why the gyro offset is 8 and not 6.
	 */
	ret = mpu6050_reg_read(imu, MPU6050_ACCEL_XOUT_H, b, sizeof(b));
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		accel[i] = mpu6050_sample(&b[i * 2]);
		gyro[i] = mpu6050_sample(&b[MPU6050_GYRO_OFF + i * 2]);
	}
	if (temp)
		*temp = mpu6050_sample(&b[MPU6050_TEMP_OFF]);
	return 0;
}

/* imu_ops.read: the same burst, in m/s^2 and rad/s. */
static int mpu6050_read(struct imu_dev *imu, struct imu_sample *s)
{
	struct mpu6050 *d = imu->priv;
	int16_t a[3], g[3], t;
	int ret, i;

	ret = mpu6050_read_raw(imu, a, g, &t);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		s->accel[i] = mpu6050_accel_g(a[i], d->accel_fs) * IMU_G;
		s->gyro[i] = mpu6050_gyro_dps(g[i], d->gyro_fs) *
			     IMU_DEG_TO_RAD;
	}
	s->temp_c = mpu6050_temp_c(t);
	return 0;
}

int mpu6050_data_ready(struct imu_dev *imu, bool *ready)
{
	uint8_t s = 0;
	int ret;

	if (!to_mpu6050(imu) || !ready)
		return -EINVAL;

	ret = mpu6050_reg_read(imu, MPU6050_INT_STATUS, &s, 1);
	if (ret)
		return ret;
	*ready = (s & MPU6050_DATA_RDY) != 0;
	return 0;
}

static const struct imu_ops mpu6050_ops = {
	.read = mpu6050_read,
};

/* ------------------------------------------------------- probe/remove ---- */

int mpu6050_probe(struct imu_dev *imu, imu_i2c_bus bus,
		  const struct mpu6050_config *cfg)
{
	struct mpu6050 *d;
	uint8_t addr, who = 0;
	float rate;
	int ret;

	if (!imu || !cfg)
		return -EINVAL;
#ifndef ESP_PLATFORM
	/*
	 * Only the HAL refuses a NULL handle here. ESP-IDF never did: the bus
	 * is an opaque handle there, and the ESP tests drive a fake bus behind
	 * a NULL one.
	 */
	if (!bus)
		return -EINVAL;
#endif
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "mpu6050";

	addr = cfg->address ? cfg->address : MPU6050_ADDR_LOW;
	if (addr != MPU6050_ADDR_LOW && addr != MPU6050_ADDR_HIGH) {
		imu_err(imu, "address must be 0x68 or 0x69, not 0x%02x", addr);
		return -EINVAL;
	}
	if (mpu6050_accel_fs_g(cfg->accel_fs) <= 0.0f ||
	    mpu6050_gyro_fs_dps(cfg->gyro_fs) <= 0.0f ||
	    mpu6050_gyro_output_rate(cfg->dlpf) <= 0.0f)
		return -EINVAL;

	rate = cfg->sample_rate_hz > 0.0f ? cfg->sample_rate_hz :
					    DEFAULT_RATE_HZ;

	/*
	 * Anything there at all? See imu_i2c_probe() for why not the first
	 * read.
	 */
	ret = imu_i2c_probe(bus, addr, I2C_TIMEOUT_MS);
	if (ret) {
		imu_err(imu, "nothing answered at 0x%02x (%d)", addr, ret);
		return ret;
	}

	d = calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;
	ret = imu_i2c_attach(&d->bus, bus, addr, mpu6050_scl_hz(cfg),
			     I2C_TIMEOUT_MS);
	if (ret) {
		imu_err(imu, "cannot add 0x%02x to the bus (%d)", addr, ret);
		free(d);
		return ret;
	}

	/* From here on the chip helpers above work on this imu_dev. */
	imu->ops = &mpu6050_ops;
	imu->priv = d;

	ret = mpu6050_reg_read(imu, MPU6050_WHO_AM_I, &who, 1);
	if (ret) {
		imu_err(imu, "no answer at 0x%02x (%d)", addr, ret);
		goto fail;
	}
	if (who != MPU6050_WHO_AM_I_VALUE) {
		imu_err(imu, "no MPU-6050 at 0x%02x: WHO_AM_I=0x%02x, expected 0x%02x",
			addr, who, MPU6050_WHO_AM_I_VALUE);
		ret = -ENODEV;
		goto fail;
	}

	ret = mpu6050_reg_write(imu, MPU6050_PWR_MGMT_1, MPU6050_DEVICE_RESET);
	if (ret)
		goto fail;
	imu_sleep_ms(RESET_WAIT_MS);

	/*
	 * Reset leaves the part asleep on its internal 8 MHz RC oscillator.
	 * Clear SLEEP and move to the gyro X PLL in the same write: that
	 * oscillator drifts enough to matter to anything integrating the gyro.
	 */
	ret = mpu6050_reg_write(imu, MPU6050_PWR_MGMT_1, MPU6050_CLKSEL_PLL_X);
	if (ret)
		goto fail;
	imu_sleep_ms(WAKE_WAIT_MS);

	ret = mpu6050_set_rate(imu, cfg->dlpf, rate);
	if (!ret)
		ret = mpu6050_set_ranges(imu, cfg->accel_fs, cfg->gyro_fs);
	if (ret)
		goto fail;

	imu_info(imu, "at 0x%02x: %.0f g, %.0f dps, %.2f Hz", addr,
		 (double)mpu6050_accel_fs_g(d->accel_fs),
		 (double)mpu6050_gyro_fs_dps(d->gyro_fs),
		 (double)mpu6050_sample_rate(imu));
	return 0;

fail:
	imu->ops = NULL;
	imu->priv = NULL;
	imu_bus_detach(&d->bus);
	free(d);
	return ret;
}

void mpu6050_remove(struct imu_dev *imu)
{
	struct mpu6050 *d = to_mpu6050(imu);

	if (!d)
		return;

	mpu6050_reg_write(imu, MPU6050_PWR_MGMT_1, MPU6050_SLEEP);
	imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

#endif /* CONFIG_IMU_MPU6050 */
