#include <errno.h>
#include <stdlib.h>

#include "imu_priv.h"

#if CONFIG_IMU_ICM45686

#include "icm45686.h"
#include "icm45686_math.h"
#include "icm45686_priv.h"
#include "icm45686_regs.h"
#include "imu_log.h"

#define SPI_TIMEOUT_MS	100
/* ESP-IDF: the part allows 24 MHz; wiring often does not. */
#define DEFAULT_SPI_HZ	8000000
/* Mode 3: CPOL high, CPHA second edge, as icm45686.h asks of the bus. */
#define SPI_MODE	3
#define RESET_WAIT_MS	2	/* soft reset needs about 1 ms to settle */
#define IREG_TRIES	20

static const struct imu_ops icm45686_ops;

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct icm45686 *to_icm45686(struct imu_dev *imu)
{
	if (!imu || imu->ops != &icm45686_ops)
		return NULL;
	return imu->priv;
}

/* ---------------------------------------------------------------- SPI ---- */

/*
 * A transaction is one address byte then the data. Bit 7 of the address is
 * set to read, clear to write -- the opposite of several other InvenSense
 * parts, so it is worth stating plainly.
 */
int icm45686_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len)
{
	struct icm45686 *d = to_icm45686(imu);

	if (!d || !buf || len == 0 || len > ICM_DATA_BURST_LEN)
		return -EINVAL;

	/* The byte shifted out under the address is dropped by the bus. */
	return imu_reg_read(&d->bus, (uint8_t)(reg | ICM_SPI_READ), buf, len);
}

int icm45686_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value)
{
	struct icm45686 *d = to_icm45686(imu);

	if (!d)
		return -EINVAL;

	return imu_reg_write(&d->bus, (uint8_t)(reg & ~ICM_SPI_READ), value);
}

/*
 * Anything above 0xFF lives in indirect space: load the address into
 * IREG_ADDR, write or read IREG_DATA, then wait for REG_MISC2.ireg_done.
 * Only SREG_CTRL needs this here, but doing it properly is what lets probe
 * pin the data byte order instead of trusting a reset default.
 */
static int icm45686_ireg_wait(struct imu_dev *imu)
{
	int i, ret;

	for (i = 0; i < IREG_TRIES; i++) {
		uint8_t misc = 0;

		ret = icm45686_reg_read(imu, ICM_REG_MISC2, &misc, 1);
		if (ret)
			return ret;
		if (misc & ICM_IREG_DONE)
			return 0;
		imu_yield();
	}
	return -ETIMEDOUT;
}

static int icm45686_ireg_write(struct imu_dev *imu, uint16_t addr,
			       uint8_t value)
{
	int ret;

	ret = icm45686_reg_write(imu, ICM_IREG_ADDR_15_8, (uint8_t)(addr >> 8));
	if (ret)
		return ret;
	ret = icm45686_reg_write(imu, ICM_IREG_ADDR_7_0, (uint8_t)addr);
	if (ret)
		return ret;
	ret = icm45686_reg_write(imu, ICM_IREG_DATA, value);
	if (ret)
		return ret;
	return icm45686_ireg_wait(imu);
}

/* ------------------------------------------------------------ config ----- */

/*
 * One register, and the cached fields it holds, updated together: the cache
 * changes only once the write has landed. A caller that writes both
 * registers and fails between them is left with a cache that still matches
 * the chip, so imu_read() scales by what the chip actually does.
 */
static int icm45686_write_accel(struct imu_dev *imu, int fs, int odr)
{
	struct icm45686 *d = imu->priv;
	int ret;

	ret = icm45686_reg_write(imu, ICM_ACCEL_CONFIG0,
				 (uint8_t)((fs << ICM_ACCEL_FS_SHIFT) |
					   (odr << ICM_ACCEL_ODR_SHIFT)));
	if (ret)
		return ret;
	d->accel_fs = fs;
	d->accel_odr = odr;
	return 0;
}

static int icm45686_write_gyro(struct imu_dev *imu, int fs, int odr)
{
	struct icm45686 *d = imu->priv;
	int ret;

	ret = icm45686_reg_write(imu, ICM_GYRO_CONFIG0,
				 (uint8_t)((fs << ICM_GYRO_FS_SHIFT) |
					   (odr << ICM_GYRO_ODR_SHIFT)));
	if (ret)
		return ret;
	d->gyro_fs = fs;
	d->gyro_odr = odr;
	return 0;
}

int icm45686_set_ranges(struct imu_dev *imu, enum icm45686_accel_fs a,
			enum icm45686_gyro_fs g)
{
	struct icm45686 *d = to_icm45686(imu);
	int ret;

	if (!d)
		return -EINVAL;
	if (icm45686_accel_fs_g(a) <= 0.0f || icm45686_gyro_fs_dps(g) <= 0.0f)
		return -EINVAL;

	ret = icm45686_write_accel(imu, a, d->accel_odr);
	if (ret)
		return ret;
	return icm45686_write_gyro(imu, g, d->gyro_odr);
}

int icm45686_set_odr(struct imu_dev *imu, enum icm45686_odr odr)
{
	struct icm45686 *d = to_icm45686(imu);
	int ret;

	if (!d)
		return -EINVAL;
	if (icm45686_odr_hz(odr) <= 0.0f)
		return -EINVAL;

	ret = icm45686_write_accel(imu, d->accel_fs, odr);
	if (ret)
		return ret;
	return icm45686_write_gyro(imu, d->gyro_fs, odr);
}

int icm45686_set_power(struct imu_dev *imu, bool accel_on, bool gyro_on)
{
	uint8_t v;

	if (!to_icm45686(imu))
		return -EINVAL;

	v = (uint8_t)(((accel_on ? ICM_MODE_LN : ICM_MODE_OFF) <<
		       ICM_ACCEL_MODE_SHIFT) |
		      ((gyro_on ? ICM_MODE_LN : ICM_MODE_OFF) <<
		       ICM_GYRO_MODE_SHIFT));
	return icm45686_reg_write(imu, ICM_PWR_MGMT0, v);
}

float icm45686_accel_sensitivity(struct imu_dev *imu)
{
	struct icm45686 *d = to_icm45686(imu);

	return d ? icm45686_accel_sensitivity_code(d->accel_fs) : 0.0f;
}

float icm45686_gyro_sensitivity(struct imu_dev *imu)
{
	struct icm45686 *d = to_icm45686(imu);

	return d ? icm45686_gyro_sensitivity_code(d->gyro_fs) : 0.0f;
}

/* -------------------------------------------------------------- read ----- */

int icm45686_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		      int16_t *temp)
{
	struct icm45686 *d = to_icm45686(imu);
	uint8_t b[ICM_DATA_BURST_LEN];
	int ret, i;

	if (!d || !accel || !gyro)
		return -EINVAL;

	/*
	 * One burst: accel, gyro and temperature are contiguous and belong to
	 * the same sample instant. Three separate reads would not.
	 */
	ret = icm45686_reg_read(imu, ICM_ACCEL_DATA_X1_UI, b, sizeof(b));
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		accel[i] = icm45686_sample(&b[i * 2], d->big_endian);
		gyro[i] = icm45686_sample(&b[6 + i * 2], d->big_endian);
	}
	if (temp)
		*temp = icm45686_sample(&b[12], d->big_endian);
	return 0;
}

/* imu_ops.read: the same burst, in m/s^2 and rad/s. */
static int icm45686_read(struct imu_dev *imu, struct imu_sample *s)
{
	struct icm45686 *d = imu->priv;
	int16_t a[3], g[3], t;
	int ret, i;

	ret = icm45686_read_raw(imu, a, g, &t);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		s->accel[i] = icm45686_accel_g(a[i], d->accel_fs) * IMU_G;
		s->gyro[i] = icm45686_gyro_dps(g[i], d->gyro_fs) *
			     IMU_DEG_TO_RAD;
	}
	s->temp_c = icm45686_temp_c(t);
	return 0;
}

int icm45686_data_ready(struct imu_dev *imu, bool *ready)
{
	uint8_t s = 0;
	int ret;

	if (!to_icm45686(imu) || !ready)
		return -EINVAL;

	ret = icm45686_reg_read(imu, ICM_INT1_STATUS0, &s, 1);
	if (ret)
		return ret;
	*ready = (s & ICM_INT_DRDY) != 0;
	return 0;
}

static const struct imu_ops icm45686_ops = {
	.read = icm45686_read,
};

/* ------------------------------------------------------- probe/remove ---- */

int icm45686_probe(struct imu_dev *imu, const struct icm45686_config *cfg)
{
	struct icm45686 *d;
	uint8_t who = 0;
	int ret;

	if (!imu || !cfg)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "icm45686";

	if (icm45686_accel_fs_g(cfg->accel_fs) <= 0.0f ||
	    icm45686_gyro_fs_dps(cfg->gyro_fs) <= 0.0f ||
	    icm45686_odr_hz(cfg->odr) <= 0.0f) {
		imu_err(imu, "full-scale or rate code outside the supported "
			"set");
		return -EINVAL;
	}
#ifndef ESP_PLATFORM
	if (!cfg->spi || !cfg->cs_port || !cfg->cs_pin ||
	    imu_spi_mode(cfg->spi) != SPI_MODE) {
		imu_err(imu, "needs an SPI handle in mode 3 with 8-bit frames, "
			"and a CS pin");
		return -EINVAL;
	}
#endif

	d = calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;

#ifdef ESP_PLATFORM
	ret = imu_spi_attach(&d->bus, cfg->host, cfg->cs_gpio,
			     cfg->spi_hz > 0 ? (uint32_t)cfg->spi_hz :
					       DEFAULT_SPI_HZ,
			     SPI_MODE);
	if (ret) {
		imu_err(imu, "cannot add CS %d to the SPI host (%d)",
			(int)cfg->cs_gpio, ret);
		free(d);
		return ret;
	}
#else
	/* Deselects CS, so the first transfer starts on a falling edge. */
	imu_spi_attach(&d->bus, cfg->spi, cfg->cs_port, cfg->cs_pin,
		       SPI_TIMEOUT_MS);
#endif

	/* From here on the chip helpers above work on this imu_dev. */
	imu->ops = &icm45686_ops;
	imu->priv = d;

	ret = icm45686_reg_write(imu, ICM_REG_MISC2, ICM_SOFT_RST);
	if (ret)
		goto fail;
	imu_sleep_ms(RESET_WAIT_MS);

	ret = icm45686_reg_read(imu, ICM_WHO_AM_I, &who, 1);
	if (ret)
		goto fail;
	if (who != ICM_WHO_AM_I_VALUE) {
		imu_err(imu, "no ICM-45686 answering: WHO_AM_I=0x%02x, expected 0x%02x",
			who, ICM_WHO_AM_I_VALUE);
		ret = -ENODEV;
		goto fail;
	}

	/*
	 * Pin the byte order rather than inherit it. SREG_CTRL survives a soft
	 * reset in some configurations, and a driver that assumes one order and
	 * meets the other returns numbers that look like data and are not.
	 * Little endian is written because it is also the reset default, so
	 * a failed write leaves behaviour unchanged rather than silently
	 * inverted.
	 */
	d->big_endian = false;
	ret = icm45686_ireg_write(imu, ICM_SREG_CTRL, ICM_SREG_LITTLE_ENDIAN <<
						  ICM_SREG_ENDIAN_SHIFT);
	if (ret)
		imu_warn(imu, "could not pin data endianness (%d); assuming "
			 "little endian, the reset default", ret);

	ret = icm45686_write_accel(imu, cfg->accel_fs, cfg->odr);
	if (!ret)
		ret = icm45686_write_gyro(imu, cfg->gyro_fs, cfg->odr);
	if (!ret)
		ret = icm45686_set_power(imu, true, true);
	if (ret)
		goto fail;

	imu_info(imu, "up: %.0f g, %.0f dps, %.4f Hz",
		 (double)icm45686_accel_fs_g(d->accel_fs),
		 (double)icm45686_gyro_fs_dps(d->gyro_fs),
		 (double)icm45686_odr_hz(d->accel_odr));
	return 0;

fail:
	imu->ops = NULL;
	imu->priv = NULL;
	imu_bus_detach(&d->bus);
	free(d);
	return ret;
}

void icm45686_remove(struct imu_dev *imu)
{
	struct icm45686 *d = to_icm45686(imu);

	if (!d)
		return;

	icm45686_set_power(imu, false, false);
	imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

#endif /* CONFIG_IMU_ICM45686 */
