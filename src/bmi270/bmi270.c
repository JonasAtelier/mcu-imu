#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "imu_priv.h"

#if CONFIG_IMU_BMI270

#include "bmi270.h"
#include "bmi270_math.h"
#include "bmi270_priv.h"
#include "bmi270_regs.h"
#include "imu_log.h"

#define BMI270_TIMEOUT_MS	100
#ifdef ESP_PLATFORM
#define BMI270_I2C_HZ		400000		/* default */
#define BMI270_I2C_MAX_HZ	1000000		/* Fast-mode Plus, 6.5 */
#define BMI270_SPI_HZ		5000000		/* default */
#define BMI270_SPI_MAX_HZ	10000000	/* Table 19 */
#endif

/*
 * The upload goes in pieces as long as one bus transfer carries, rounded
 * down to whole 16-bit words, which is what INIT_ADDR counts in (4.4,
 * footnote 4; 5.2.65).
 */
#define BMI270_UPLOAD_CHUNK	(IMU_BUS_MAX_LEN & ~1)

/*
 * INTERNAL_STATUS reports the load "after at most 20 msec" (4.4). One poll
 * per millisecond, one more than that, so the last poll comes after 20 ms
 * whatever the tick.
 */
#define BMI270_INIT_POLLS	(BMI270_INIT_WAIT_MS + 1)

static const struct imu_ops bmi270_ops;

/* The chip's reset values (5.2.41-5.2.44); what a NULL config selects. */
static const struct bmi270_config bmi270_defaults = {
	.accel_range = BMI270_ACCEL_RANGE_8G,
	.accel_odr = BMI270_ACCEL_ODR_100_HZ,
	.accel_filter = BMI270_ACCEL_FILTER_NORMAL,
	.gyro_range = BMI270_GYRO_RANGE_2000_DPS,
	.gyro_odr = BMI270_GYRO_ODR_200_HZ,
	.gyro_filter = BMI270_GYRO_FILTER_NORMAL,
};

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct bmi270 *to_bmi270(struct imu_dev *imu)
{
	if (!imu || imu->ops != &bmi270_ops)
		return NULL;
	return imu->priv;
}

/* ---------------------------------------------------------------- bus ---- */

/*
 * len bytes from consecutive registers. On SPI the chip sends one dummy
 * byte after the address before the first register's content (6.4,
 * Figure 13: "for a multiple read operation of n bytes, n+1 bytes have to
 * be read, the first has to be dropped"); on I2C it does not.
 */
static int bmi270_read(struct bmi270 *d, uint8_t reg, uint8_t *buf,
		       size_t len)
{
	uint8_t rx[1 + BMI270_BURST_LEN];
	int ret;

	if (len == 0 || len > BMI270_BURST_LEN)
		return -EINVAL;
	if (d->bus.ops->kind == IMU_BUS_I2C)
		return imu_reg_read(&d->bus, reg, buf, len);

	ret = imu_reg_read(&d->bus, (uint8_t)(reg | BMI270_SPI_READ), rx,
			   1 + len);
	if (ret)
		return ret;
	memcpy(buf, &rx[1], len);
	return 0;
}

/*
 * len bytes to consecutive registers -- or, to INIT_DATA, the next len
 * bytes of the configuration -- in one transfer, then the idle time a
 * write needs before the next access: 2 us in normal mode (Table 19,
 * Table 20). The one write made while advanced power save is still on,
 * which needs 450 us, waits for itself.
 */
static int bmi270_write_burst(struct bmi270 *d, uint8_t reg,
			      const uint8_t *data, size_t len)
{
	int ret;

	ret = d->bus.ops->transfer(&d->bus, (uint8_t)(reg & ~BMI270_SPI_READ),
				   data, len, NULL, 0);
	if (ret)
		return ret;
	imu_delay_us(BMI270_WRITE_IDLE_US);
	return 0;
}

static int bmi270_write(struct bmi270 *d, uint8_t reg, uint8_t value)
{
	return bmi270_write_burst(d, reg, &value, 1);
}

int bmi270_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len)
{
	struct bmi270 *d = to_bmi270(imu);

	if (!d || !buf)
		return -EINVAL;
	return bmi270_read(d, reg, buf, len);
}

int bmi270_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value)
{
	struct bmi270 *d = to_bmi270(imu);

	if (!d)
		return -EINVAL;
	return bmi270_write(d, reg, value);
}

/*
 * The interface comes up as I2C after power-up and after every soft reset,
 * and switches to SPI on a rising edge of CSB, "after 200 us" (6.3; 4.17).
 * One read whose answer is thrown away gives that edge.
 */
static int bmi270_spi_select(struct bmi270 *d)
{
	uint8_t ignored;
	int ret;

	if (d->bus.ops->kind != IMU_BUS_SPI)
		return 0;
	ret = bmi270_read(d, BMI270_CHIP_ID, &ignored, 1);
	if (ret)
		return ret;
	imu_delay_us(BMI270_SPI_SWITCH_US);
	return 0;
}

/* ------------------------------------------------------------- upload ---- */

/*
 * The configuration file into INIT_DATA, a piece at a time: before each
 * piece, its offset in words into INIT_ADDR_0 (bits 3:0) and INIT_ADDR_1
 * (bits 11:4), both in one write (4.4, footnote 4; 5.2.65-5.2.67).
 */
static int bmi270_upload(struct bmi270 *d)
{
	size_t at, len;
	int ret;

	for (at = 0; at < BMI270_CONFIG_SIZE; at += len) {
		uint16_t word = (uint16_t)(at / 2);
		uint8_t addr[2] = {
			(uint8_t)(word & BMI270_INIT_ADDR_0_MASK),
			(uint8_t)(word >> BMI270_INIT_ADDR_1_SHIFT),
		};

		len = BMI270_CONFIG_SIZE - at;
		if (len > BMI270_UPLOAD_CHUNK)
			len = BMI270_UPLOAD_CHUNK;
		ret = bmi270_write_burst(d, BMI270_INIT_ADDR_0, addr,
					 sizeof(addr));
		if (!ret)
			ret = bmi270_write_burst(d, BMI270_INIT_DATA,
						 &bmi270_config_data[at], len);
		if (ret)
			return ret;
	}
	return 0;
}

/* Poll INTERNAL_STATUS.message until it reads init_ok (4.4, 5.2.33). */
static int bmi270_wait_init(struct imu_dev *imu, struct bmi270 *d)
{
	uint8_t status = 0;
	int i, ret;

	for (i = 0; i < BMI270_INIT_POLLS; i++) {
		imu_sleep_ms(1);
		ret = bmi270_read(d, BMI270_INTERNAL_STATUS, &status, 1);
		if (ret)
			return ret;
		if ((status & BMI270_MESSAGE_MASK) == BMI270_MESSAGE_INIT_OK)
			return 0;
	}
	/* 0x00 is "not initialized"; anything else names a failure. */
	if (status & BMI270_MESSAGE_MASK) {
		imu_err(imu, "configuration load failed: INTERNAL_STATUS "
			"message 0x%x", status & BMI270_MESSAGE_MASK);
		return -EPROTO;
	}
	imu_err(imu, "configuration not loaded after %d ms",
		BMI270_INIT_WAIT_MS);
	return -ETIMEDOUT;
}

/* ------------------------------------------------------------ config ----- */

static bool bmi270_config_valid(const struct bmi270_config *c)
{
	return bmi270_accel_range_g(c->accel_range) > 0.0f &&
	       bmi270_accel_odr_valid(c->accel_odr) &&
	       (unsigned)c->accel_filter <= BMI270_ACCEL_FILTER_CIC &&
	       bmi270_gyro_range_dps(c->gyro_range) > 0.0f &&
	       bmi270_gyro_odr_valid(c->gyro_odr) &&
	       (unsigned)c->gyro_filter <= BMI270_GYRO_FILTER_NORMAL;
}

/*
 * One register and the cached field it holds, together: the cache changes
 * only once the write has landed, so imu_read() always scales by what the
 * chip does.
 */
static int bmi270_write_acc_conf(struct bmi270 *d, uint8_t conf)
{
	int ret = bmi270_write(d, BMI270_ACC_CONF, conf);

	if (ret)
		return ret;
	d->acc_conf = conf;
	d->accel_odr = conf & 0x0F;
	return 0;
}

static int bmi270_write_gyr_conf(struct bmi270 *d, uint8_t conf)
{
	int ret = bmi270_write(d, BMI270_GYR_CONF, conf);

	if (ret)
		return ret;
	d->gyr_conf = conf;
	d->gyro_odr = conf & 0x0F;
	return 0;
}

static int bmi270_write_accel_range(struct bmi270 *d, int range)
{
	int ret = bmi270_write(d, BMI270_ACC_RANGE, (uint8_t)range);

	if (!ret)
		d->accel_range = range;
	return ret;
}

static int bmi270_write_gyro_range(struct bmi270 *d, int range)
{
	int ret = bmi270_write(d, BMI270_GYR_RANGE, (uint8_t)range);

	if (!ret)
		d->gyro_range = range;
	return ret;
}

/*
 * The datasheet's normal mode, its quick start 3 in that order: PWR_CTRL
 * on for accelerometer, gyroscope and temperature, ACC_CONF and GYR_CONF
 * with filter performance on, then PWR_CONF with advanced power save off
 * and FIFO self wake-up left on (section 3; Table 6). gyro_low_noise turns
 * that into performance mode (section 3, step 4).
 */
static int bmi270_apply(struct bmi270 *d, const struct bmi270_config *c)
{
	uint8_t acc_conf, gyr_conf;
	int ret;

	acc_conf = (uint8_t)(BMI270_ACC_FILTER_PERF |
			     (c->accel_filter << BMI270_ACC_BWP_SHIFT) |
			     (c->accel_odr << BMI270_ACC_ODR_SHIFT));
	gyr_conf = (uint8_t)(BMI270_GYR_FILTER_PERF |
			     (c->gyro_low_noise ? BMI270_GYR_NOISE_PERF : 0) |
			     (c->gyro_filter << BMI270_GYR_BWP_SHIFT) |
			     (c->gyro_odr << BMI270_GYR_ODR_SHIFT));

	ret = bmi270_write(d, BMI270_PWR_CTRL,
			   BMI270_ACC_EN | BMI270_GYR_EN | BMI270_TEMP_EN);
	if (!ret)
		ret = bmi270_write_acc_conf(d, acc_conf);
	if (!ret)
		ret = bmi270_write_accel_range(d, c->accel_range);
	if (!ret)
		ret = bmi270_write_gyr_conf(d, gyr_conf);
	if (!ret)
		ret = bmi270_write_gyro_range(d, c->gyro_range);
	if (!ret)
		ret = bmi270_write(d, BMI270_PWR_CONF,
				   BMI270_FIFO_SELF_WAKEUP);
	if (ret || !c->data_ready_int1.enable)
		return ret;

	/* Pin behaviour, latch mode, then the data-ready mapping (4.9). */
	ret = bmi270_write(d, BMI270_INT1_IO_CTRL,
			   (uint8_t)(BMI270_INT_OUTPUT_EN |
				     (c->data_ready_int1.active_high ?
				      BMI270_INT_LVL_HIGH : 0) |
				     (c->data_ready_int1.open_drain ?
				      BMI270_INT_OD : 0)));
	if (!ret)
		ret = bmi270_write(d, BMI270_INT_LATCH,
				   c->data_ready_int1.latched ?
				   BMI270_INT_LATCHED : 0);
	if (!ret)
		ret = bmi270_write(d, BMI270_INT_MAP_DATA, BMI270_DRDY_INT1);
	return ret;
}

int bmi270_set_ranges(struct imu_dev *imu, enum bmi270_accel_range a,
		      enum bmi270_gyro_range g)
{
	struct bmi270 *d = to_bmi270(imu);
	int ret;

	if (!d || bmi270_accel_range_g(a) <= 0.0f ||
	    bmi270_gyro_range_dps(g) <= 0.0f)
		return -EINVAL;
	ret = bmi270_write_accel_range(d, a);
	if (ret)
		return ret;
	return bmi270_write_gyro_range(d, g);
}

int bmi270_set_odr(struct imu_dev *imu, enum bmi270_accel_odr a,
		   enum bmi270_gyro_odr g)
{
	struct bmi270 *d = to_bmi270(imu);
	int ret;

	if (!d || !bmi270_accel_odr_valid(a) || !bmi270_gyro_odr_valid(g))
		return -EINVAL;
	ret = bmi270_write_acc_conf(d, (uint8_t)((d->acc_conf & 0xF0) | a));
	if (ret)
		return ret;
	return bmi270_write_gyr_conf(d, (uint8_t)((d->gyr_conf & 0xF0) | g));
}

float bmi270_accel_sensitivity(struct imu_dev *imu)
{
	struct bmi270 *d = to_bmi270(imu);

	return d ? bmi270_accel_lsb_per_g(d->accel_range) : 0.0f;
}

float bmi270_gyro_sensitivity(struct imu_dev *imu)
{
	struct bmi270 *d = to_bmi270(imu);

	return d ? bmi270_gyro_lsb_per_dps(d->gyro_range) : 0.0f;
}

/* -------------------------------------------------------------- read ----- */

/*
 * One burst, DATA_8 to TEMPERATURE_1, so accel, gyro and temperature belong
 * together. Reading the registers between them clears EVENT (5.2.27) and
 * INT_STATUS_0/1 (5.2.28-5.2.29), which this driver does not otherwise
 * use; in latched mode that releases the INT1 pin.
 */
int bmi270_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		    int16_t *temp)
{
	struct bmi270 *d = to_bmi270(imu);
	uint8_t b[BMI270_BURST_LEN];
	int ret, i;

	if (!d || !accel || !gyro)
		return -EINVAL;

	ret = bmi270_read(d, BMI270_DATA_8, b, sizeof(b));
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		accel[i] = bmi270_sample(&b[i * 2]);
		gyro[i] = bmi270_sample(&b[BMI270_BURST_GYRO + i * 2]);
	}
	gyro[0] = bmi270_cas_correct(gyro[0], gyro[2], d->cas_factor);
	if (temp)
		*temp = bmi270_sample(&b[BMI270_BURST_TEMP]);
	return 0;
}

/* imu_ops.read: the same burst, in m/s^2, rad/s and degC. */
static int bmi270_read_sample(struct imu_dev *imu, struct imu_sample *s)
{
	struct bmi270 *d = imu->priv;
	float accel_lsb = bmi270_accel_lsb_per_g(d->accel_range);
	float gyro_lsb = bmi270_gyro_lsb_per_dps(d->gyro_range);
	int16_t a[3], g[3], t;
	int ret, i;

	ret = bmi270_read_raw(imu, a, g, &t);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		s->accel[i] = (float)a[i] / accel_lsb * IMU_G;
		s->gyro[i] = (float)g[i] / gyro_lsb * IMU_DEG_TO_RAD;
	}
	s->temp_c = bmi270_temp_c(t);
	return 0;
}

int bmi270_data_ready(struct imu_dev *imu, uint8_t *flags)
{
	struct bmi270 *d = to_bmi270(imu);
	uint8_t status = 0;
	int ret;

	if (!d || !flags)
		return -EINVAL;
	ret = bmi270_read(d, BMI270_STATUS, &status, 1);
	if (ret)
		return ret;
	*flags = (uint8_t)((status & BMI270_STATUS_DRDY_ACC ?
			    BMI270_DATA_READY_ACCEL : 0) |
			   (status & BMI270_STATUS_DRDY_GYR ?
			    BMI270_DATA_READY_GYRO : 0));
	return 0;
}

static const struct imu_ops bmi270_ops = {
	.read = bmi270_read_sample,
};

/* ------------------------------------------------------- probe/remove ---- */

/*
 * The chip half of both probes, on a bus already attached: the
 * initialization of section 4.4, the cross-axis factor, then config.
 */
static int bmi270_start(struct imu_dev *imu, struct bmi270 *d,
			const struct bmi270_config *config)
{
	uint8_t id = 0, cas = 0;
	int ret;

	ret = bmi270_spi_select(d);
	if (!ret)
		ret = bmi270_read(d, BMI270_CHIP_ID, &id, 1);
	if (ret)
		return ret;
	if (id != BMI270_CHIP_ID_VALUE) {
		imu_err(imu, "no BMI270 answering: CHIP_ID=0x%02x, expected "
			"0x%02x", id, BMI270_CHIP_ID_VALUE);
		return -ENODEV;
	}

	/*
	 * A soft reset first, so the upload always meets a chip fresh from
	 * reset: INIT_CTRL may be set to 1 only once after a POR or soft
	 * reset (4.4), and a re-probe would otherwise be a second time.
	 */
	ret = bmi270_write(d, BMI270_CMD, BMI270_CMD_SOFTRESET);
	if (ret)
		return ret;
	imu_sleep_ms(BMI270_RESET_WAIT_MS);
	ret = bmi270_spi_select(d);
	if (ret)
		return ret;

	/* Section 4.4, in order. */
	ret = bmi270_write(d, BMI270_PWR_CONF, 0);	/* adv_power_save off */
	if (ret)
		return ret;
	imu_delay_us(BMI270_APS_OFF_WAIT_US);
	ret = bmi270_write(d, BMI270_INIT_CTRL, BMI270_INIT_CTRL_PREPARE);
	if (!ret)
		ret = bmi270_upload(d);
	if (!ret)
		ret = bmi270_write(d, BMI270_INIT_CTRL, BMI270_INIT_CTRL_DONE);
	if (!ret)
		ret = bmi270_wait_init(imu, d);
	if (ret)
		return ret;

	/* GYR_CAS sits on feature page 0 (5.2.39-5.2.40). */
	ret = bmi270_write(d, BMI270_FEAT_PAGE, 0);
	if (!ret)
		ret = bmi270_read(d, BMI270_GYR_CAS, &cas, 1);
	if (ret)
		return ret;
	d->cas_factor = bmi270_cas_factor(cas);

	return bmi270_apply(d, config);
}

/* Allocate state and bind it to imu; the bus starts unattached. */
static struct bmi270 *bmi270_bind(struct imu_dev *imu)
{
	struct bmi270 *d = calloc(1, sizeof(*d));

	if (!d)
		return NULL;
	imu->ops = &bmi270_ops;
	imu->priv = d;
	return d;
}

/* Detach the bus if attached, free d, and unbind imu. */
static void bmi270_release(struct imu_dev *imu, struct bmi270 *d)
{
	if (d->bus.ops)
		imu_bus_detach(&d->bus);
	free(d);
	imu->ops = NULL;
	imu->priv = NULL;
}

/* The checks both probes make before touching anything. */
static int bmi270_check(struct imu_dev *imu, const void *bus_config,
			const struct bmi270_config **config)
{
	if (!imu || !bus_config)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "bmi270";
	if (!*config)
		*config = &bmi270_defaults;
	if (!bmi270_config_valid(*config)) {
		imu_err(imu, "range, rate or filter code outside the supported "
			"set");
		return -EINVAL;
	}
	return 0;
}

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int bmi270_probe_i2c(struct imu_dev *imu,
		     const struct bmi270_i2c_config *bus_config,
		     const struct bmi270_config *config)
{
	struct bmi270 *d;
	uint8_t address;
	uint32_t timeout_ms;
	uint32_t scl_hz = 0;
	int ret;

	ret = bmi270_check(imu, bus_config, &config);
	if (ret)
		return ret;
	if (!bus_config->bus)
		return -EINVAL;

	address = bus_config->address ? bus_config->address :
		  BMI270_I2C_ADDR_LOW;
	if (address != BMI270_I2C_ADDR_LOW && address != BMI270_I2C_ADDR_HIGH) {
		imu_err(imu, "address must be 0x68 or 0x69, not 0x%02x",
			address);
		return -EINVAL;
	}
#ifdef ESP_PLATFORM
	if (bus_config->clock_hz > BMI270_I2C_MAX_HZ) {
		imu_err(imu, "I2C clock %u Hz is above 1 MHz",
			(unsigned int)bus_config->clock_hz);
		return -EINVAL;
	}
	scl_hz = bus_config->clock_hz ? bus_config->clock_hz : BMI270_I2C_HZ;
#endif
	timeout_ms = bus_config->timeout_ms > 0 ?
		     (uint32_t)bus_config->timeout_ms : BMI270_TIMEOUT_MS;

	/* There at all? See imu_i2c_probe(); decided before allocating. */
	ret = imu_i2c_probe(bus_config->bus, address, timeout_ms);
	if (ret) {
		imu_err(imu, "nothing answering at 0x%02x (%d)", address, ret);
		return ret;
	}

	d = bmi270_bind(imu);
	if (!d)
		return -ENOMEM;
	ret = imu_i2c_attach(&d->bus, bus_config->bus, address, scl_hz,
			     timeout_ms);
	if (!ret)
		ret = bmi270_start(imu, d, config);
	if (ret) {
		imu_err(imu, "probe on I2C failed (%d)", ret);
		bmi270_release(imu, d);
		return ret;
	}
	imu_info(imu, "up on I2C at 0x%02x", address);
	return 0;
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_SPI_MODULE_ENABLED)
int bmi270_probe_spi(struct imu_dev *imu,
		     const struct bmi270_spi_config *bus_config,
		     const struct bmi270_config *config)
{
	struct bmi270 *d;
	int ret;
#ifndef ESP_PLATFORM
	int mode;
#endif

	ret = bmi270_check(imu, bus_config, &config);
	if (ret)
		return ret;

#ifdef ESP_PLATFORM
	if (bus_config->cs_pin < 0 ||
	    bus_config->clock_hz > BMI270_SPI_MAX_HZ ||
	    (bus_config->mode != 0 && bus_config->mode != 3)) {
		imu_err(imu, "SPI needs a CS pin, at most 10 MHz and mode 0 "
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

	d = bmi270_bind(imu);
	if (!d)
		return -ENOMEM;
#ifdef ESP_PLATFORM
	ret = imu_spi_attach(&d->bus, bus_config->host, bus_config->cs_pin,
			     bus_config->clock_hz ? bus_config->clock_hz :
						    BMI270_SPI_HZ,
			     bus_config->mode);
#else
	/* Deselects CS, so the first transfer starts on a falling edge. */
	ret = imu_spi_attach(&d->bus, bus_config->bus, bus_config->cs_port,
			     bus_config->cs_pin, BMI270_TIMEOUT_MS);
#endif
	if (!ret)
		ret = bmi270_start(imu, d, config);
	if (ret) {
		imu_err(imu, "probe on SPI failed (%d)", ret);
		bmi270_release(imu, d);
		return ret;
	}
	imu_info(imu, "up on SPI");
	return 0;
}
#endif

void bmi270_remove(struct imu_dev *imu)
{
	struct bmi270 *d = to_bmi270(imu);

	if (!d)
		return;
	bmi270_write(d, BMI270_PWR_CTRL, 0);
	bmi270_release(imu, d);
}

#endif /* CONFIG_IMU_BMI270 */
