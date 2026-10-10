#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "imu_priv.h"

#if CONFIG_IMU_BMI088

#include "bmi088.h"
#include "bmi088_priv.h"
#include "imu_log.h"

/* The chip half never reads more than one 6-byte burst at a time. */
#define BMI088_BUS_MAX_READ	6
#define BMI088_TIMEOUT_MS	100

#ifdef ESP_PLATFORM
#define BMI088_I2C_HZ		400000
#define BMI088_SPI_HZ		5000000		/* default */
#define BMI088_SPI_MAX_HZ	10000000	/* what the BMI088 tolerates */
#endif

static struct imu_bus *bmi088_die_bus(struct bmi088 *dev, enum bmi088_die die)
{
	return die == BMI088_DIE_ACCEL ? &dev->bus.accel : &dev->bus.gyro;
}

static int bmi088_bus_read(void *context, enum bmi088_die die, uint8_t reg,
			   uint8_t *data, size_t length)
{
	struct bmi088 *dev = context;
	uint8_t rx[1 + BMI088_BUS_MAX_READ];
	struct imu_bus *bus;
	int ret;

	if (!dev || !data || length == 0)
		return -EINVAL;
	if (length > BMI088_BUS_MAX_READ)
		return -EMSGSIZE;

	bus = bmi088_die_bus(dev, die);
	if (!bus->ops)
		return -ENODEV;
	if (bus->ops->kind == IMU_BUS_I2C)
		return imu_reg_read(bus, reg, data, length);

	/*
	 * On a read the accelerometer sends one unpredictable byte after the
	 * address before the register content starts. The gyroscope does not
	 * (datasheet 6.1).
	 */
	if (die == BMI088_DIE_GYRO)
		return imu_reg_read(bus, (uint8_t)(reg | 0x80), data, length);
	ret = imu_reg_read(bus, (uint8_t)(reg | 0x80), rx, 1 + length);
	if (ret)
		return ret;
	memcpy(data, &rx[1], length);
	return 0;
}

static int bmi088_bus_write(void *context, enum bmi088_die die, uint8_t reg,
			    uint8_t value)
{
	struct bmi088 *dev = context;
	struct imu_bus *bus;

	if (!dev)
		return -EINVAL;

	bus = bmi088_die_bus(dev, die);
	if (!bus->ops)
		return -ENODEV;
	if (bus->ops->kind == IMU_BUS_SPI)
		reg &= 0x7f;
	return imu_reg_write(bus, reg, value);
}

static void bmi088_bus_delay_us(void *context, uint32_t usec)
{
	(void)context;
	imu_delay_us(usec);
}

static const struct bmi088_bus_ops bmi088_imu_bus_ops = {
	.read = bmi088_bus_read,
	.write = bmi088_bus_write,
	.delay_us = bmi088_bus_delay_us,
};

/* Detach whichever dies are attached, free dev, and unbind imu. */
static void bmi088_release(struct imu_dev *imu, struct bmi088 *dev)
{
	if (dev->bus.accel.ops)
		imu_bus_detach(&dev->bus.accel);
	if (dev->bus.gyro.ops)
		imu_bus_detach(&dev->bus.gyro);

	free(dev);
	imu->ops = NULL;
	imu->priv = NULL;
}

/* Allocate state and bind it to imu; both buses start unattached. */
static struct bmi088 *bmi088_bind(struct imu_dev *imu)
{
	struct bmi088 *dev = calloc(1, sizeof(*dev));

	if (!dev)
		return NULL;

	imu->ops = &bmi088_imu_ops;
	imu->priv = dev;
	return dev;
}

void bmi088_remove(struct imu_dev *imu)
{
	if (!imu || imu->ops != &bmi088_imu_ops || !imu->priv)
		return;

	bmi088_release(imu, imu->priv);
}

int bmi088_probe_i2c(struct imu_dev *imu,
		     const struct bmi088_i2c_config *bus_config,
		     const struct bmi088_config *config)
{
	struct bmi088 *dev;
	uint8_t accel_address;
	uint8_t gyro_address;
	uint32_t timeout_ms;
	uint32_t scl_hz = 0;
	int ret;

	if (!imu || !bus_config || !bus_config->bus)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "bmi088";

	accel_address = bus_config->accel_address ? bus_config->accel_address :
			BMI088_ACCEL_I2C_ADDR_LOW;
	gyro_address = bus_config->gyro_address ? bus_config->gyro_address :
		       BMI088_GYRO_I2C_ADDR_LOW;

	if ((accel_address != BMI088_ACCEL_I2C_ADDR_LOW &&
	     accel_address != BMI088_ACCEL_I2C_ADDR_HIGH) ||
	    (gyro_address != BMI088_GYRO_I2C_ADDR_LOW &&
	     gyro_address != BMI088_GYRO_I2C_ADDR_HIGH)) {
		imu_err(imu, "accel must be 0x18/0x19 and gyro 0x68/0x69, "
			"not 0x%02x/0x%02x", accel_address, gyro_address);
		return -EINVAL;
	}

#ifdef ESP_PLATFORM
	if (bus_config->clock_hz > BMI088_I2C_HZ) {
		imu_err(imu, "I2C clock %u Hz is above 400 kHz",
			(unsigned int)bus_config->clock_hz);
		return -EINVAL;
	}
	scl_hz = bus_config->clock_hz ? bus_config->clock_hz : BMI088_I2C_HZ;
	/* As before: any nonzero value goes to IDF, -1 waiting for ever. */
	timeout_ms = bus_config->timeout_ms ? (uint32_t)bus_config->timeout_ms :
		     BMI088_TIMEOUT_MS;
#else
	timeout_ms = bus_config->timeout_ms > 0 ?
		     (uint32_t)bus_config->timeout_ms : BMI088_TIMEOUT_MS;
#endif

	/*
	 * Both dies there at all? See imu_i2c_probe() for why not the first
	 * read. Decided before anything is allocated or added to the bus.
	 */
	ret = imu_i2c_probe(bus_config->bus, accel_address, timeout_ms);
	if (ret) {
		imu_err(imu, "no accel answering at 0x%02x (%d)",
			accel_address, ret);
		return ret;
	}
	ret = imu_i2c_probe(bus_config->bus, gyro_address, timeout_ms);
	if (ret) {
		imu_err(imu, "no gyro answering at 0x%02x (%d)", gyro_address,
			ret);
		return ret;
	}

	dev = bmi088_bind(imu);
	if (!dev)
		return -ENOMEM;

	ret = imu_i2c_attach(&dev->bus.accel, bus_config->bus, accel_address,
			     scl_hz, timeout_ms);
	if (!ret)
		ret = imu_i2c_attach(&dev->bus.gyro, bus_config->bus,
				     gyro_address, scl_hz, timeout_ms);
	if (!ret)
		ret = bmi088_init(imu, &bmi088_imu_bus_ops, dev, config);
	if (ret)
		goto fail;

	imu_info(imu, "up on I2C at 0x%02x/0x%02x", accel_address,
		 gyro_address);
	return 0;

fail:
	imu_err(imu, "probe on I2C failed (%d)", ret);
	bmi088_release(imu, dev);
	return ret;
}

int bmi088_probe_spi(struct imu_dev *imu,
		     const struct bmi088_spi_config *bus_config,
		     const struct bmi088_config *config)
{
	struct bmi088 *dev;
	uint8_t chip_id;
	int ret;
#ifdef ESP_PLATFORM
	uint32_t clock_hz;
#else
	int mode;
#endif

	if (!imu || !bus_config)
		return -EINVAL;
	/* Bound already: a second probe would leak the first binding. */
	if (imu->ops || imu->priv)
		return -EBUSY;

	imu->name = "bmi088";

#ifdef ESP_PLATFORM
	if (bus_config->accel_cs_pin == bus_config->gyro_cs_pin ||
	    bus_config->clock_hz > BMI088_SPI_MAX_HZ ||
	    (bus_config->mode != 0 && bus_config->mode != 3)) {
		imu_err(imu, "SPI needs two distinct CS pins, <= 10 MHz and "
			"mode 0 or 3");
		return -EINVAL;
	}
	clock_hz = bus_config->clock_hz ? bus_config->clock_hz : BMI088_SPI_HZ;
#else
	mode = bus_config->bus ? imu_spi_mode(bus_config->bus) : -1;
	if ((mode != 0 && mode != 3) ||
	    !bus_config->accel_cs_port || !bus_config->accel_cs_pin ||
	    !bus_config->gyro_cs_port || !bus_config->gyro_cs_pin ||
	    (bus_config->accel_cs_port == bus_config->gyro_cs_port &&
	     bus_config->accel_cs_pin == bus_config->gyro_cs_pin)) {
		imu_err(imu, "SPI needs mode 0 or 3 with 8-bit frames and two "
			"distinct CS pins");
		return -EINVAL;
	}
#endif

	dev = bmi088_bind(imu);
	if (!dev)
		return -ENOMEM;

#ifdef ESP_PLATFORM
	ret = imu_spi_attach(&dev->bus.accel, bus_config->host,
			     bus_config->accel_cs_pin, clock_hz,
			     bus_config->mode);
	if (!ret)
		ret = imu_spi_attach(&dev->bus.gyro, bus_config->host,
				     bus_config->gyro_cs_pin, clock_hz,
				     bus_config->mode);
	if (ret)
		goto fail;
#else
	/* Each attach deselects its die: each first transfer starts falling. */
	imu_spi_attach(&dev->bus.accel, bus_config->bus,
		       bus_config->accel_cs_port, bus_config->accel_cs_pin,
		       BMI088_TIMEOUT_MS);
	imu_spi_attach(&dev->bus.gyro, bus_config->bus,
		       bus_config->gyro_cs_port, bus_config->gyro_cs_pin,
		       BMI088_TIMEOUT_MS);
#endif

	/*
	 * The accelerometer powers up in I2C mode and only switches to SPI on
	 * a rising edge of its chip select, so throw one read away first. The
	 * value it returns is meaningless (datasheet 3).
	 */
	ret = bmi088_bus_read(dev, BMI088_DIE_ACCEL, 0x00, &chip_id, 1);
	if (ret)
		goto fail;

	ret = bmi088_init(imu, &bmi088_imu_bus_ops, dev, config);
	if (ret)
		goto fail;

#ifdef ESP_PLATFORM
	imu_info(imu, "up on SPI, CS %d/%d", bus_config->accel_cs_pin,
		 bus_config->gyro_cs_pin);
#else
	imu_info(imu, "up on SPI");
#endif
	return 0;

fail:
	imu_err(imu, "probe on SPI failed (%d)", ret);
	bmi088_release(imu, dev);
	return ret;
}

#endif /* CONFIG_IMU_BMI088 */
