#include <errno.h>

#include "imu_priv.h"

#if CONFIG_IMU_BMI088

#include "bmi088.h"
#include "bmi088_priv.h"
#include "bmi088_regs.h"
#include "imu_log.h"

/*
 * Through int, because an enum's size and signedness are the ABI's choice:
 * bare-metal ARM packs these into an unsigned byte, where a "< 0" check is
 * dead code and -Wextra says so.
 */
static int bmi088_in_range(int value, int low, int high)
{
	return value >= low && value <= high;
}

static int bmi088_validate_config(const struct bmi088_config *config)
{
	if (!bmi088_in_range(config->accel_range, BMI088_ACCEL_RANGE_3G,
			     BMI088_ACCEL_RANGE_24G))
		return -EINVAL;

	if (!bmi088_in_range(config->accel_odr, BMI088_ACCEL_ODR_12_5_HZ,
			     BMI088_ACCEL_ODR_1600_HZ))
		return -EINVAL;

	if (!bmi088_in_range(config->accel_filter, BMI088_ACCEL_FILTER_OSR4,
			     BMI088_ACCEL_FILTER_NORMAL))
		return -EINVAL;

	if (!bmi088_in_range(config->gyro_range, BMI088_GYRO_RANGE_2000_DPS,
			     BMI088_GYRO_RANGE_125_DPS))
		return -EINVAL;

	if (!bmi088_in_range(config->gyro_bandwidth,
			     BMI088_GYRO_ODR_2000_HZ_BW_532_HZ,
			     BMI088_GYRO_ODR_100_HZ_BW_32_HZ))
		return -EINVAL;

	return 0;
}

static int bmi088_read_reg(struct bmi088 *dev, enum bmi088_die die,
			   uint8_t reg, uint8_t *value)
{
	return dev->ops->read(dev->bus_context, die, reg, value, 1);
}

static int bmi088_write_reg(struct bmi088 *dev, enum bmi088_die die,
			    uint8_t reg, uint8_t value)
{
	return dev->ops->write(dev->bus_context, die, reg, value);
}

static int bmi088_update_bits(struct bmi088 *dev, enum bmi088_die die,
			      uint8_t reg, uint8_t mask, uint8_t value)
{
	uint8_t old_value;
	int ret;

	ret = bmi088_read_reg(dev, die, reg, &old_value);
	if (ret)
		return ret;

	/* Preserve reserved bits when a register only exposes a bit field. */
	value = (old_value & (uint8_t)~mask) | (value & mask);

	ret = bmi088_write_reg(dev, die, reg, value);
	if (ret)
		return ret;

	/* Register writes need at least 2 us before the next bus access. */
	dev->ops->delay_us(dev->bus_context, 2);

	return 0;
}

static int bmi088_check_bits(struct bmi088 *dev, enum bmi088_die die,
			     uint8_t reg, uint8_t mask, uint8_t expected)
{
	uint8_t value;
	int ret;

	ret = bmi088_read_reg(dev, die, reg, &value);
	if (ret)
		return ret;

	if ((value & mask) != (expected & mask))
		return -EIO;

	return 0;
}

static int bmi088_check_chip_id(struct imu_dev *imu, enum bmi088_die die)
{
	struct bmi088 *dev = imu->priv;
	uint8_t expected = die == BMI088_DIE_ACCEL ? BMI088_ACCEL_CHIP_ID :
						     BMI088_GYRO_CHIP_ID;
	uint8_t chip_id;
	int ret;

	ret = bmi088_read_reg(dev, die, BMI088_REG_CHIP_ID, &chip_id);
	if (ret)
		return ret;
	if (chip_id != expected) {
		imu_err(imu, "%s CHIP_ID=0x%02x, expected 0x%02x",
			die == BMI088_DIE_ACCEL ? "accel" : "gyro", chip_id,
			expected);
		return -ENODEV;
	}

	return 0;
}

static int32_t bmi088_decode_s16(uint8_t lsb, uint8_t msb)
{
	uint16_t value = ((uint16_t)msb << 8) | lsb;

	/* Avoid implementation-defined unsigned-to-signed conversion. */
	if (value >= 0x8000)
		return (int32_t)value - 65536;

	return value;
}

static void bmi088_decode_vec3(const uint8_t data[6],
			       struct bmi088_vec3_raw *vector)
{
	vector->x = bmi088_decode_s16(data[0], data[1]);
	vector->y = bmi088_decode_s16(data[2], data[3]);
	vector->z = bmi088_decode_s16(data[4], data[5]);
}

/* Our state, or NULL if this imu_dev was not probed by this driver. */
static struct bmi088 *to_bmi088(struct imu_dev *imu)
{
	if (!imu || imu->ops != &bmi088_imu_ops)
		return NULL;
	return imu->priv;
}

static int bmi088_require_ready(const struct bmi088 *dev, const void *output)
{
	if (!dev || !output)
		return -EINVAL;

	if (!dev->initialized)
		return -ENODEV;

	return 0;
}

struct bmi088_config bmi088_default_config(void)
{
	struct bmi088_config config = {
		.accel_range = BMI088_ACCEL_RANGE_6G,
		.accel_odr = BMI088_ACCEL_ODR_100_HZ,
		.accel_filter = BMI088_ACCEL_FILTER_NORMAL,
		.gyro_range = BMI088_GYRO_RANGE_2000_DPS,
		.gyro_bandwidth = BMI088_GYRO_ODR_200_HZ_BW_23_HZ,
	};

	return config;
}

int bmi088_init(struct imu_dev *imu, const struct bmi088_bus_ops *ops,
		void *bus_context, const struct bmi088_config *config)
{
	struct bmi088_config selected_config;
	struct bmi088 *dev;
	uint8_t accel_conf;
	uint8_t chip_id;
	uint8_t error;
	int ret;

	if (!imu || !imu->priv || !ops || !ops->read || !ops->write ||
	    !ops->delay_us)
		return -EINVAL;
	dev = imu->priv;

	selected_config = config ? *config : bmi088_default_config();

	ret = bmi088_validate_config(&selected_config);
	if (ret) {
		imu_err(imu, "config outside the supported set");
		return ret;
	}

	dev->ops = ops;
	dev->bus_context = bus_context;
	dev->config = selected_config;
	dev->initialized = 0;

	ops->delay_us(bus_context, 1000);

	ret = bmi088_check_chip_id(imu, BMI088_DIE_ACCEL);
	if (ret)
		return ret;
	ret = bmi088_check_chip_id(imu, BMI088_DIE_GYRO);
	if (ret)
		return ret;

	/*
	 * Stop an already-running accelerometer before reset. This also makes
	 * reinitialization safe after an MCU-only restart.
	 */
	ret = bmi088_write_reg(dev, BMI088_DIE_ACCEL,
			       BMI088_ACCEL_REG_PWR_CONF,
			       BMI088_ACCEL_PWR_SUSPEND);
	if (ret)
		return ret;
	ops->delay_us(bus_context, 1000);

	ret = bmi088_write_reg(dev, BMI088_DIE_ACCEL,
			       BMI088_ACCEL_REG_PWR_CTRL,
			       BMI088_ACCEL_DISABLE);
	if (ret)
		return ret;
	ops->delay_us(bus_context, 1000);

	ret = bmi088_write_reg(dev, BMI088_DIE_ACCEL,
			       BMI088_ACCEL_REG_SOFT_RESET,
			       BMI088_SOFT_RESET_COMMAND);
	if (ret)
		return ret;
	/* Accel reset needs 1 ms; gyro reset below needs 30 ms. */
	ops->delay_us(bus_context, 1000);

	/*
	 * The first post-reset SPI read only reselects SPI. Discarding one read
	 * is harmless on I2C and keeps bus-specific reset handling out of here.
	 */
	ret = bmi088_read_reg(dev, BMI088_DIE_ACCEL, BMI088_REG_CHIP_ID,
			      &chip_id);
	if (ret)
		return ret;
	ret = bmi088_check_chip_id(imu, BMI088_DIE_ACCEL);
	if (ret)
		return ret;

	ret = bmi088_write_reg(dev, BMI088_DIE_GYRO,
			       BMI088_GYRO_REG_SOFT_RESET,
			       BMI088_SOFT_RESET_COMMAND);
	if (ret)
		return ret;
	ops->delay_us(bus_context, 30000);

	ret = bmi088_write_reg(dev, BMI088_DIE_ACCEL,
			       BMI088_ACCEL_REG_PWR_CTRL, BMI088_ACCEL_ENABLE);
	if (ret)
		return ret;
	/* Each power register change needs 1 ms before another access. */
	ops->delay_us(bus_context, 1000);

	ret = bmi088_write_reg(dev, BMI088_DIE_ACCEL,
			       BMI088_ACCEL_REG_PWR_CONF,
			       BMI088_ACCEL_PWR_ACTIVE);
	if (ret)
		return ret;
	ops->delay_us(bus_context, 1000);

	ret = bmi088_update_bits(dev, BMI088_DIE_ACCEL,
				 BMI088_ACCEL_REG_RANGE,
				 BMI088_ACCEL_RANGE_MASK,
				 selected_config.accel_range);
	if (ret)
		return ret;

	accel_conf = ((uint8_t)selected_config.accel_filter << 4) |
		     (uint8_t)selected_config.accel_odr;
	ret = bmi088_update_bits(dev, BMI088_DIE_ACCEL,
				 BMI088_ACCEL_REG_CONF,
				 BMI088_ACCEL_CONF_MASK, accel_conf);
	if (ret)
		return ret;

	ret = bmi088_update_bits(dev, BMI088_DIE_GYRO,
				 BMI088_GYRO_REG_RANGE,
				 BMI088_GYRO_RANGE_MASK,
				 selected_config.gyro_range);
	if (ret)
		return ret;

	ret = bmi088_update_bits(dev, BMI088_DIE_GYRO,
				 BMI088_GYRO_REG_BANDWIDTH,
				 BMI088_GYRO_BANDWIDTH_MASK,
				 selected_config.gyro_bandwidth);
	if (ret)
		return ret;

	ret = bmi088_update_bits(dev, BMI088_DIE_GYRO,
				 BMI088_GYRO_REG_INT_CTRL,
				 BMI088_GYRO_DRDY_ENABLE,
				 BMI088_GYRO_DRDY_ENABLE);
	if (ret)
		return ret;

	ret = bmi088_read_reg(dev, BMI088_DIE_ACCEL,
			      BMI088_ACCEL_REG_ERR, &error);
	if (ret)
		return ret;
	if (error & BMI088_ACCEL_ERR_MASK) {
		imu_err(imu, "accel ERR_REG=0x%02x after config", error);
		return -EIO;
	}

	ret = bmi088_check_bits(dev, BMI088_DIE_ACCEL,
				BMI088_ACCEL_REG_PWR_CTRL, 0xff,
				BMI088_ACCEL_ENABLE);
	if (ret)
		return ret;

	ret = bmi088_check_bits(dev, BMI088_DIE_ACCEL,
				BMI088_ACCEL_REG_PWR_CONF, 0xff,
				BMI088_ACCEL_PWR_ACTIVE);
	if (ret)
		return ret;

	ret = bmi088_check_bits(dev, BMI088_DIE_ACCEL,
				BMI088_ACCEL_REG_RANGE,
				BMI088_ACCEL_RANGE_MASK,
				selected_config.accel_range);
	if (ret)
		return ret;

	ret = bmi088_check_bits(dev, BMI088_DIE_ACCEL,
				BMI088_ACCEL_REG_CONF,
				BMI088_ACCEL_CONF_MASK, accel_conf);
	if (ret)
		return ret;

	ret = bmi088_check_bits(dev, BMI088_DIE_GYRO,
				BMI088_GYRO_REG_RANGE,
				BMI088_GYRO_RANGE_MASK,
				selected_config.gyro_range);
	if (ret)
		return ret;

	ret = bmi088_check_bits(dev, BMI088_DIE_GYRO,
				BMI088_GYRO_REG_BANDWIDTH,
				BMI088_GYRO_BANDWIDTH_MASK,
				selected_config.gyro_bandwidth);
	if (ret)
		return ret;

	ret = bmi088_check_bits(dev, BMI088_DIE_GYRO,
				BMI088_GYRO_REG_INT_CTRL,
				BMI088_GYRO_DRDY_ENABLE,
				BMI088_GYRO_DRDY_ENABLE);
	if (ret)
		return ret;

	dev->initialized = 1;

	return 0;
}

int bmi088_read_accel_raw(struct imu_dev *imu, struct bmi088_vec3_raw *accel)
{
	struct bmi088 *dev = to_bmi088(imu);
	struct bmi088_vec3_raw value;
	uint8_t data[6];
	int ret;

	ret = bmi088_require_ready(dev, accel);
	if (ret)
		return ret;

	ret = dev->ops->read(dev->bus_context, BMI088_DIE_ACCEL,
			     BMI088_ACCEL_REG_DATA, data, sizeof(data));
	if (ret)
		return ret;

	bmi088_decode_vec3(data, &value);
	*accel = value;

	return 0;
}

int bmi088_read_gyro_raw(struct imu_dev *imu, struct bmi088_vec3_raw *gyro)
{
	struct bmi088 *dev = to_bmi088(imu);
	struct bmi088_vec3_raw value;
	uint8_t data[6];
	int ret;

	ret = bmi088_require_ready(dev, gyro);
	if (ret)
		return ret;

	ret = dev->ops->read(dev->bus_context, BMI088_DIE_GYRO,
			     BMI088_GYRO_REG_DATA, data, sizeof(data));
	if (ret)
		return ret;

	bmi088_decode_vec3(data, &value);
	*gyro = value;

	return 0;
}

int bmi088_read_accel(struct imu_dev *imu, struct bmi088_vec3 *accel_mps2)
{
	static const float full_scale_g[] = { 3.0f, 6.0f, 12.0f, 24.0f };
	struct bmi088 *dev = to_bmi088(imu);
	struct bmi088_vec3_raw raw;
	struct bmi088_vec3 value;
	float scale;
	int ret;

	ret = bmi088_require_ready(dev, accel_mps2);
	if (ret)
		return ret;

	ret = bmi088_read_accel_raw(imu, &raw);
	if (ret)
		return ret;

	scale = full_scale_g[dev->config.accel_range] * IMU_G / 32768.0f;
	value.x = raw.x * scale;
	value.y = raw.y * scale;
	value.z = raw.z * scale;
	*accel_mps2 = value;

	return 0;
}

int bmi088_read_gyro(struct imu_dev *imu, struct bmi088_vec3 *gyro_rads)
{
	static const float full_scale_dps[] = {
		2000.0f, 1000.0f, 500.0f, 250.0f, 125.0f
	};
	struct bmi088 *dev = to_bmi088(imu);
	struct bmi088_vec3_raw raw;
	struct bmi088_vec3 value;
	float scale;
	int ret;

	ret = bmi088_require_ready(dev, gyro_rads);
	if (ret)
		return ret;

	ret = bmi088_read_gyro_raw(imu, &raw);
	if (ret)
		return ret;

	scale = full_scale_dps[dev->config.gyro_range] * IMU_DEG_TO_RAD /
		32768.0f;
	value.x = raw.x * scale;
	value.y = raw.y * scale;
	value.z = raw.z * scale;
	*gyro_rads = value;

	return 0;
}

int bmi088_read_temperature(struct imu_dev *imu, float *temperature_c)
{
	struct bmi088 *dev = to_bmi088(imu);
	uint16_t unsigned_value;
	uint8_t data[2];
	int32_t signed_value;
	float value;
	int ret;

	ret = bmi088_require_ready(dev, temperature_c);
	if (ret)
		return ret;

	ret = dev->ops->read(dev->bus_context, BMI088_DIE_ACCEL,
			     BMI088_ACCEL_REG_TEMP, data, sizeof(data));
	if (ret)
		return ret;

	/* This reserved code marks invalid temperature data. */
	if (data[0] == 0x80 && data[1] == 0x00)
		return -ERANGE;

	unsigned_value = ((uint16_t)data[0] << 3) | (data[1] >> 5);
	signed_value = unsigned_value > 1023 ?
		       (int32_t)unsigned_value - 2048 : unsigned_value;
	value = 23.0f + signed_value * 0.125f;
	if (value < -104.0f || value > 150.0f)
		return -ERANGE;

	*temperature_c = value;

	return 0;
}

/* imu_ops.read: three transfers, already SI. */
static int bmi088_read(struct imu_dev *imu, struct imu_sample *s)
{
	struct bmi088_vec3 accel;
	struct bmi088_vec3 gyro;
	float temperature;
	int ret;

	ret = bmi088_read_accel(imu, &accel);
	if (ret)
		return ret;

	ret = bmi088_read_gyro(imu, &gyro);
	if (ret)
		return ret;

	ret = bmi088_read_temperature(imu, &temperature);
	if (ret)
		return ret;

	s->accel[0] = accel.x;
	s->accel[1] = accel.y;
	s->accel[2] = accel.z;
	s->gyro[0] = gyro.x;
	s->gyro[1] = gyro.y;
	s->gyro[2] = gyro.z;
	s->temp_c = temperature;

	return 0;
}

const struct imu_ops bmi088_imu_ops = {
	.read = bmi088_read,
};

int bmi088_data_ready(struct imu_dev *imu, uint8_t *ready_mask)
{
	struct bmi088 *dev = to_bmi088(imu);
	uint8_t gyro_status;
	uint8_t accel_status;
	uint8_t value = 0;
	int ret;

	ret = bmi088_require_ready(dev, ready_mask);
	if (ret)
		return ret;

	ret = bmi088_read_reg(dev, BMI088_DIE_ACCEL,
			      BMI088_ACCEL_REG_STATUS, &accel_status);
	if (ret)
		return ret;

	ret = bmi088_read_reg(dev, BMI088_DIE_GYRO,
			      BMI088_GYRO_REG_STATUS, &gyro_status);
	if (ret)
		return ret;

	if (accel_status & BMI088_ACCEL_STATUS_DRDY)
		value |= BMI088_DATA_READY_ACCEL;
	if (gyro_status & BMI088_GYRO_STATUS_DRDY)
		value |= BMI088_DATA_READY_GYRO;

	*ready_mask = value;

	return 0;
}

#endif /* CONFIG_IMU_BMI088 */
