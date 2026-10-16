/*
 * bmi088 -- Bosch BMI088 6-axis IMU, as an imu chip driver.
 *
 * Two dies in one package: an accelerometer to 24 g and a gyroscope to
 * 2000 dps, each with its own I2C address or its own chip select. Over I2C or
 * SPI, hence two probes.
 *
 * Samples come out of imu_read(). Note that one sample is three transfers --
 * accelerometer, gyroscope, temperature -- so its halves are not from the
 * same instant the way a single-die part's are. An invalid temperature code
 * reads as NaN, and the rest of the sample stands.
 *
 * Everything below that is BMI088 specific takes the struct imu_dev a probe
 * filled, and returns -EINVAL if handed a device some other driver probed.
 */
#ifndef BMI088_H
#define BMI088_H

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#else
#include CMSIS_device_header
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_ACCEL_I2C_ADDR_LOW	0x18
#define BMI088_ACCEL_I2C_ADDR_HIGH	0x19
#define BMI088_GYRO_I2C_ADDR_LOW	0x68
#define BMI088_GYRO_I2C_ADDR_HIGH	0x69

#define BMI088_DATA_READY_ACCEL		(1U << 0)
#define BMI088_DATA_READY_GYRO		(1U << 1)

enum bmi088_accel_range {
	BMI088_ACCEL_RANGE_3G = 0,
	BMI088_ACCEL_RANGE_6G,
	BMI088_ACCEL_RANGE_12G,
	BMI088_ACCEL_RANGE_24G,
};

enum bmi088_accel_odr {
	BMI088_ACCEL_ODR_12_5_HZ = 0x05,
	BMI088_ACCEL_ODR_25_HZ,
	BMI088_ACCEL_ODR_50_HZ,
	BMI088_ACCEL_ODR_100_HZ,
	BMI088_ACCEL_ODR_200_HZ,
	BMI088_ACCEL_ODR_400_HZ,
	BMI088_ACCEL_ODR_800_HZ,
	BMI088_ACCEL_ODR_1600_HZ,
};

enum bmi088_accel_filter {
	BMI088_ACCEL_FILTER_OSR4 = 0x08,
	BMI088_ACCEL_FILTER_OSR2,
	BMI088_ACCEL_FILTER_NORMAL,
};

enum bmi088_gyro_range {
	BMI088_GYRO_RANGE_2000_DPS = 0,
	BMI088_GYRO_RANGE_1000_DPS,
	BMI088_GYRO_RANGE_500_DPS,
	BMI088_GYRO_RANGE_250_DPS,
	BMI088_GYRO_RANGE_125_DPS,
};

enum bmi088_gyro_bandwidth {
	BMI088_GYRO_ODR_2000_HZ_BW_532_HZ = 0,
	BMI088_GYRO_ODR_2000_HZ_BW_230_HZ,
	BMI088_GYRO_ODR_1000_HZ_BW_116_HZ,
	BMI088_GYRO_ODR_400_HZ_BW_47_HZ,
	BMI088_GYRO_ODR_200_HZ_BW_23_HZ,
	BMI088_GYRO_ODR_100_HZ_BW_12_HZ,
	BMI088_GYRO_ODR_200_HZ_BW_64_HZ,
	BMI088_GYRO_ODR_100_HZ_BW_32_HZ,
};

struct bmi088_config {
	enum bmi088_accel_range accel_range;
	enum bmi088_accel_odr accel_odr;
	enum bmi088_accel_filter accel_filter;
	enum bmi088_gyro_range gyro_range;
	enum bmi088_gyro_bandwidth gyro_bandwidth;
};

struct bmi088_vec3_raw {
	/* Raw axes are signed sensor counts. */
	int32_t x;
	int32_t y;
	int32_t z;
};

struct bmi088_vec3 {
	float x;
	float y;
	float z;
};

#ifdef ESP_PLATFORM
/*
 * Your application owns the I2C bus or SPI host and must initialise it before
 * probing; the driver only adds and removes its own two devices, so other
 * sensors can share the same bus. Both dies of the BMI088 are separate
 * devices, hence two addresses or two chip-select pins.
 */
struct bmi088_i2c_config {
	i2c_master_bus_handle_t bus;
	/* 0 selects the 0x18 accelerometer and 0x68 gyroscope defaults. */
	uint8_t accel_address;
	uint8_t gyro_address;
	/* 0 selects 400 kHz. */
	uint32_t clock_hz;
	/* 0 selects 100 ms. */
	int timeout_ms;
};

struct bmi088_spi_config {
	/* The host must already be initialised with spi_bus_initialize(). */
	spi_host_device_t host;
	int accel_cs_pin;
	int gyro_cs_pin;
	/* 0 selects 5 MHz. The BMI088 tolerates at most 10 MHz. */
	uint32_t clock_hz;
	/* 0 or 3; the BMI088 supports SPI mode 0 and mode 3 only. */
	uint8_t mode;
};
#else
/*
 * Your application owns the I2C or SPI bus and must HAL_*_Init() it before
 * probing; the driver only keeps the handle, so other sensors can share the
 * same bus. Both dies of the BMI088 are separate devices, hence two
 * addresses or two chip-select pins. Bus speed and SPI mode are the handle's.
 */
struct bmi088_i2c_config {
	/* At most 400 kHz. */
	I2C_HandleTypeDef *bus;
	/* 7-bit; 0 selects the 0x18 accelerometer and 0x68 gyroscope defaults. */
	uint8_t accel_address;
	uint8_t gyro_address;
	/* 0 selects 100 ms. */
	int timeout_ms;
};

struct bmi088_spi_config {
	/*
	 * Mode 0 or 3 with 8-bit frames, at most 10 MHz; the BMI088 supports
	 * SPI mode 0 and mode 3 only.
	 */
	SPI_HandleTypeDef *bus;
	/* Software chip selects: GPIO outputs the app set up, idle high. */
	GPIO_TypeDef *accel_cs_port;
	uint16_t accel_cs_pin;
	GPIO_TypeDef *gyro_cs_port;
	uint16_t gyro_cs_pin;
};
#endif

/* Defaults are +/-6 g at 100 Hz and +/-2000 dps at 200 Hz/23 Hz bandwidth. */
struct bmi088_config bmi088_default_config(void);

/*
 * Fill imu->name, ops and priv and bring both dies up; imu->mount is left
 * alone. config is copied; NULL selects the defaults. On failure nothing is
 * left allocated, and on ESP-IDF no device is left registered on the bus.
 *
 * -EBUSY if imu is already bound (see struct imu_dev); nothing is touched.
 * -ENODEV if a die answers with the wrong chip ID, -EIO if a register does
 * not read back what was written or the accelerometer flags an error, -ENXIO
 * if nothing ACKs on I2C, -EINVAL for a bad address, clock, SPI mode, CS or
 * config.
 */
int bmi088_probe_i2c(struct imu_dev *imu,
		     const struct bmi088_i2c_config *bus_config,
		     const struct bmi088_config *config);
int bmi088_probe_spi(struct imu_dev *imu,
		     const struct bmi088_spi_config *bus_config,
		     const struct bmi088_config *config);

/*
 * Releases this driver's state, and on ESP-IDF its two devices. The bus or
 * host itself is left alone.
 */
void bmi088_remove(struct imu_dev *imu);

/*
 * Single-die reads, for when you want only one sensor or raw counts. Outputs
 * are changed only when the full operation succeeds. Accel in m/s^2, gyro in
 * rad/s, both in the chip frame -- the mount matrix is imu_read()'s job.
 */
int bmi088_read_accel_raw(struct imu_dev *imu, struct bmi088_vec3_raw *accel);
int bmi088_read_gyro_raw(struct imu_dev *imu, struct bmi088_vec3_raw *gyro);
int bmi088_read_accel(struct imu_dev *imu, struct bmi088_vec3 *accel_mps2);
int bmi088_read_gyro(struct imu_dev *imu, struct bmi088_vec3 *gyro_rads);
/* NaN for the reserved invalid code; -ERANGE for an out-of-range value. */
int bmi088_read_temperature(struct imu_dev *imu, float *temperature_c);

/* BMI088_DATA_READY_ACCEL | BMI088_DATA_READY_GYRO, as each die reports. */
int bmi088_data_ready(struct imu_dev *imu, uint8_t *ready_mask);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_H */
