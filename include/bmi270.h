/*
 * bmi270 -- Bosch BMI270 6-axis IMU, as an imu chip driver.
 *
 * A 3-axis accelerometer to 16 g and a 3-axis gyroscope to 2000 dps in one
 * die, over I2C or SPI, hence two probes. The chip measures nothing until
 * Bosch's 8 KB configuration file has been uploaded into it, at every
 * power-up and every soft reset; the probe does that, from the copy in
 * src/bmi270/bmi270_config_file.c (BSD-3-Clause, see NOTICE).
 *
 * The driver runs both sensors in the datasheet's normal or performance
 * mode (filter performance on), with the gyroscope's cross-axis correction
 * applied as the datasheet requires (section 4.6.10). Samples come out of
 * imu_read(): accelerometer, gyroscope and temperature from one burst. That
 * burst also reads, and so clears, the EVENT and INT_STATUS_0/1 registers:
 * with a latched data-ready interrupt, reading the sample releases the pin.
 * A temperature the chip marks invalid (0x8000) reads as NaN, and the rest
 * of the sample stands, as on the BMI088.
 *
 * Everything below that is BMI270 specific takes the struct imu_dev a probe
 * filled, and returns -EINVAL if handed a device some other driver probed.
 *
 * Register provenance is in src/bmi270/bmi270_regs.h. Nothing in this
 * driver has been run against a physical chip.
 */
#ifndef BMI270_H
#define BMI270_H

#include <stdbool.h>
#include <stddef.h>
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

/* SDO to GND or to VDDIO (datasheet 6.5). */
#define BMI270_I2C_ADDR_LOW	0x68
#define BMI270_I2C_ADDR_HIGH	0x69

/* bmi270_data_ready() flags: STATUS.drdy_acc, STATUS.drdy_gyr (5.2.3). */
#define BMI270_DATA_READY_ACCEL	(1U << 0)
#define BMI270_DATA_READY_GYRO	(1U << 1)

/* ACC_RANGE.acc_range (5.2.42). */
enum bmi270_accel_range {
	BMI270_ACCEL_RANGE_2G = 0,
	BMI270_ACCEL_RANGE_4G,
	BMI270_ACCEL_RANGE_8G,
	BMI270_ACCEL_RANGE_16G,
};

/*
 * ACC_CONF.acc_odr (5.2.41), the rates the filter-performance mode takes:
 * 12.5 Hz to 1.6 kHz (4.6.4).
 */
enum bmi270_accel_odr {
	BMI270_ACCEL_ODR_12_5_HZ = 0x05,
	BMI270_ACCEL_ODR_25_HZ,
	BMI270_ACCEL_ODR_50_HZ,
	BMI270_ACCEL_ODR_100_HZ,
	BMI270_ACCEL_ODR_200_HZ,
	BMI270_ACCEL_ODR_400_HZ,
	BMI270_ACCEL_ODR_800_HZ,
	BMI270_ACCEL_ODR_1600_HZ,
};

/* ACC_CONF.acc_bwp with acc_filter_perf = 1 (5.2.41). */
enum bmi270_accel_filter {
	BMI270_ACCEL_FILTER_OSR4 = 0,
	BMI270_ACCEL_FILTER_OSR2,
	BMI270_ACCEL_FILTER_NORMAL,
	BMI270_ACCEL_FILTER_CIC,
};

/* GYR_RANGE.gyr_range (5.2.44). */
enum bmi270_gyro_range {
	BMI270_GYRO_RANGE_2000_DPS = 0,
	BMI270_GYRO_RANGE_1000_DPS,
	BMI270_GYRO_RANGE_500_DPS,
	BMI270_GYRO_RANGE_250_DPS,
	BMI270_GYRO_RANGE_125_DPS,
};

/* GYR_CONF.gyr_odr (5.2.43): 25 Hz to 3.2 kHz (4.6.11). */
enum bmi270_gyro_odr {
	BMI270_GYRO_ODR_25_HZ = 0x06,
	BMI270_GYRO_ODR_50_HZ,
	BMI270_GYRO_ODR_100_HZ,
	BMI270_GYRO_ODR_200_HZ,
	BMI270_GYRO_ODR_400_HZ,
	BMI270_GYRO_ODR_800_HZ,
	BMI270_GYRO_ODR_1600_HZ,
	BMI270_GYRO_ODR_3200_HZ,
};

/* GYR_CONF.gyr_bwp (5.2.43). */
enum bmi270_gyro_filter {
	BMI270_GYRO_FILTER_OSR4 = 0,
	BMI270_GYRO_FILTER_OSR2,
	BMI270_GYRO_FILTER_NORMAL,
};

/*
 * The data-ready interrupt on the INT1 pin (4.9, INT1_IO_CTRL 5.2.58,
 * INT_LATCH 5.2.60, INT_MAP_DATA 5.2.63). Off leaves the pin disabled.
 * Latched holds the pin until INT_STATUS_1 is read, which every imu_read()
 * does; not latched, the chip clears it on its own after 1/6400 s (4.6.6).
 */
struct bmi270_int_config {
	bool enable;
	bool active_high;	/* else active low */
	bool open_drain;	/* else push-pull */
	bool latched;
};

struct bmi270_config {
	enum bmi270_accel_range accel_range;
	enum bmi270_accel_odr accel_odr;
	enum bmi270_accel_filter accel_filter;
	enum bmi270_gyro_range gyro_range;
	enum bmi270_gyro_odr gyro_odr;
	enum bmi270_gyro_filter gyro_filter;
	/* GYR_CONF.gyr_noise_perf: performance mode, less noise (Table 6) */
	bool gyro_low_noise;
	struct bmi270_int_config data_ready_int1;
};

#ifdef ESP_PLATFORM
/*
 * Your application owns the I2C bus or SPI host and initialises it before
 * probing; the driver only adds and removes its own device, so other
 * sensors can share the bus.
 */
struct bmi270_i2c_config {
	i2c_master_bus_handle_t bus;
	/* 0 selects 0x68. */
	uint8_t address;
	/* 0 selects 400 kHz; at most 1 MHz, Fast-mode Plus (6.5). */
	uint32_t clock_hz;
	/* 0 selects 100 ms; a negative value is -EINVAL. */
	int timeout_ms;
};

struct bmi270_spi_config {
	/* The host must already be initialised with spi_bus_initialize(). */
	spi_host_device_t host;
	int cs_pin;
	/* 0 selects 5 MHz; at most 10 MHz (Table 19). */
	uint32_t clock_hz;
	/* 0 or 3, the two modes the BMI270 takes (6.4). */
	uint8_t mode;
};
#else
/*
 * Your application owns the bus and HAL_*_Init()s it before probing; the
 * driver only keeps the handle. Bus speed and SPI mode are the handle's.
 */
struct bmi270_i2c_config {
	/* At most 1 MHz (6.5). */
	I2C_HandleTypeDef *bus;
	/* 7-bit; 0 selects 0x68. */
	uint8_t address;
	/* 0 selects 100 ms; a negative value is -EINVAL. */
	int timeout_ms;
};

struct bmi270_spi_config {
	/* Mode 0 or 3 with 8-bit frames, at most 10 MHz (6.4, Table 19). */
	SPI_HandleTypeDef *bus;
	/* A GPIO output the app has set up, idle high. */
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
};
#endif

/*
 * Check CHIP_ID (0x24), soft-reset, upload the configuration file, wait for
 * INTERNAL_STATUS to report it loaded, read the gyroscope's cross-axis
 * factor, then apply config -- NULL selects the chip's reset values: 8 g at
 * 100 Hz and 2000 dps at 200 Hz, both filters normal (5.2.41-5.2.44) -- and
 * start both sensors and the temperature sensor. Fills imu->name, ops and
 * priv; leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound (see struct imu_dev). -EINVAL for a bad
 * bus or chip config. -ENXIO when nothing answers at the I2C address;
 * -ENODEV when the wrong identity answers. -ETIMEDOUT when the configuration
 * never reports loaded, -EPROTO when the chip reports it failed. A bus
 * error as the port gives it. Nothing stays bound after a failure.
 *
 * The upload is 8 KB in 30-byte pieces, two writes each with a 2 us idle
 * after it: on the STM32 HAL, whose shortest wait is 1 ms, the probe takes
 * about 0.6 s.
 */
#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int bmi270_probe_i2c(struct imu_dev *imu,
		     const struct bmi270_i2c_config *bus_config,
		     const struct bmi270_config *config);
#endif
#if defined(ESP_PLATFORM) || defined(HAL_SPI_MODULE_ENABLED)
int bmi270_probe_spi(struct imu_dev *imu,
		     const struct bmi270_spi_config *bus_config,
		     const struct bmi270_config *config);
#endif

/* Turns every sensor off and releases the device. Safe on an unprobed one. */
void bmi270_remove(struct imu_dev *imu);

/*
 * The same burst as imu_read(), as counts: accel and gyro (gyro X already
 * corrected for cross-axis sensitivity, 4.6.10), and the temperature word,
 * which may be 0x8000, invalid. temp may be NULL.
 */
int bmi270_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		    int16_t *temp);

/*
 * STATUS.drdy_acc and drdy_gyr as BMI270_DATA_READY_* flags. Each clears
 * when its sensor's data is read (5.2.3).
 */
int bmi270_data_ready(struct imu_dev *imu, uint8_t *flags);

/* Change ranges or rates after probe (see section 4.6.16 for when). */
int bmi270_set_ranges(struct imu_dev *imu,
		      enum bmi270_accel_range accel_range,
		      enum bmi270_gyro_range gyro_range);
int bmi270_set_odr(struct imu_dev *imu, enum bmi270_accel_odr accel_odr,
		   enum bmi270_gyro_odr gyro_odr);

/* Counts per g and per dps for the ranges set; 0 if not ours. */
float bmi270_accel_sensitivity(struct imu_dev *imu);
float bmi270_gyro_sensitivity(struct imu_dev *imu);

/*
 * Escape hatch for the registers this driver does not model: the FIFO, the
 * feature engine, the auxiliary interface, self-test. len is at most 24.
 */
int bmi270_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len);
int bmi270_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_H */
