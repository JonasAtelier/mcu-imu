/*
 * icm45686 -- TDK InvenSense ICM-45686 6-axis IMU, as an imu chip driver.
 *
 * A 3-axis gyroscope to 4000 dps and a 3-axis accelerometer to 32 g, sampled
 * together up to 6.4 kHz. Read six numbers and a temperature; that is the
 * whole surface.
 *
 * SPI only. The part also speaks I2C and I3C; neither is implemented here,
 * because one transport with no indirection reads better than three behind an
 * ops table -- and at 6.4 kHz you want SPI anyway.
 *
 * Samples come out of imu_read(). Everything below that is ICM-45686 specific
 * takes the struct imu_dev that icm45686_probe() filled, and returns -EINVAL
 * if handed a device some other driver probed.
 *
 * Register provenance is documented in src/icm45686_regs.h. Nothing in this
 * driver has been run against a physical chip.
 */
#ifndef ICM45686_H
#define ICM45686_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/gpio.h"
#include "driver/spi_master.h"
#else
#include CMSIS_device_header
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Full-scale codes. Note these run backwards -- 0 is the *widest* range on
 * both sensors. The names carry the value so you never have to remember which
 * way round it goes.
 */
enum icm45686_accel_fs {
	ICM45686_ACCEL_FS_32G	= 0,
	ICM45686_ACCEL_FS_16G	= 1,
	ICM45686_ACCEL_FS_8G	= 2,
	ICM45686_ACCEL_FS_4G	= 3,
	ICM45686_ACCEL_FS_2G	= 4,
};

enum icm45686_gyro_fs {
	ICM45686_GYRO_FS_4000DPS	= 0,
	ICM45686_GYRO_FS_2000DPS	= 1,
	ICM45686_GYRO_FS_1000DPS	= 2,
	ICM45686_GYRO_FS_500DPS		= 3,
	ICM45686_GYRO_FS_250DPS		= 4,
	ICM45686_GYRO_FS_125DPS		= 5,
	ICM45686_GYRO_FS_62_5DPS	= 6,
	ICM45686_GYRO_FS_31_25DPS	= 7,
	ICM45686_GYRO_FS_15_625DPS	= 8,
};

enum icm45686_odr {
	ICM45686_ODR_6400HZ	= 0x3,
	ICM45686_ODR_3200HZ	= 0x4,
	ICM45686_ODR_1600HZ	= 0x5,
	ICM45686_ODR_800HZ	= 0x6,
	ICM45686_ODR_400HZ	= 0x7,
	ICM45686_ODR_200HZ	= 0x8,
	ICM45686_ODR_100HZ	= 0x9,
	ICM45686_ODR_50HZ	= 0xA,
	ICM45686_ODR_25HZ	= 0xB,
	ICM45686_ODR_12_5HZ	= 0xC,
	ICM45686_ODR_6_25HZ	= 0xD,
	ICM45686_ODR_3_125HZ	= 0xE,
	ICM45686_ODR_1_5625HZ	= 0xF,
};

/*
 * The app owns the SPI bus. On ESP-IDF it is a host already initialised, to
 * which the probe adds the chip in mode 3. On the STM32 HAL, HAL_SPI_Init()
 * it before probing, as a master in mode 3 (CPOL high, CPHA second edge) with
 * 8-bit frames, at 24 MHz or less; chip select is driven in software, so
 * cs_port/cs_pin is an ordinary GPIO output, set up by the app and idle high.
 */
struct icm45686_config {
#ifdef ESP_PLATFORM
	spi_host_device_t host;		/* an SPI bus you already initialised */
	gpio_num_t cs_gpio;
	int spi_hz;			/* 0 selects 8 MHz; the part takes 24 */
#else
	SPI_HandleTypeDef *spi;
	GPIO_TypeDef *cs_port;		/* GPIOA ... */
	uint16_t cs_pin;		/* GPIO_PIN_0 ... */
#endif

	enum icm45686_accel_fs accel_fs;
	enum icm45686_gyro_fs gyro_fs;
	enum icm45686_odr odr;		/* applied to both sensors */
};

/*
 * Reset the device, verify WHO_AM_I reads 0xE9, pin the data byte order,
 * apply the ranges and rate, and leave both sensors running in low-noise
 * mode. Fills imu->name, ops and priv; leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound (see struct imu_dev); nothing is touched.
 * -ENODEV if the wrong identity answers -- usually miswired SPI or the wrong
 * CS, since SPI has no ACK to fail on.
 * -EINVAL for a full-scale or rate code outside the enums above; on the HAL
 * also a missing handle or CS, or an SPI handle not in mode 3 with 8-bit
 * frames.
 */
int icm45686_probe(struct imu_dev *imu, const struct icm45686_config *cfg);

/* Powers both sensors off and releases the device. Safe on an unprobed one. */
void icm45686_remove(struct imu_dev *imu);

/*
 * One burst read of all fourteen data bytes, as counts, for when you want to
 * log counts or do your own maths. temp may be NULL.
 */
int icm45686_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		      int16_t *temp);

/* True once a new sample is waiting. Reading INT1_STATUS0 clears it. */
int icm45686_data_ready(struct imu_dev *imu, bool *ready);

/* Change ranges or rate after probe. Both take effect on the next sample. */
int icm45686_set_ranges(struct imu_dev *imu, enum icm45686_accel_fs accel_fs,
			enum icm45686_gyro_fs gyro_fs);
int icm45686_set_odr(struct imu_dev *imu, enum icm45686_odr odr);

/*
 * Turn either sensor off to save power. Both off is the lowest state this
 * driver offers; the gyro needs time to spin up again after being off.
 */
int icm45686_set_power(struct imu_dev *imu, bool accel_on, bool gyro_on);

/*
 * Counts per g and per degree per second for the ranges currently set -- what
 * you need to scale icm45686_read_raw() counts yourself. 0 if not ours.
 */
float icm45686_accel_sensitivity(struct imu_dev *imu);
float icm45686_gyro_sensitivity(struct imu_dev *imu);

/*
 * Escape hatch for the registers this driver does not model: the FIFO, the
 * APEX motion engine, the AUX interface, self-test.
 */
int icm45686_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len);
int icm45686_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_H */
