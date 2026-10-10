/*
 * mpu6050 -- InvenSense MPU-6050 6-axis IMU, as an imu chip driver.
 *
 * The one on every GY-521. A 3-axis gyroscope to 2000 dps and a 3-axis
 * accelerometer to 16 g, over I2C.
 *
 * I2C only, because the part is I2C only -- the SPI sibling is the MPU-6000,
 * a different order code. The auxiliary bus is exposed just far enough to put
 * the chip in bypass so you can talk to a magnetometer sitting behind it.
 *
 * The DMP is not here. Its firmware blob is undocumented, and a driver that
 * ships one cannot tell you what it does.
 *
 * Samples come out of imu_read(). Everything below that is MPU-6050 specific
 * takes the struct imu_dev that mpu6050_probe() filled, and returns -EINVAL
 * if handed a device some other driver probed.
 *
 * Register provenance is documented in src/mpu6050/mpu6050_regs.h. Nothing
 * in this driver has been run against a physical chip.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#else
#include CMSIS_device_header
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

enum mpu6050_accel_fs {
	MPU6050_ACCEL_FS_2G	= 0,
	MPU6050_ACCEL_FS_4G	= 1,
	MPU6050_ACCEL_FS_8G	= 2,
	MPU6050_ACCEL_FS_16G	= 3,
};

enum mpu6050_gyro_fs {
	MPU6050_GYRO_FS_250DPS	= 0,
	MPU6050_GYRO_FS_500DPS	= 1,
	MPU6050_GYRO_FS_1000DPS	= 2,
	MPU6050_GYRO_FS_2000DPS	= 3,
};

/*
 * Digital low-pass filter. This also decides the gyro output rate that
 * SMPLRT_DIV divides: 8 kHz at OFF and RESERVED, 1 kHz at every other
 * setting. Changing the filter therefore changes the sample rate, which is
 * why sample_rate_hz below is re-solved whenever the filter moves.
 */
enum mpu6050_dlpf {
	/* 260 Hz accel, 256 Hz gyro, 8 kHz base */
	MPU6050_DLPF_OFF	= 0,
	MPU6050_DLPF_184HZ	= 1,
	MPU6050_DLPF_94HZ	= 2,
	MPU6050_DLPF_44HZ	= 3,
	MPU6050_DLPF_21HZ	= 4,
	MPU6050_DLPF_10HZ	= 5,
	MPU6050_DLPF_5HZ	= 6,
};

struct mpu6050_config {
#ifdef ESP_PLATFORM
	uint8_t address;		/* 0 selects 0x68, AD0 low */
	uint32_t scl_hz;		/* 0 selects 400 kHz */
#else
	uint8_t address;		/* 7-bit; 0 selects 0x68, AD0 low */
#endif

	enum mpu6050_accel_fs accel_fs;
	enum mpu6050_gyro_fs gyro_fs;
	enum mpu6050_dlpf dlpf;
	float sample_rate_hz;		/* 0 selects 100 Hz */
};

/*
 * bus is, on ESP-IDF, an i2c_master bus the app has already created; the
 * driver adds its device at cfg->scl_hz. On the STM32 HAL it is an I2C handle
 * the app has already set up with HAL_I2C_Init(), at 400 kHz or less; the bus
 * speed is the handle's, not this driver's.
 *
 * Reset the device, verify WHO_AM_I reads 0x68, wake it, put it on the gyro
 * PLL clock, then apply the filter, rate and ranges. Fills imu->name, ops and
 * priv; leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound (see struct imu_dev); nothing is touched.
 * -ENXIO if nothing ACKs at the address, decided before anything is allocated.
 * -ENODEV if something answers but is not an MPU-6050.
 * -EINVAL for a range or filter outside the enums, an address other than
 * 0x68 or 0x69, or a sample rate the divider cannot reach.
 * Bus failures come back as the errno of the failed transfer.
 */
int mpu6050_probe(struct imu_dev *imu,
#ifdef ESP_PLATFORM
		  i2c_master_bus_handle_t bus,
#else
		  I2C_HandleTypeDef *bus,
#endif
		  const struct mpu6050_config *cfg);

/* Puts the chip to sleep and releases it. Safe on an unprobed imu_dev. */
void mpu6050_remove(struct imu_dev *imu);

/* One burst of all fourteen data bytes, as counts. temp may be NULL. */
int mpu6050_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		     int16_t *temp);

/* True once a new sample is waiting. Reading INT_STATUS clears it. */
int mpu6050_data_ready(struct imu_dev *imu, bool *ready);

int mpu6050_set_ranges(struct imu_dev *imu, enum mpu6050_accel_fs accel_fs,
		       enum mpu6050_gyro_fs gyro_fs);

/*
 * Set the filter and rate together, because the filter moves the base rate
 * the divider works from. Returns -EINVAL if the wanted rate is unreachable
 * with that filter -- see mpu6050_sample_rate().
 */
int mpu6050_set_rate(struct imu_dev *imu, enum mpu6050_dlpf dlpf,
		     float sample_rate_hz);

/*
 * The rate actually programmed, which is the closest the 8-bit divider could
 * get to what you asked for. 0 for a device this driver did not probe.
 */
float mpu6050_sample_rate(struct imu_dev *imu);

/* Sleep saves about 5 mA. Waking takes a moment before samples are good. */
int mpu6050_sleep(struct imu_dev *imu, bool sleep);

/*
 * Connect the auxiliary bus straight through to the main one, so a device
 * behind the MPU-6050 -- typically an HMC5883L or AK8975 on a 9-axis board --
 * appears as an ordinary node on your own I2C bus at its own address.
 *
 * Only works while the internal I2C master is off, which is how this driver
 * leaves it. The magnetometer is then yours to drive; this driver does not
 * touch it.
 */
int mpu6050_i2c_bypass(struct imu_dev *imu, bool enable);

/* Counts per g and per dps for the ranges currently set; 0 if not ours. */
float mpu6050_accel_sensitivity(struct imu_dev *imu);
float mpu6050_gyro_sensitivity(struct imu_dev *imu);

/* Escape hatch for the registers this driver does not model. */
int mpu6050_reg_read(struct imu_dev *imu, uint8_t reg, void *buf, size_t len);
int mpu6050_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
