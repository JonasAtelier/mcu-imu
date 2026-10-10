/* Internals of the MPU-6050 driver. Not public API. */
#ifndef MPU6050_PRIV_H
#define MPU6050_PRIV_H

#include <stdint.h>

#include "imu_priv.h"

struct mpu6050 {
	struct imu_bus bus;
	int accel_fs;		/* register codes, so the maths header can */
	int gyro_fs;		/* index its tables with them directly */
	int dlpf;
	uint8_t smplrt_div;
};

#endif /* MPU6050_PRIV_H */
