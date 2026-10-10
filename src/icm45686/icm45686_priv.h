/* Internals of the ICM-45686 driver. Not public API. */
#ifndef ICM45686_PRIV_H
#define ICM45686_PRIV_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_priv.h"

struct icm45686 {
	struct imu_bus bus;
	/* register codes, so the maths header can index its tables with them */
	int accel_fs;
	int gyro_fs;
	int accel_odr;		/* ACCEL_CONFIG0 and GYRO_CONFIG0 each hold */
	int gyro_odr;		/* an ODR; each is cached with its register */
	bool big_endian;	/* data register layout, pinned at probe */
};

#endif /* ICM45686_PRIV_H */
