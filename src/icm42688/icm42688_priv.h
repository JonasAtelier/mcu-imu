/* Internals of the ICM-42688-P driver. Not public API. */
#ifndef ICM42688_PRIV_H
#define ICM42688_PRIV_H

#include <stdint.h>

#include "imu_priv.h"

struct icm42688 {
	struct imu_bus bus;
	/* register codes, so the maths header can index its tables with them */
	int accel_fs;
	int gyro_fs;
	int odr;
	uint8_t pwr_mgmt0;	/* as last written */
};

#endif /* ICM42688_PRIV_H */
