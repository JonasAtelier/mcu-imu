/* Internals of the BMI270 driver. Not public API. */
#ifndef BMI270_PRIV_H
#define BMI270_PRIV_H

#include <stdbool.h>
#include <stdint.h>

#include "bmi270_regs.h"
#include "imu_priv.h"

struct bmi270 {
	struct imu_bus bus;
	/* register codes, so the maths header can index its tables with them */
	int accel_range;
	int accel_odr;
	int gyro_range;
	int gyro_odr;
	uint8_t acc_conf;	/* ACC_CONF and GYR_CONF as written, so a */
	uint8_t gyr_conf;	/* rate change keeps the filter bits */
	int8_t cas_factor;	/* GYR_CAS.factor_zx, read at probe */
};

/*
 * Bosch's configuration file, from its BMI270 Sensor API; provenance and
 * licence in bmi270_config_file.c.
 */
extern const uint8_t bmi270_config_data[BMI270_CONFIG_SIZE];

#endif /* BMI270_PRIV_H */
