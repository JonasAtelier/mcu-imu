/* Internals of the LIS2DH12 driver. Not public API. */
#ifndef LIS2DH12_PRIV_H
#define LIS2DH12_PRIV_H

#include <stdint.h>

#include "imu_priv.h"
#include "lis2dh12.h"

struct lis2dh12 {
	struct imu_bus bus;
	/* register codes as last written, so the maths header can index
	 * its tables with them */
	int mode;
	int odr;
	int full_scale;
	uint8_t ctrl_reg2;			/* HPCF and HP_IA1 */
	struct lis2dh12_motion_config motion;	/* axes 0: detector off */
};

#endif /* LIS2DH12_PRIV_H */
