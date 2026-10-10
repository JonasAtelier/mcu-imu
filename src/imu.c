#include <errno.h>
#include <string.h>

#include "imu.h"

/* v = mount * v, via a copy so a row never reads already-rotated axes. */
static void imu_rotate(const struct imu_dev *imu, float v[3])
{
	float in[3] = { v[0], v[1], v[2] };
	int i;

	for (i = 0; i < 3; i++)
		v[i] = imu->mount[i][0] * in[0] + imu->mount[i][1] * in[1] +
		       imu->mount[i][2] * in[2];
}

int imu_read(struct imu_dev *imu, struct imu_sample *s)
{
	static const int8_t all_zero[3][3];
	struct imu_sample tmp;
	int64_t t;
	int ret;

	if (!imu || !imu->ops || !imu->ops->read || !s)
		return -EINVAL;

	/* Stamp before the bus transfer: that is closer to the sample. */
	t = imu_time_us();
	ret = imu->ops->read(imu, &tmp);
	if (ret)
		return ret;
	tmp.t_us = t;

	/* A zeroed imu_dev must just work, so all-zero means identity. */
	if (memcmp(imu->mount, all_zero, sizeof(all_zero))) {
		imu_rotate(imu, tmp.accel);
		imu_rotate(imu, tmp.gyro);
	}

	*s = tmp;
	return 0;
}
