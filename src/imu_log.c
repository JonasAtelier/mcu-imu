#include <stdint.h>

#include "imu_log.h"

/*
 * Decimal text of v, written backwards from end. newlib-nano's printf, the
 * one stm-forge links, has no %lld, so the timestamp cannot go through PRId64.
 */
static const char *imu_format_i64(char *end, int64_t v)
{
	uint64_t u = v < 0 ? 0 - (uint64_t)v : (uint64_t)v;

	*--end = '\0';
	do {
		*--end = (char)('0' + u % 10);
		u /= 10;
	} while (u);
	if (v < 0)
		*--end = '-';
	return end;
}

void imu_log_sample(const struct imu_dev *imu, const struct imu_sample *s)
{
	char t[21];	/* "-9223372036854775808" and its NUL */

	imu_info(imu, "t %s us  a % 7.3f % 7.3f % 7.3f m/s^2  "
		 "g % 7.3f % 7.3f % 7.3f rad/s  %5.1f C",
		 imu_format_i64(t + sizeof(t), s->t_us),
		 (double)s->accel[0], (double)s->accel[1], (double)s->accel[2],
		 (double)s->gyro[0], (double)s->gyro[1], (double)s->gyro[2],
		 (double)s->temp_c);
}
