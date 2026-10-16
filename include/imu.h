/*
 * imu -- a small Linux-IIO-style IMU subsystem for ESP-IDF and the STM32 HAL.
 *
 * One struct imu_dev per sensor. A chip driver's probe fills in its name, its
 * ops and its private state; imu_read() does the rest: timestamp, the chip's
 * own read, then the mount rotation into the board frame. Code above this
 * header sees one shape of sample whichever chip produced it.
 *
 * Every function returns 0 or a negative errno.
 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct imu_sample {
	float accel[3];		/* m/s^2 */
	float gyro[3];		/* rad/s */
	float temp_c;		/* NaN if the chip marks it invalid */
	int64_t t_us;		/* imu_time_us() at read */
};

struct imu_dev;

struct imu_ops {
	/* chip frame, SI; must not touch s->t_us */
	int (*read)(struct imu_dev *imu, struct imu_sample *s);
};

/*
 * Must start zeroed -- static storage or `= { 0 }` -- because probe treats a
 * non-NULL ops or priv as "already bound" and returns -EBUSY rather than
 * leak the first binding. remove, or a failed probe, zeroes them again, so
 * the same imu_dev can be probed afresh.
 */
struct imu_dev {
	const char *name;		/* "mpu6050"; log prefix */
	const struct imu_ops *ops;
	void *priv;			/* chip state, owned by probe/remove */
	int8_t mount[3][3];		/* chip->board; all-zero = identity */
};

/*
 * Timestamp, ops->read, then board[i] = sum_j mount[i][j] * chip[j] for both
 * accel and gyro. Temperature is not rotated. *s is written only on success.
 */
int imu_read(struct imu_dev *imu, struct imu_sample *s);

/*
 * The clock imu_read() stamps samples with, in microseconds. On ESP-IDF it
 * is esp_timer_get_time(). On the STM32 HAL the default is weak and only
 * HAL_GetTick() * 1000: millisecond steps that wrap after 49.7 days. Define your own imu_time_us() for real microseconds -- the
 * DWT->CYCCNT cycle counter divided by the core clock in MHz, or a free
 * running 32-bit timer at 1 MHz extended to 64 bits.
 */
int64_t imu_time_us(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
