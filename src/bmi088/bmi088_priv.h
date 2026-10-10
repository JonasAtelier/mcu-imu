/*
 * Internals of the BMI088 driver. Not public API.
 *
 * The driver is two files. bmi088.c is the chip: register sequences, scaling,
 * the imu_ops. bmi088_bus.c is the transport and probe/remove, on one
 * struct imu_bus per die. They meet at struct bmi088_bus_ops, which is what
 * lets the chip half run against a mock bus on the host
 * (test_bmi088_core.c, in test/esp and test/stm).
 */
#ifndef BMI088_PRIV_H
#define BMI088_PRIV_H

#include <stddef.h>
#include <stdint.h>

#include "bmi088.h"
#include "imu_priv.h"

enum bmi088_die {
	BMI088_DIE_ACCEL,
	BMI088_DIE_GYRO,
};

struct bmi088_bus_ops {
	/*
	 * The caller owns this table and its context; both must outlive dev.
	 * read() fills length bytes from consecutive registers.
	 * Transfers return 0 on success or a negative errno.
	 * delay_us() must wait at least usec microseconds.
	 */
	int (*read)(void *context, enum bmi088_die die, uint8_t reg,
		    uint8_t *data, size_t length);
	int (*write)(void *context, enum bmi088_die die, uint8_t reg,
		     uint8_t value);
	void (*delay_us)(void *context, uint32_t usec);
};

struct bmi088 {
	const struct bmi088_bus_ops *ops;
	void *bus_context;
	struct bmi088_config config;
	uint8_t initialized;

	/*
	 * The transport, bmi088_bus.c only: the dies are separate devices --
	 * two I2C addresses or two chip selects -- so one bus each.
	 */
	struct {
		struct imu_bus accel;
		struct imu_bus gyro;
	} bus;
};

/* The imu_ops every BMI088 imu_dev carries; the wrong-chip check uses it. */
extern const struct imu_ops bmi088_imu_ops;

/*
 * Verify both chip IDs, reset both dies and apply config (NULL selects the
 * defaults). imu->priv must already point at the struct bmi088 to fill, and
 * imu->name is used for the log. The chip half of probe.
 */
int bmi088_init(struct imu_dev *imu, const struct bmi088_bus_ops *ops,
		void *bus_context, const struct bmi088_config *config);

#endif /* BMI088_PRIV_H */
