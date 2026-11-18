/*
 * lis2dh12 -- ST LIS2DH12 3-axis accelerometer, as an imu chip driver.
 *
 * An accelerometer to 16 g with no gyroscope, a few microamps in its
 * low-power mode, over I2C (the part also has SPI; this driver does not).
 * Samples come out of imu_read(): acceleration in m/s^2; the gyroscope
 * reads NaN on all three axes, and so does the temperature, because the
 * part's sensor only reports changes, with no absolute offset.
 *
 * Besides samples, the driver sets up interrupt generator 1 on the INT1
 * pin as a motion detector: any chosen axis above a threshold, optionally
 * high-pass filtered so gravity does not count, for a number of samples.
 * Click, 6D, free-fall, the FIFO, INT2 and self-test are not here; reach
 * their registers with lis2dh12_reg_read() and lis2dh12_reg_write().
 *
 * Everything below that is LIS2DH12 specific takes the struct imu_dev the
 * probe filled, and returns -EINVAL if handed a device some other driver
 * probed. Register provenance is in src/lis2dh12/lis2dh12_regs.h: written
 * from memory of the datasheet and on the UNVERIFIED list of docs/chips.md.
 * Nothing in this driver has been run against a physical chip.
 */
#ifndef LIS2DH12_H
#define LIS2DH12_H

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

/* SDO/SA0 low or high (6.1.1). */
#define LIS2DH12_I2C_ADDR_LOW	0x18
#define LIS2DH12_I2C_ADDR_HIGH	0x19

/* Axes for the motion detector, as a mask. */
#define LIS2DH12_AXIS_X		(1u << 0)
#define LIS2DH12_AXIS_Y		(1u << 1)
#define LIS2DH12_AXIS_Z		(1u << 2)
#define LIS2DH12_AXIS_ALL	0x7u

/*
 * Operating mode: CTRL_REG1.LPen and CTRL_REG4.HR together (table
 * "Operating mode selection"). Resolution and current go up together.
 */
enum lis2dh12_mode {
	LIS2DH12_MODE_LOW_POWER = 0,	/* 8-bit */
	LIS2DH12_MODE_NORMAL	= 1,	/* 10-bit */
	LIS2DH12_MODE_HIGH_RES	= 2,	/* 12-bit */
};

/* CTRL_REG1.ODR (9.7, table "Data rate configuration"). */
enum lis2dh12_odr {
	LIS2DH12_ODR_POWER_DOWN	= 0x0,
	LIS2DH12_ODR_1HZ	= 0x1,
	LIS2DH12_ODR_10HZ	= 0x2,
	LIS2DH12_ODR_25HZ	= 0x3,
	LIS2DH12_ODR_50HZ	= 0x4,
	LIS2DH12_ODR_100HZ	= 0x5,
	LIS2DH12_ODR_200HZ	= 0x6,
	LIS2DH12_ODR_400HZ	= 0x7,
	LIS2DH12_ODR_1620HZ	= 0x8,	/* low-power mode only */
	LIS2DH12_ODR_FASTEST	= 0x9,	/* 1344 Hz; 5376 Hz in low power */
};

/* CTRL_REG4.FS (9.10). */
enum lis2dh12_full_scale {
	LIS2DH12_FS_2G	= 0,
	LIS2DH12_FS_4G	= 1,
	LIS2DH12_FS_8G	= 2,
	LIS2DH12_FS_16G	= 3,
};

struct lis2dh12_config {
	enum lis2dh12_mode mode;
	/* Not LIS2DH12_ODR_POWER_DOWN here: lis2dh12_set_mode() for that. */
	enum lis2dh12_odr odr;
	enum lis2dh12_full_scale full_scale;
	/*
	 * CTRL_REG2.HPCF, 0..3: the high-pass cut-off the motion detector's
	 * filter uses, a fraction of the ODR; 0 is the highest, about ODR/50,
	 * each step about half the one before (9.8).
	 */
	uint8_t high_pass_cutoff;
	/*
	 * CTRL_REG6.INT_POLARITY, for INT1 and INT2 alike: the pins are
	 * push-pull, so this only picks which level means "event".
	 */
	bool int_active_low;
	/*
	 * CTRL_REG0.SDO_PU_DISC: disconnect the SDO/SA0 pin's internal
	 * pull-up, which draws current when SA0 is tied low. Only for a board
	 * that ties SA0 to a rail: left floating, the address is undefined.
	 */
	bool sa0_pullup_off;
};

#ifdef ESP_PLATFORM
/*
 * Your application owns the I2C bus and creates it before probing; the
 * driver only adds and removes its own device.
 */
struct lis2dh12_i2c_config {
	i2c_master_bus_handle_t bus;
	/* 0 selects 0x18. */
	uint8_t address;
	/* 0 selects 400 kHz, the most the part takes. */
	uint32_t clock_hz;
	/* 0 selects 100 ms; a negative value is -EINVAL. */
	int timeout_ms;
};
#else
/*
 * Your application owns the bus and HAL_I2C_Init()s it, at 400 kHz or
 * less, before probing; the driver only keeps the handle.
 */
struct lis2dh12_i2c_config {
	I2C_HandleTypeDef *bus;
	/* 7-bit; 0 selects 0x18. */
	uint8_t address;
	/* 0 selects 100 ms; a negative value is -EINVAL. */
	int timeout_ms;
};
#endif

/*
 * Check WHO_AM_I (0x33), then write every control register to a known
 * state, whatever an earlier run left: temperature sensor, FIFO, click,
 * both interrupt generators and the sleep-to-wake function off, nothing
 * routed to the pins, block data update on; then the mode, rate and full
 * scale, the pin polarity and the SA0 pull-up as configured. A latched
 * INT1 left over from an earlier run is cleared. Fills imu->name, ops and
 * priv; leaves imu->mount alone.
 *
 * The first samples after the probe may still be settling: up to 7
 * samples' time in high-resolution mode, 1 otherwise (table "Turn-on
 * time"); lis2dh12_data_ready() says when one is new.
 *
 * -EBUSY if imu is already bound (see struct imu_dev). -EINVAL for a bad
 * bus or chip config. -ENXIO when nothing answers at the address; -ENODEV
 * when the wrong identity answers. A bus error as the port gives it.
 * Nothing stays bound after a failure.
 */
#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int lis2dh12_probe_i2c(struct imu_dev *imu,
		       const struct lis2dh12_i2c_config *bus_config,
		       const struct lis2dh12_config *config);
#endif

/* Power down and release the device. Safe on an unprobed one. */
void lis2dh12_remove(struct imu_dev *imu);

/*
 * The same burst as imu_read(), as counts at the mode's resolution:
 * -128..127 in low power, -512..511 in normal, -2048..2047 in high res.
 */
int lis2dh12_read_raw(struct imu_dev *imu, int16_t accel[3]);

/* STATUS_REG.ZYXDA: a new X, Y and Z sample since the last read (9.14). */
int lis2dh12_data_ready(struct imu_dev *imu, bool *ready);

/*
 * Mode, rate and full scale at any time; LIS2DH12_ODR_POWER_DOWN stops
 * the chip at about 0.5 uA. The LPen and HR bits are written in the order
 * that never sets both, which the part does not allow. If the motion
 * detector is on, its threshold is rewritten for the new full scale.
 */
int lis2dh12_set_mode(struct imu_dev *imu, enum lis2dh12_mode mode,
		      enum lis2dh12_odr odr,
		      enum lis2dh12_full_scale full_scale);

/*
 * The motion detector: interrupt generator 1, routed to INT1.
 *
 * Fires when any axis in `axes` is above threshold_g -- the generator
 * compares each axis on its own, not the vector's length -- for more than
 * `duration` samples in a row (0..127, each 1/ODR). With high_pass the
 * generator sees high-pass-filtered data (cut-off from the config), so a
 * still badge in any orientation reads about 0 g and only movement counts;
 * without it gravity is part of what it compares. Latched, the pin and
 * INT1_SRC hold the event until lis2dh12_read_motion(); not latched they
 * follow the condition.
 *
 * threshold_g is rounded to the full scale's step (16, 32, 62 or 186 mg)
 * and must land on 1..127 steps, else -EINVAL.
 */
struct lis2dh12_motion_config {
	uint8_t axes;		/* LIS2DH12_AXIS_*; 0 turns the detector off */
	float threshold_g;
	uint8_t duration;
	bool high_pass;
	bool latched;
};

int lis2dh12_set_motion(struct imu_dev *imu,
			const struct lis2dh12_motion_config *config);

/*
 * Read INT1_SRC, which clears a latched event and releases the pin.
 * *active is the generator's IA bit; *axes the LIS2DH12_AXIS_* that were
 * above the threshold. axes may be NULL.
 */
int lis2dh12_read_motion(struct imu_dev *imu, bool *active, uint8_t *axes);

/*
 * Counts per g at the current mode and full scale, the inverse of the
 * datasheet's mg/digit (table "Mechanical characteristics"); 0 if not ours.
 */
float lis2dh12_accel_sensitivity(struct imu_dev *imu);

/*
 * Escape hatch for the registers this driver does not model. A read of
 * more than one byte sets the I2C auto-increment bit itself.
 */
int lis2dh12_reg_read(struct imu_dev *imu, uint8_t reg, void *buf,
		      size_t len);
int lis2dh12_reg_write(struct imu_dev *imu, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_H */
