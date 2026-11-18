/*
 * jy901 -- WitMotion JY901 attitude sensor module, as an imu chip driver,
 * on its TTL serial link or on I2C.
 *
 * Accelerometer, gyroscope and magnetometer behind a processor that fuses
 * them into roll, pitch and yaw. On the UART the module streams 11-byte
 * frames on its own, and this driver reads that stream, checking every
 * frame's SUM; on I2C it is a register file, and imu_read() is one burst.
 *
 * The document in docs/WT901.pdf is WitMotion's "WT901 Attitude Angle Sensor
 * SPECIFICATION", which names the module JY-901 in its IIC sections (§5.2,
 * chapter 7). No JY901-specific manual was found. Every byte, register and
 * scale in src/jy901/jy901_regs.h and jy901.c cites its section and page,
 * and docs/chips.md lists what had to be assumed. Nothing here has been run
 * against a physical module.
 *
 * Samples come out of imu_read(). Everything below that is JY901 specific
 * takes the struct imu_dev a jy901 probe filled, and returns -EINVAL if
 * handed a device some other driver probed. Outputs are undefined when it
 * returns an error.
 */
#ifndef JY901_H
#define JY901_H

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/uart.h"
#else
#include CMSIS_device_header
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* All zero, or a NULL pointer, selects the defaults. */
struct jy901_uart_config {
	/*
	 * The longest wait for the stream, per read. 0 selects 200 ms: two
	 * output periods at the module's default 10 Hz (§6.2, p.35). Raise it
	 * if the module is set to a slower rate. A negative value is -EINVAL.
	 */
	int timeout_ms;
};

#ifdef ESP_PLATFORM
/*
 * Your application owns the I2C bus and initialises it before probing; the
 * driver only adds and removes its own device. The module's bus is open
 * drain: fit 4.7 k pull-ups (§5.2, p.26).
 */
struct jy901_i2c_config {
	i2c_master_bus_handle_t bus;
	/* 7-bit; 0 selects 0x50 (§6.2.32, p.45). */
	uint8_t address;
	/* 0 selects 400 kHz, which is also the most (chapter 7, p.46). */
	uint32_t clock_hz;
	/* 0 selects 100 ms; a negative value is -EINVAL. */
	int timeout_ms;
};
#elif defined(HAL_I2C_MODULE_ENABLED)
/*
 * Your application owns the bus and HAL_I2C_Init()s it before probing, at
 * most 400 kHz (chapter 7, p.46), with 4.7 k pull-ups (§5.2, p.26).
 */
struct jy901_i2c_config {
	I2C_HandleTypeDef *bus;
	/* 7-bit; 0 selects 0x50 (§6.2.32, p.45). */
	uint8_t address;
	/* 0 selects 100 ms; a negative value is -EINVAL. */
	int timeout_ms;
};
#endif

/*
 * uart is, on ESP-IDF, a port whose driver the app has already installed
 * and configured; on the STM32 HAL a handle already HAL_UART_Init()ed. The
 * link is the module's: 9600 bps out of the box, no parity (chapter 6,
 * p.30; §2 item 11, p.5).
 *
 * Nothing is written to the module. Probe waits for one valid
 * accelerometer frame (0x51) and one gyroscope frame (0x52), which the
 * module sends by default (§6.2.8, p.38). Fills imu->name, ops and priv;
 * leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound (see struct imu_dev); nothing is touched.
 * -ENXIO if nothing arrives at all, decided before anything is allocated.
 * -EBADMSG if bytes arrive but no frame's SUM ever matches.
 * -ENODATA if valid frames arrive but never both 0x51 and 0x52: their
 * output is switched off in the module's return content.
 * -EINVAL for a NULL imu or handle, a negative timeout, or on ESP-IDF a
 * port with no driver installed.
 */
#if defined(ESP_PLATFORM) || defined(HAL_UART_MODULE_ENABLED)
int jy901_probe_uart(struct imu_dev *imu,
#ifdef ESP_PLATFORM
		     uart_port_t uart,
#else
		     UART_HandleTypeDef *uart,
#endif
		     const struct jy901_uart_config *config);
#endif

/*
 * Nothing is written to the module. Probe reads IICADDR (0x1a, §6.2.32)
 * and requires it to hold the address it was reached at, which tells the
 * module from another part at 0x50 such as an EEPROM; then one burst. Fills
 * imu->name, ops and priv; leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound. -EINVAL for a NULL imu, config or bus, an
 * address above 0x7f, a negative timeout, or on ESP-IDF a clock above
 * 400 kHz. -ENXIO when
 * nothing answers at the address, -ENODEV when IICADDR does not match, both
 * decided before anything is allocated. A bus error as the port gives it.
 */
#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int jy901_probe_i2c(struct imu_dev *imu,
		    const struct jy901_i2c_config *bus_config);
#endif

/* Releases it. The module runs on; the bus is the app's. */
void jy901_remove(struct imu_dev *imu);

/*
 * The module's own roll, pitch and yaw, in degrees (§6.1.4, p.32): on the
 * UART the next 0x53 frame, -ENODATA if the angle output is switched off;
 * on I2C the Roll, Pitch and Yaw registers (0x3d, §7.2 p.49).
 */
int jy901_read_angles(struct imu_dev *imu, float roll_pitch_yaw_deg[3]);

/*
 * Escape hatch: write value to register address -- return content, rate,
 * calibration, bias (§6.2, p.35-46). On the UART that is 0xFF 0xAA address
 * valueL valueH (§6.2 reminder 2, p.35); on I2C a register write, valueL
 * then valueH (§7.1, p.48). Several only take effect after a save (address
 * 0x00) and a power cycle, as their sections say.
 */
int jy901_command(struct imu_dev *imu, uint8_t address, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* JY901_H */
