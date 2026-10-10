/*
 * wt901 -- WitMotion WT901C-TTL attitude sensor module, as an imu chip
 * driver, on its TTL serial link.
 *
 * Accelerometer, gyroscope and magnetometer behind a processor that fuses
 * them into roll, pitch and yaw. The module streams 11-byte frames on its
 * own; this driver reads that stream, checks every frame's SUM, and keeps
 * the accelerometer and gyroscope frames for imu_read().
 *
 * The document in docs/WT901.pdf is WitMotion's "WT901 Attitude Angle Sensor
 * SPECIFICATION", for the WT901 core. No C-TTL-specific manual was found;
 * the cased C-TTL is assumed to speak the same serial protocol. Every byte,
 * register and scale in src/wt901_proto.h and wt901.c cites its section and
 * page. The core also has an IIC mode (chapter 7); this driver does not
 * support it. Nothing here has been run against a physical module.
 *
 * Samples come out of imu_read(). Everything below that is WT901 specific
 * takes the struct imu_dev that wt901_probe() filled, and returns -EINVAL if
 * handed a device some other driver probed. Outputs are undefined when it
 * returns an error.
 */
#ifndef WT901_H
#define WT901_H

#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/uart.h"
#else
#include CMSIS_device_header
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* All zero, or a NULL pointer, selects the defaults. */
struct wt901_config {
	/*
	 * The longest wait for the stream, per read. 0 selects 200 ms: two
	 * output periods at the module's default 10 Hz (§6.2, p.35). Raise it
	 * if the module is set to a slower rate.
	 */
	uint32_t timeout_ms;
};

/*
 * uart is, on ESP-IDF, a port whose driver the app has already installed
 * and configured; on the STM32 HAL a handle already HAL_UART_Init()ed. The
 * link is the module's: 9600 bps out of the box, no parity (chapter 6,
 * p.30; §2 item 11, p.5).
 *
 * Nothing is written to the module: changing its output rate takes a save
 * and a power cycle (§6.2.9, p.39-40), and save writes its flash. Probe
 * waits for one valid accelerometer frame (0x51) and one gyroscope frame
 * (0x52), which the module sends by default (§6.2.8, p.38). Fills imu->name,
 * ops and priv; leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound (see struct imu_dev); nothing is touched.
 * -ENXIO if nothing arrives at all, decided before anything is allocated.
 * -EBADMSG if bytes arrive but no frame's SUM ever matches.
 * -ENODATA if valid frames arrive but never both 0x51 and 0x52: their
 * output is switched off in the module's return content.
 * -EINVAL for a NULL imu or handle, or on ESP-IDF a port with no driver
 * installed.
 */
int wt901_probe(struct imu_dev *imu,
#ifdef ESP_PLATFORM
		uart_port_t uart,
#else
		UART_HandleTypeDef *uart,
#endif
		const struct wt901_config *cfg);

/* Releases it. The module streams on; the UART is the app's. */
void wt901_remove(struct imu_dev *imu);

/*
 * The module's own roll, pitch and yaw, in degrees (§6.1.4, p.32): the next
 * 0x53 frame on the stream. -ENODATA if the angle output is switched off.
 */
int wt901_read_angles(struct imu_dev *imu, float rpy_deg[3]);

/*
 * Escape hatch: send 0xFF 0xAA address valueL valueH (§6.2, p.35-46) --
 * return content, rate, calibration, bias. Several only take effect after
 * a save (address 0x00) and a power cycle, as their sections say.
 */
int wt901_command(struct imu_dev *imu, uint8_t address, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* WT901_H */
