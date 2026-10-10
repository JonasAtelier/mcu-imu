/*
 * Kernel dev_err()-style logging for IMU drivers and their users.
 *
 * ESP-IDF: everything goes out under the one ESP-IDF tag "imu", prefixed with
 * the device's name, so one esp_log_level_set("imu", ...) controls every chip
 * and a line still says which chip it came from. There is deliberately no
 * level machinery of our own; imu_dbg() is also subject to
 * CONFIG_LOG_MAXIMUM_LEVEL, and CONFIG_IMU_DEBUG is not used.
 *
 * STM32 HAL: every line goes out through printf() as "imu: <name>: ...", so a
 * line still says which chip it came from. Where printf() ends up is the
 * app's business: with newlib, retarget _write() to a UART. There is
 * deliberately no level machinery; imu_dbg() alone is compiled out unless
 * CONFIG_IMU_DEBUG is 1 (stm-forge: IMU_DEBUG = y).
 */
#ifndef IMU_LOG_H
#define IMU_LOG_H

#ifdef ESP_PLATFORM
#include "esp_log.h"
#else
#include <stdio.h>
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef ESP_PLATFORM
#define IMU_LOG_TAG	"imu"

#define imu_err(imu, fmt, ...) \
	ESP_LOGE(IMU_LOG_TAG, "%s: " fmt, (imu)->name, ##__VA_ARGS__)
#define imu_warn(imu, fmt, ...) \
	ESP_LOGW(IMU_LOG_TAG, "%s: " fmt, (imu)->name, ##__VA_ARGS__)
#define imu_info(imu, fmt, ...) \
	ESP_LOGI(IMU_LOG_TAG, "%s: " fmt, (imu)->name, ##__VA_ARGS__)
#define imu_dbg(imu, fmt, ...) \
	ESP_LOGD(IMU_LOG_TAG, "%s: " fmt, (imu)->name, ##__VA_ARGS__)

#else /* STM32 HAL */
#define imu_err(imu, fmt, ...) \
	printf("imu: %s: " fmt "\n", (imu)->name, ##__VA_ARGS__)
#define imu_warn(imu, fmt, ...) \
	printf("imu: %s: " fmt "\n", (imu)->name, ##__VA_ARGS__)
#define imu_info(imu, fmt, ...) \
	printf("imu: %s: " fmt "\n", (imu)->name, ##__VA_ARGS__)

#if CONFIG_IMU_DEBUG
#define imu_dbg(imu, fmt, ...) \
	printf("imu: %s: " fmt "\n", (imu)->name, ##__VA_ARGS__)
#else
/* Still type-checked, so a debug line cannot rot while it is off. */
#define imu_dbg(imu, fmt, ...) do { \
	if (0) \
		printf("imu: %s: " fmt "\n", (imu)->name, ##__VA_ARGS__); \
} while (0)
#endif
#endif /* ESP_PLATFORM */

/* One info line: timestamp, accel xyz, gyro xyz, temperature. */
void imu_log_sample(const struct imu_dev *imu, const struct imu_sample *s);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LOG_H */
