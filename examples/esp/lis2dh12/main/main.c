/*
 * LIS2DH12: stream acceleration through the imu core. The part is an
 * accelerometer only, so every sample's gyroscope and temperature are NaN,
 * as lis2dh12.h promises. Its motion detector on INT1 stays off here;
 * lis2dh12_set_motion() turns it on.
 *
 * Wiring: CS to 3.3 V selects I2C, SDO/SA0 to GND gives address 0x18, and
 * SDA and SCL want 4.7 k pull-ups (docs/hardware.md). The bare chip is
 * 3.3 V only.
 */
#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
#include "imu_log.h"
#include "lis2dh12.h"

#define PIN_SDA 21
#define PIN_SCL 22

void app_main(void)
{
	i2c_master_bus_config_t buscfg = {
		.i2c_port = -1,
		.sda_io_num = PIN_SDA,
		.scl_io_num = PIN_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	/* Zero address, clock and timeout: 0x18, 400 kHz, 100 ms. */
	struct lis2dh12_i2c_config chip_bus = { 0 };
	/* 12-bit samples at 100 Hz, +-2 g. */
	const struct lis2dh12_config cfg = {
		.mode = LIS2DH12_MODE_HIGH_RES,
		.odr = LIS2DH12_ODR_100HZ,
		.full_scale = LIS2DH12_FS_2G,
	};
	/* mount all zero: chip axes are board axes */
	struct imu_dev imu = { 0 };

	ESP_ERROR_CHECK(i2c_new_master_bus(&buscfg, &chip_bus.bus));

	/* probe already logged why, under the "imu" tag */
	if (lis2dh12_probe_i2c(&imu, &chip_bus, &cfg))
		return;

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (lis2dh12_data_ready(&imu, &ready) || !ready) {
			vTaskDelay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		/* gyro[] and temp_c print as nan: the part has neither. */
		imu_log_sample(&imu, &s);
	}
}
