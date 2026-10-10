/*
 * MPU-6050: stream accelerometer, gyroscope and die temperature through the
 * imu core.
 *
 * Wiring: any I2C bus. A GY-521 breakout carries its own regulator and level
 * shifters, so 3.3 V or 5 V both work on that board -- the bare chip is 3.3 V
 * only.
 */
#include <stdbool.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
#include "imu_log.h"
#include "mpu6050.h"

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
	/*
	 * 4 g and 500 dps suit a hand-held board. The 44 Hz filter puts the
	 * base rate at 1 kHz, which is what makes 100 Hz reachable -- see the
	 * README.
	 */
	struct mpu6050_config cfg = {
		.accel_fs = MPU6050_ACCEL_FS_4G,
		.gyro_fs = MPU6050_GYRO_FS_500DPS,
		.dlpf = MPU6050_DLPF_44HZ,
		.sample_rate_hz = 100.0f,
	};
	/* mount all zero: chip axes are board axes */
	struct imu_dev imu = { 0 };
	i2c_master_bus_handle_t bus;

	ESP_ERROR_CHECK(i2c_new_master_bus(&buscfg, &bus));

	/* probe already logged why, under the "imu" tag */
	if (mpu6050_probe(&imu, bus, &cfg))
		return;
	imu_info(&imu, "sampling at %.2f Hz",
		 (double)mpu6050_sample_rate(&imu));

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (mpu6050_data_ready(&imu, &ready) || !ready) {
			vTaskDelay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		imu_log_sample(&imu, &s);
	}
}
