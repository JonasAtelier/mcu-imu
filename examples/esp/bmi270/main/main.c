/*
 * BMI270 over I2C: stream accelerometer, gyroscope and temperature through
 * the imu core. For SPI, initialise a host and call bmi270_probe_spi() with
 * the chip select instead; everything after the probe is the same. The probe
 * uploads Bosch's 8 KB configuration file, so it takes a moment.
 */
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi270.h"
#include "imu.h"
#include "imu_log.h"

#define SDA_PIN 21
#define SCL_PIN 22

void app_main(void)
{
	i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	struct bmi270_i2c_config imu_bus = { 0 };
	/* mount all zero: chip axes are board axes */
	struct imu_dev imu = { 0 };
	i2c_master_bus_handle_t bus;
	int ret;

	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));
	imu_bus.bus = bus;

	/*
	 * NULL selects the chip's reset values: +/-8 g at 100 Hz and
	 * +/-2000 dps at 200 Hz. The probe logs why it failed, under the "imu"
	 * tag.
	 */
	if (bmi270_probe_i2c(&imu, &imu_bus, NULL))
		return;

	for (;;) {
		struct imu_sample s;
		uint8_t ready;

		ret = bmi270_data_ready(&imu, &ready);
		if (ret) {
			imu_err(&imu, "data ready: %d", ret);
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		if (ready != (BMI270_DATA_READY_ACCEL |
			      BMI270_DATA_READY_GYRO)) {
			vTaskDelay(pdMS_TO_TICKS(5));
			continue;
		}

		ret = imu_read(&imu, &s);
		if (ret) {
			imu_err(&imu, "read: %d", ret);
			vTaskDelay(pdMS_TO_TICKS(1000));
			continue;
		}

		imu_log_sample(&imu, &s);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
