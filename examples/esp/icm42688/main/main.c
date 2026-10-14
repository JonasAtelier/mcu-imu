/*
 * ICM-42688-P over SPI: stream accelerometer, gyroscope and die temperature
 * through the imu core. For I2C, initialise a bus and call
 * icm42688_probe_i2c() instead; everything after the probe is the same.
 *
 * Wiring: any SPI bus plus CS. The part takes VDD and VDDIO from 1.71 V to
 * 3.6 V (datasheet Table 3); check your breakout before wiring it.
 */
#include <stdbool.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "icm42688.h"
#include "imu.h"
#include "imu_log.h"

#define PIN_SCK		12
#define PIN_MOSI	11
#define PIN_MISO	13
#define PIN_CS		10

void app_main(void)
{
	spi_bus_config_t bus = {
		.sclk_io_num = PIN_SCK,
		.mosi_io_num = PIN_MOSI,
		.miso_io_num = PIN_MISO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = 32,
	};
	struct icm42688_spi_config imu_bus = {
		.host = SPI2_HOST,
		.cs_pin = PIN_CS,
	};
	/*
	 * 4 g and 500 dps at 200 Hz; a 3rd-order UI filter at max(400 Hz,
	 * ODR)/4, the anti-alias filters left at the chip's reset rows.
	 */
	struct icm42688_config cfg = {
		.accel_fs = ICM42688_ACCEL_FS_4G,
		.gyro_fs = ICM42688_GYRO_FS_500DPS,
		.odr = ICM42688_ODR_200HZ,
		.accel_ui_bw = ICM42688_UI_BW_M_4,
		.gyro_ui_bw = ICM42688_UI_BW_M_4,
		.accel_ui_order = ICM42688_UI_ORDER_3,
		.gyro_ui_order = ICM42688_UI_ORDER_3,
		.accel_aaf = ICM42688_AAF_DEFAULT,
		.gyro_aaf = ICM42688_AAF_DEFAULT,
	};
	/* mount all zero: chip axes are board axes */
	struct imu_dev imu = { 0 };

	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

	/* probe already logged why, under the "imu" tag */
	if (icm42688_probe_spi(&imu, &imu_bus, &cfg))
		return;
	imu_info(&imu, "%.0f counts/g, %.1f counts/dps",
		 (double)icm42688_accel_sensitivity(&imu),
		 (double)icm42688_gyro_sensitivity(&imu));

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (icm42688_data_ready(&imu, &ready) || !ready) {
			vTaskDelay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		imu_log_sample(&imu, &s);
	}
}
