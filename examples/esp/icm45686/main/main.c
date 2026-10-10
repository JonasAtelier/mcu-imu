/*
 * ICM-45686: stream accelerometer, gyroscope and die temperature through the
 * imu core.
 *
 * Wiring: any SPI bus plus CS. The part is 1.8 V core with VDDIO down to
 * 1.71 V -- check your breakout's regulator before wiring it to a 3.3 V ESP32.
 */
#include <stdbool.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "icm45686.h"
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
	/*
	 * 4 g and 500 dps suit a hand-held board. Widen them for anything that
	 * gets thrown, dropped or spun -- this part goes to 32 g and 4000 dps,
	 * and a clipped sample is indistinguishable from a real one.
	 */
	struct icm45686_config cfg = {
		.host = SPI2_HOST,
		.cs_gpio = PIN_CS,
		.accel_fs = ICM45686_ACCEL_FS_4G,
		.gyro_fs = ICM45686_GYRO_FS_500DPS,
		.odr = ICM45686_ODR_200HZ,
	};
	/* mount all zero: chip axes are board axes */
	struct imu_dev imu = { 0 };

	ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

	/* probe already logged why, under the "imu" tag */
	if (icm45686_probe(&imu, &cfg))
		return;
	imu_info(&imu, "%.0f counts/g, %.3f counts/dps",
		 (double)icm45686_accel_sensitivity(&imu),
		 (double)icm45686_gyro_sensitivity(&imu));

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (icm45686_data_ready(&imu, &ready) || !ready) {
			vTaskDelay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		imu_log_sample(&imu, &s);
	}
}
