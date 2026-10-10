/*
 * BMI088 over SPI: stream accelerometer, gyroscope and temperature through
 * the imu core. Each die has its own chip select; both need a pull-up so
 * neither is selected while the ESP32 boots.
 */
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmi088.h"
#include "imu.h"
#include "imu_log.h"

#define SPI_HOST_ID	SPI2_HOST
#define SCK_PIN		18
#define MOSI_PIN	23
#define MISO_PIN	19
#define ACCEL_CS_PIN	5
#define GYRO_CS_PIN	17

void app_main(void)
{
	spi_bus_config_t bus_config = {
		.sclk_io_num = SCK_PIN,
		.mosi_io_num = MOSI_PIN,
		.miso_io_num = MISO_PIN,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
	};
	struct bmi088_spi_config imu_bus = {
		.host = SPI_HOST_ID,
		.accel_cs_pin = ACCEL_CS_PIN,
		.gyro_cs_pin = GYRO_CS_PIN,
		/* Zero fields select SPI mode 0 at 5 MHz. */
	};
	/* mount all zero: chip axes are board axes */
	struct imu_dev imu = { 0 };
	int ret;

	ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &bus_config,
					   SPI_DMA_CH_AUTO));

	/*
	 * NULL selects +/-6 g at 100 Hz and +/-2000 dps at 200 Hz. The probe
	 * logs why it failed, under the "imu" tag.
	 */
	if (bmi088_probe_spi(&imu, &imu_bus, NULL))
		return;

	for (;;) {
		struct imu_sample s;

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
