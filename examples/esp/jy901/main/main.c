/*
 * JY901 on its serial link: stream accelerometer, gyroscope and temperature
 * through the imu core, and the module's own roll, pitch and yaw. For I2C,
 * initialise a bus and call jy901_probe_i2c() instead; everything after the
 * probe is the same.
 *
 * Wiring: the module's TX to PIN_RX, its RX to PIN_TX, TTL levels. It takes
 * 3.3 V to 5 V (§2 item 1 of docs/WT901.pdf) and talks at 9600 bps, no
 * parity, out of the box (chapter 6).
 */
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
#include "imu_log.h"
#include "jy901.h"

#define JY901_UART	UART_NUM_1
#define PIN_TX		17
#define PIN_RX		18

void app_main(void)
{
	uart_config_t link = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};
	/* mount all zero: module axes are board axes */
	struct imu_dev imu = { 0 };

	ESP_ERROR_CHECK(uart_driver_install(JY901_UART, 1024, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(JY901_UART, &link));
	ESP_ERROR_CHECK(uart_set_pin(JY901_UART, PIN_TX, PIN_RX,
				     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	/* probe already logged why, under the "imu" tag */
	if (jy901_probe_uart(&imu, JY901_UART, NULL))
		return;

	for (;;) {
		struct imu_sample s;
		float angles[3];

		/* Blocks for the next pair of frames: 10 Hz by default. */
		if (imu_read(&imu, &s))
			continue;
		imu_log_sample(&imu, &s);

		if (!jy901_read_angles(&imu, angles))
			imu_info(&imu, "roll %.2f pitch %.2f yaw %.2f deg",
				 (double)angles[0], (double)angles[1],
				 (double)angles[2]);
	}
}
