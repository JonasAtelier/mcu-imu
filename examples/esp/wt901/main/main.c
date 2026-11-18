/*
 * WT901C-TTL: stream accelerometer, gyroscope and temperature through the
 * imu core, from the frames the module sends on its own.
 *
 * Wiring: the module's TX to PIN_RX, its RX to PIN_TX, TTL levels, never
 * RS232. It takes 3.3 V to 5 V (§2 item 1 of docs/WT901.pdf) and talks at
 * 9600 bps, no parity, out of the box (chapter 6).
 */
#include "driver/uart.h"
#include "esp_err.h"

#include "imu.h"
#include "imu_log.h"
#include "wt901.h"

#define WT901_UART	UART_NUM_1
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

	ESP_ERROR_CHECK(uart_driver_install(WT901_UART, 1024, 0, 0, NULL, 0));
	ESP_ERROR_CHECK(uart_param_config(WT901_UART, &link));
	ESP_ERROR_CHECK(uart_set_pin(WT901_UART, PIN_TX, PIN_RX,
				     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	/* NULL: the defaults. The probe already logged why, under "imu". */
	if (wt901_probe(&imu, WT901_UART, NULL))
		return;

	for (;;) {
		struct imu_sample s;

		/* Blocks for the next pair of frames: 10 Hz by default. */
		if (imu_read(&imu, &s))
			continue;
		imu_log_sample(&imu, &s);
	}
}
