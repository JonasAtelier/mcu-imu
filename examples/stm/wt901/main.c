/*
 * WT901C-TTL: stream accelerometer, gyroscope and temperature through the
 * imu core, from the frames the module sends on its own.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * UART are the app's. Pins are examples -- USART1 on PA9 (TX) and PA10
 * (RX), AF7; the module's TX goes to PA10 and its RX to PA9, TTL levels,
 * never RS232. It takes 3.3 V to 5 V (§2 item 1 of docs/WT901.pdf) and
 * talks at 9600 bps, 8 data bits, no parity, one stop bit out of the box
 * (chapter 6). Written for the F4. Needs HAL_UART_MODULE_ENABLED and
 * IMU_WT901 = y.
 *
 * USART1 carries the sensor only: retarget printf() to another UART, such
 * as USART2, which a Nucleo-64 wires to the ST-LINK's virtual COM port.
 */
#include CMSIS_device_header

#include "imu.h"
#include "imu_log.h"
#include "wt901.h"

static UART_HandleTypeDef uart1;

/* The app's half: pins, peripheral clock and the module's line settings. */
static void uart1_init(void)
{
	GPIO_InitTypeDef pins = {
		.Pin = GPIO_PIN_9 | GPIO_PIN_10,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_PULLUP,	/* RX idles high, even unplugged */
		.Speed = GPIO_SPEED_FREQ_HIGH,
		.Alternate = GPIO_AF7_USART1,
	};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &pins);

	uart1.Instance = USART1;
	uart1.Init.BaudRate = 9600;
	uart1.Init.WordLength = UART_WORDLENGTH_8B;
	uart1.Init.StopBits = UART_STOPBITS_1;
	uart1.Init.Parity = UART_PARITY_NONE;
	uart1.Init.Mode = UART_MODE_TX_RX;
	uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&uart1);
}

int main(void)
{
	/* mount all zero: module axes are board axes */
	static struct imu_dev imu;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	uart1_init();

	/* NULL: the defaults. The probe logs why, as "imu: wt901: ...". */
	if (wt901_probe(&imu, &uart1, NULL))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;

		/* Blocks for the next pair of frames: 10 Hz by default. */
		if (imu_read(&imu, &s))
			continue;
		imu_log_sample(&imu, &s);
	}
}
