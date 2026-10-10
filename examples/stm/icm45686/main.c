/*
 * ICM-45686: stream accelerometer, gyroscope and die temperature through the
 * imu core.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the bus
 * are the app's. Pins are examples -- SPI1 on PA5 (SCK), PA6 (MISO) and PA7
 * (MOSI), AF5, with PD14 as a plain GPIO chip select -- any SPI your chip
 * routes works. Needs HAL_SPI_MODULE_ENABLED and IMU_ICM45686 = y.
 *
 * Wiring: the part is 1.8 V core with VDDIO down to 1.71 V -- check your
 * breakout's regulator before wiring it to a 3.3 V STM32.
 */
#include <stdbool.h>

#include CMSIS_device_header

#include "icm45686.h"
#include "imu.h"
#include "imu_log.h"

static SPI_HandleTypeDef spi1;

/* The app's half: pins, chip select, peripheral clock and SPI mode. */
static void spi1_init(void)
{
	GPIO_InitTypeDef pins = {
		.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
		.Mode = GPIO_MODE_AF_PP,
		.Pull = GPIO_NOPULL,
		.Speed = GPIO_SPEED_FREQ_VERY_HIGH,
		.Alternate = GPIO_AF5_SPI1,
	};
	GPIO_InitTypeDef cs = {
		.Pin = GPIO_PIN_14,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_HIGH,
	};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &pins);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);	/* idle high */
	HAL_GPIO_Init(GPIOD, &cs);

	/* Mode 3, 8-bit, software CS. /16 of a 90 MHz APB2 is 5.6 MHz. */
	spi1.Instance = SPI1;
	spi1.Init.Mode = SPI_MODE_MASTER;
	spi1.Init.Direction = SPI_DIRECTION_2LINES;
	spi1.Init.DataSize = SPI_DATASIZE_8BIT;
	spi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	spi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	spi1.Init.NSS = SPI_NSS_SOFT;
	spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	HAL_SPI_Init(&spi1);
}

int main(void)
{
	/*
	 * 4 g and 500 dps suit a hand-held board. Widen them for anything that
	 * gets thrown, dropped or spun -- this part goes to 32 g and 4000 dps,
	 * and a clipped sample is indistinguishable from a real one.
	 */
	struct icm45686_config cfg = {
		.spi = &spi1,
		.cs_port = GPIOD,
		.cs_pin = GPIO_PIN_14,
		.accel_fs = ICM45686_ACCEL_FS_4G,
		.gyro_fs = ICM45686_GYRO_FS_500DPS,
		.odr = ICM45686_ODR_200HZ,
	};
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	spi1_init();

	/* probe already logged why, as "imu: icm45686: ..." */
	if (icm45686_probe(&imu, &cfg))
		for (;;)
			;
	imu_info(&imu, "%.0f counts/g, %.3f counts/dps",
		 (double)icm45686_accel_sensitivity(&imu),
		 (double)icm45686_gyro_sensitivity(&imu));

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (icm45686_data_ready(&imu, &ready) || !ready) {
			HAL_Delay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		imu_log_sample(&imu, &s);
	}
}
