/*
 * BMI088 over SPI: stream accelerometer, gyroscope and temperature through
 * the imu core. Each die has its own chip select; both need a pull-up so
 * neither is selected while the STM32 boots.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the bus
 * are the app's. Pins are examples -- SPI1 on PA5 (SCK), PA6 (MISO) and PA7
 * (MOSI), AF5; PD14 selects the accelerometer and PD15 the gyroscope.
 * IMU_BMI088 = y needs both HAL_I2C_MODULE_ENABLED and HAL_SPI_MODULE_ENABLED,
 * because the driver carries both transports.
 */
#include CMSIS_device_header

#include "bmi088.h"
#include "imu.h"
#include "imu_log.h"

static SPI_HandleTypeDef spi1;

/* The app's half: pins, chip selects, peripheral clock and SPI mode. */
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
		.Pin = GPIO_PIN_14 | GPIO_PIN_15,
		.Mode = GPIO_MODE_OUTPUT_PP,
		.Pull = GPIO_PULLUP,
		.Speed = GPIO_SPEED_FREQ_HIGH,
	};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();
	HAL_GPIO_Init(GPIOA, &pins);
	/* idle high */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_Init(GPIOD, &cs);

	/* Mode 0, 8-bit, software CS. /16 of a 90 MHz APB2 is 5.6 MHz. */
	spi1.Instance = SPI1;
	spi1.Init.Mode = SPI_MODE_MASTER;
	spi1.Init.Direction = SPI_DIRECTION_2LINES;
	spi1.Init.DataSize = SPI_DATASIZE_8BIT;
	spi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi1.Init.NSS = SPI_NSS_SOFT;
	spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	HAL_SPI_Init(&spi1);
}

int main(void)
{
	struct bmi088_spi_config imu_bus = {
		.bus = &spi1,
		.accel_cs_port = GPIOD,
		.accel_cs_pin = GPIO_PIN_14,
		.gyro_cs_port = GPIOD,
		.gyro_cs_pin = GPIO_PIN_15,
	};
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;
	int ret;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	spi1_init();

	/*
	 * NULL selects +/-6 g at 100 Hz and +/-2000 dps at 200 Hz. The probe
	 * logs why it failed, as "imu: bmi088: ...".
	 */
	if (bmi088_probe_spi(&imu, &imu_bus, NULL))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;

		ret = imu_read(&imu, &s);
		if (ret) {
			imu_err(&imu, "read: %d", ret);
			HAL_Delay(1000);
			continue;
		}

		imu_log_sample(&imu, &s);
		HAL_Delay(100);
	}
}
