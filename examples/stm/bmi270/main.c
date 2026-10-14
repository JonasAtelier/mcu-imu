/*
 * BMI270 over SPI: stream accelerometer, gyroscope and temperature through
 * the imu core, with the data-ready interrupt on INT1 for an app that wants
 * an EXTI line instead of polling STATUS.
 *
 * A snippet, not a project: the clock tree, the printf() retarget, the bus
 * and the EXTI handler are the app's. Pins are examples -- SPI1 on PA5
 * (SCK), PA6 (MISO) and PA7 (MOSI), AF5; PD14 is the chip select, pulled up
 * so the chip is not selected while the STM32 boots. Written for the F4.
 * IMU_BMI270 = y needs HAL_SPI_MODULE_ENABLED for bmi270_probe_spi(). The
 * probe uploads Bosch's 8 KB configuration file; with the HAL's 1 ms
 * delays it takes about 0.6 s.
 */
#include <stdbool.h>

#include CMSIS_device_header

#include "bmi270.h"
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
	/* idle high */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
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
	struct bmi270_spi_config imu_bus = {
		.bus = &spi1,
		.cs_port = GPIOD,
		.cs_pin = GPIO_PIN_14,
	};
	/* The reset values, plus data ready on INT1: push-pull, active high */
	struct bmi270_config config = {
		.accel_range = BMI270_ACCEL_RANGE_8G,
		.accel_odr = BMI270_ACCEL_ODR_100_HZ,
		.accel_filter = BMI270_ACCEL_FILTER_NORMAL,
		.gyro_range = BMI270_GYRO_RANGE_2000_DPS,
		.gyro_odr = BMI270_GYRO_ODR_200_HZ,
		.gyro_filter = BMI270_GYRO_FILTER_NORMAL,
		.data_ready_int1 = { .enable = true, .active_high = true },
	};
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;
	int ret;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	spi1_init();

	/* The probe logs why it failed, as "imu: bmi270: ...". */
	if (bmi270_probe_spi(&imu, &imu_bus, &config))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;
		uint8_t ready;

		/* Or wait for the INT1 edge here instead of polling. */
		ret = bmi270_data_ready(&imu, &ready);
		if (ret) {
			imu_err(&imu, "data ready: %d", ret);
			HAL_Delay(1000);
			continue;
		}

		if (ready != (BMI270_DATA_READY_ACCEL |
			      BMI270_DATA_READY_GYRO)) {
			HAL_Delay(5);
			continue;
		}

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
