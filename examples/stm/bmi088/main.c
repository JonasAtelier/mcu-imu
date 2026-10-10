/*
 * BMI088 over I2C: stream accelerometer, gyroscope and temperature through
 * the imu core. For SPI see examples/stm/bmi088_spi; everything after the
 * probe is the same.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the bus
 * are the app's. Pins are examples -- I2C1 on PB8 (SCL) and PB9 (SDA), AF4.
 * Written for the F4; an F7 sets Init.Timing instead of Init.ClockSpeed.
 * IMU_BMI088 = y needs both HAL_I2C_MODULE_ENABLED and HAL_SPI_MODULE_ENABLED,
 * because the driver carries both transports.
 */
#include <stdint.h>

#include CMSIS_device_header

#include "bmi088.h"
#include "imu.h"
#include "imu_log.h"

static I2C_HandleTypeDef i2c1;

/* The app's half: pins, peripheral clock and bus speed. */
static void i2c1_init(void)
{
	GPIO_InitTypeDef pins = {
		.Pin = GPIO_PIN_8 | GPIO_PIN_9,
		.Mode = GPIO_MODE_AF_OD,
		.Pull = GPIO_PULLUP,	/* weak; fit real pull-ups too */
		.Speed = GPIO_SPEED_FREQ_HIGH,
		.Alternate = GPIO_AF4_I2C1,
	};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_GPIO_Init(GPIOB, &pins);

	i2c1.Instance = I2C1;
	i2c1.Init.ClockSpeed = 400000;
	i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&i2c1);
}

int main(void)
{
	/* Zero addresses select 0x18 and 0x68, zero timeout 100 ms. */
	struct bmi088_i2c_config imu_bus = { .bus = &i2c1 };
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;
	int ret;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	i2c1_init();

	/*
	 * NULL selects +/-6 g at 100 Hz and +/-2000 dps at 200 Hz. The probe
	 * logs why it failed, as "imu: bmi088: ...".
	 */
	if (bmi088_probe_i2c(&imu, &imu_bus, NULL))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;
		uint8_t ready;

		ret = bmi088_data_ready(&imu, &ready);
		if (ret) {
			imu_err(&imu, "data ready: %d", ret);
			HAL_Delay(1000);
			continue;
		}

		if (ready != (BMI088_DATA_READY_ACCEL |
			      BMI088_DATA_READY_GYRO)) {
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
