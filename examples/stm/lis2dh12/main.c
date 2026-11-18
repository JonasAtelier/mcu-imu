/*
 * LIS2DH12: stream acceleration through the imu core. The part is an
 * accelerometer only, so every sample's gyroscope and temperature are NaN,
 * as lis2dh12.h promises. Its motion detector on INT1 stays off here;
 * lis2dh12_set_motion() turns it on.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * bus are the app's. Pins are examples -- I2C1 on PB8 (SCL) and PB9 (SDA),
 * AF4, with 4.7 k pull-ups; CS to 3.3 V selects I2C and SDO/SA0 to GND
 * gives address 0x18 (docs/hardware.md). Written for the F4, whose I2C
 * takes Init.ClockSpeed; an F7 sets Init.Timing instead, as CubeMX computes
 * it. Needs HAL_I2C_MODULE_ENABLED and IMU_LIS2DH12 = y.
 */
#include <stdbool.h>

#include CMSIS_device_header

#include "imu.h"
#include "imu_log.h"
#include "lis2dh12.h"

static I2C_HandleTypeDef i2c1;

/* The app's half: pins, peripheral clock and bus speed. */
static void i2c1_init(void)
{
	GPIO_InitTypeDef pins = {
		.Pin = GPIO_PIN_8 | GPIO_PIN_9,
		.Mode = GPIO_MODE_AF_OD,
		.Pull = GPIO_PULLUP,	/* weak; fit real 4.7k pull-ups too */
		.Speed = GPIO_SPEED_FREQ_HIGH,
		.Alternate = GPIO_AF4_I2C1,
	};

	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();
	HAL_GPIO_Init(GPIOB, &pins);

	/* 400 kHz is the most the part takes. */
	i2c1.Instance = I2C1;
	i2c1.Init.ClockSpeed = 400000;
	i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&i2c1);
}

int main(void)
{
	/* Zero address and timeout: 0x18, 100 ms. */
	struct lis2dh12_i2c_config chip_bus = { .bus = &i2c1 };
	/* 12-bit samples at 100 Hz, +-2 g. */
	const struct lis2dh12_config cfg = {
		.mode = LIS2DH12_MODE_HIGH_RES,
		.odr = LIS2DH12_ODR_100HZ,
		.full_scale = LIS2DH12_FS_2G,
	};
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	i2c1_init();

	/* probe already logged why, as "imu: lis2dh12: ..." */
	if (lis2dh12_probe_i2c(&imu, &chip_bus, &cfg))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (lis2dh12_data_ready(&imu, &ready) || !ready) {
			HAL_Delay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		/* gyro[] and temp_c print as nan: the part has neither. */
		imu_log_sample(&imu, &s);
	}
}
