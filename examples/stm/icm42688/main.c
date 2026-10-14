/*
 * ICM-42688-P over I2C: stream accelerometer, gyroscope and die temperature
 * through the imu core, with the data-ready interrupt on INT1 for an app
 * that wants an EXTI line instead of polling INT_STATUS.
 *
 * A snippet, not a project: the clock tree, the printf() retarget, the bus
 * and the EXTI handler are the app's. Pins are examples -- I2C1 on PB8
 * (SCL) and PB9 (SDA), AF4. Written for the F4; an F7 sets Init.Timing
 * instead of Init.ClockSpeed. IMU_ICM42688 = y needs HAL_I2C_MODULE_ENABLED
 * for icm42688_probe_i2c().
 */
#include <stdbool.h>

#include CMSIS_device_header

#include "icm42688.h"
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
	/* Zero address selects 0x68, zero timeout 100 ms. */
	struct icm42688_i2c_config imu_bus = { .bus = &i2c1 };
	/* 2 g and 250 dps at 1 kHz, gyro anti-alias filter at row 20,
	 * 943 Hz; data ready on INT1, push-pull, active high, pulsed. */
	struct icm42688_config cfg = {
		.accel_fs = ICM42688_ACCEL_FS_2G,
		.gyro_fs = ICM42688_GYRO_FS_250DPS,
		.odr = ICM42688_ODR_1KHZ,
		.accel_ui_bw = ICM42688_UI_BW_M_4,
		.gyro_ui_bw = ICM42688_UI_BW_M_4,
		.accel_ui_order = ICM42688_UI_ORDER_2,
		.gyro_ui_order = ICM42688_UI_ORDER_2,
		.accel_aaf = ICM42688_AAF_DEFAULT,
		.gyro_aaf = 20,
		.data_ready_int1 = { .enable = true, .active_high = true },
	};
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	i2c1_init();

	/* The probe logs why it failed, as "imu: icm42688: ...". */
	if (icm42688_probe_i2c(&imu, &imu_bus, &cfg))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		/* Or wait for the INT1 edge here instead of polling. */
		if (icm42688_data_ready(&imu, &ready) || !ready)
			continue;
		if (imu_read(&imu, &s))
			continue;

		imu_log_sample(&imu, &s);
		HAL_Delay(100);
	}
}
