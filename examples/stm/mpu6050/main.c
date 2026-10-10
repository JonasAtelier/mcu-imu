/*
 * MPU-6050: stream accelerometer, gyroscope and die temperature through the
 * imu core.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the bus
 * are the app's. Pins are examples -- I2C1 on PB8 (SCL) and PB9 (SDA), AF4 --
 * any pair your chip routes to an I2C peripheral works. Written for the F4,
 * whose I2C takes Init.ClockSpeed; an F7 sets Init.Timing instead, as CubeMX
 * computes it. Needs HAL_I2C_MODULE_ENABLED and IMU_MPU6050 = y.
 *
 * Wiring: a GY-521 breakout carries its own regulator and level shifters, so
 * 3.3 V or 5 V both work on that board -- the bare chip is 3.3 V only.
 */
#include <stdbool.h>

#include CMSIS_device_header

#include "imu.h"
#include "imu_log.h"
#include "mpu6050.h"

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

	i2c1.Instance = I2C1;
	i2c1.Init.ClockSpeed = 400000;
	i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&i2c1);
}

int main(void)
{
	/*
	 * 4 g and 500 dps suit a hand-held board. The 44 Hz filter puts the
	 * base rate at 1 kHz, which is what makes 100 Hz reachable -- see the
	 * README.
	 */
	struct mpu6050_config cfg = {
		.accel_fs = MPU6050_ACCEL_FS_4G,
		.gyro_fs = MPU6050_GYRO_FS_500DPS,
		.dlpf = MPU6050_DLPF_44HZ,
		.sample_rate_hz = 100.0f,
	};
	/* mount all zero: chip axes are board axes */
	static struct imu_dev imu;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	i2c1_init();

	/* probe already logged why, as "imu: mpu6050: ..." */
	if (mpu6050_probe(&imu, &i2c1, &cfg))
		for (;;)
			;
	imu_info(&imu, "sampling at %.2f Hz",
		 (double)mpu6050_sample_rate(&imu));

	for (;;) {
		struct imu_sample s;
		bool ready = false;

		if (mpu6050_data_ready(&imu, &ready) || !ready) {
			HAL_Delay(1);
			continue;
		}
		if (imu_read(&imu, &s))
			continue;

		imu_log_sample(&imu, &s);
	}
}
