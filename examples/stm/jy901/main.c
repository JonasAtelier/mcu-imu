/*
 * JY901 over I2C: read accelerometer, gyroscope and temperature through the
 * imu core, all from one register burst, and the module's own roll, pitch
 * and yaw. For the serial link, HAL_UART_Init() a handle at 9600 bps and
 * call jy901_probe_uart() instead.
 *
 * A snippet, not a project: the clock tree, the printf() retarget and the
 * bus are the app's. Pins are examples -- I2C1 on PB8 (SCL) and PB9 (SDA),
 * AF4, with 4.7 k pull-ups: the module's bus is open drain (§5.2 of
 * docs/WT901.pdf). Written for the F4; an F7 sets Init.Timing instead of
 * Init.ClockSpeed. IMU_JY901 = y needs HAL_I2C_MODULE_ENABLED for
 * jy901_probe_i2c().
 */
#include CMSIS_device_header

#include "imu.h"
#include "imu_log.h"
#include "jy901.h"

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

	/* At most 400 kHz (chapter 7). */
	i2c1.Instance = I2C1;
	i2c1.Init.ClockSpeed = 400000;
	i2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	HAL_I2C_Init(&i2c1);
}

int main(void)
{
	/* Zero address selects 0x50, zero timeout 100 ms. */
	struct jy901_i2c_config imu_bus = { .bus = &i2c1 };
	/* mount all zero: module axes are board axes */
	static struct imu_dev imu;

	HAL_Init();
	/* SystemClock_Config() and the UART behind printf() go here. */
	i2c1_init();

	/* The probe logs why it failed, as "imu: jy901: ...". */
	if (jy901_probe_i2c(&imu, &imu_bus))
		for (;;)
			;

	for (;;) {
		struct imu_sample s;
		float angles[3];

		if (imu_read(&imu, &s))
			continue;
		imu_log_sample(&imu, &s);

		if (!jy901_read_angles(&imu, angles))
			imu_info(&imu, "roll %.2f pitch %.2f yaw %.2f deg",
				 (double)angles[0], (double)angles[1],
				 (double)angles[2]);
		/* The PDF gives no register update rate; 10 Hz is the stream's. */
		HAL_Delay(100);
	}
}
