/*
 * MPU-6050 register addresses and the bit fields this driver touches.
 *
 * Every address and bit position below appears in two independent drivers and
 * they agree: Zephyr's drivers/sensor/tdk/mpu6050/ and the Linux kernel's
 * drivers/iio/imu/inv_mpu6050/. Where only one of them states a number, the
 * comment says which.
 *
 * Registers this driver does not model -- the FIFO, the DMP, the auxiliary
 * I2C master's slave registers, self-test, the motion detector -- are left
 * out. Reach them with mpu6050_reg_read/write().
 */
#ifndef MPU6050_REGS_H
#define MPU6050_REGS_H

#define MPU6050_SMPLRT_DIV	0x19
#define MPU6050_CONFIG		0x1A	/* DLPF_CFG in bits 2:0 */
#define MPU6050_GYRO_CONFIG	0x1B	/* FS_SEL  in bits 4:3 */
#define MPU6050_ACCEL_CONFIG	0x1C	/* AFS_SEL in bits 4:3 */
#define MPU6050_INT_PIN_CFG	0x37
#define MPU6050_INT_ENABLE	0x38
#define MPU6050_INT_STATUS	0x3A
#define MPU6050_ACCEL_XOUT_H	0x3B
#define MPU6050_USER_CTRL	0x6A
#define MPU6050_PWR_MGMT_1	0x6B
#define MPU6050_WHO_AM_I	0x75

/*
 * WHO_AM_I reports bits 6:1 of the I2C address, so it reads 0x68 on both
 * addresses -- AD0 is bit 0 and does not appear here. A part at 0x69 still
 * answers 0x68, which surprises people.
 */
#define MPU6050_WHO_AM_I_VALUE	0x68

#define MPU6050_ADDR_LOW	0x68	/* AD0 tied low, the usual breakout */
#define MPU6050_ADDR_HIGH	0x69	/* AD0 tied high */

/*
 * One burst from ACCEL_XOUT_H covers everything, but note the order:
 * accelerometer, then TEMPERATURE, then gyroscope. The temperature sits in
 * the middle, not at the end.
 */
#define MPU6050_BURST_LEN	14
#define MPU6050_TEMP_OFF	6	/* TEMP_OUT_H offset in the burst */
#define MPU6050_GYRO_OFF	8	/* byte offset of GYRO_XOUT_H (0x43) */

/* ---- PWR_MGMT_1 (0x6B) ---- */
#define MPU6050_DEVICE_RESET	0x80
#define MPU6050_SLEEP		0x40
#define MPU6050_CLKSEL_MASK	0x07
#define MPU6050_CLKSEL_PLL_X	0x01	/* gyro X PLL, steadier than the RC */

/* ---- GYRO_CONFIG / ACCEL_CONFIG ---- */
#define MPU6050_FS_SHIFT	3	/* both, bits 4:3 */

/* ---- INT_PIN_CFG (0x37) ---- */
#define MPU6050_I2C_BYPASS_EN	0x02

/* ---- INT_ENABLE (0x38) / INT_STATUS (0x3A) ---- */
#define MPU6050_DATA_RDY	0x01

/* ---- USER_CTRL (0x6A) ---- */
#define MPU6050_I2C_MST_EN	0x20

#endif /* MPU6050_REGS_H */
