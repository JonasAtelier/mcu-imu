/*
 * ICM-45686 register addresses and the bit fields this driver touches.
 *
 * Addresses and bit positions are transcribed from TDK InvenSense's own
 * reference driver, icm45686/imu/inv_imu_regmap_le.h in
 * github.com/tdk-invn-oss/motion.mcu.icm45686.driver, and every one of them
 * was cross-checked against Zephyr's independent driver at
 * drivers/sensor/tdk/icm45686/icm45686_reg.h. The two agree.
 *
 * The WHO_AM_I value is the one place they do not both state a number: TDK's
 * driver only returns the register, so 0xE9 comes from Zephyr alone.
 *
 * Registers this driver never touches -- FIFO, APEX/eDMP, the AUX interface,
 * self-test -- are left out. Reach them with icm45686_reg_read/write().
 */
#ifndef ICM45686_REGS_H
#define ICM45686_REGS_H

/* ---- data, bank 0. 14 bytes contiguous from ACCEL_DATA_X1_UI. ---- */
#define ICM_ACCEL_DATA_X1_UI	0x00	/* accel X..Z, 6 bytes */
#define ICM_GYRO_DATA_X1_UI	0x06	/* gyro  X..Z, 6 bytes */
#define ICM_TEMP_DATA1_UI	0x0C	/* temperature, 2 bytes */
#define ICM_DATA_BURST_LEN	14

/* ---- configuration ---- */
#define ICM_PWR_MGMT0		0x10
#define ICM_INT1_CONFIG0	0x16
#define ICM_INT1_STATUS0	0x19
#define ICM_ACCEL_CONFIG0	0x1B
#define ICM_GYRO_CONFIG0	0x1C

/* ---- indirect register access, for anything above 0xFF ---- */
#define ICM_IREG_ADDR_15_8	0x7C
#define ICM_IREG_ADDR_7_0	0x7D
#define ICM_IREG_DATA		0x7E
#define ICM_REG_MISC2		0x7F

#define ICM_WHO_AM_I		0x72
#define ICM_WHO_AM_I_VALUE	0xE9

#define ICM_SPI_READ		0x80	/* OR into the address byte to read */

/* ---- PWR_MGMT0 (0x10) ---- */
#define ICM_ACCEL_MODE_SHIFT	0	/* bits 1:0 */
#define ICM_GYRO_MODE_SHIFT	2	/* bits 3:2 */
#define ICM_MODE_OFF		0x00
#define ICM_MODE_STANDBY	0x01	/* gyro only */
#define ICM_MODE_LP		0x02
#define ICM_MODE_LN		0x03

/* ---- ACCEL_CONFIG0 (0x1B): odr 3:0, fs 6:4 ---- */
#define ICM_ACCEL_ODR_SHIFT	0
#define ICM_ACCEL_FS_SHIFT	4
/* ---- GYRO_CONFIG0 (0x1C): odr 3:0, fs 7:4 ---- */
#define ICM_GYRO_ODR_SHIFT	0
#define ICM_GYRO_FS_SHIFT	4

/* ---- INT1_STATUS0 (0x19) ---- */
#define ICM_INT_FIFO_FULL	(1u << 0)
#define ICM_INT_FIFO_THS	(1u << 1)
#define ICM_INT_DRDY		(1u << 2)
#define ICM_INT_RESET_DONE	(1u << 7)

/* ---- REG_MISC2 (0x7F) ---- */
#define ICM_IREG_DONE		(1u << 0)
#define ICM_SOFT_RST		(1u << 1)

/*
 * ---- SREG_CTRL (0xA267), reachable only through IREG ----
 * bit 1 selects how the sensor data registers are laid out:
 * 0 = little endian (low byte first), 1 = big endian.
 */
#define ICM_SREG_CTRL		0xA267
#define ICM_SREG_ENDIAN_SHIFT	1
#define ICM_SREG_LITTLE_ENDIAN	0
#define ICM_SREG_BIG_ENDIAN	1

#endif /* ICM45686_REGS_H */
