/*
 * ICM-42688-P registers, bit fields and timings this driver uses.
 *
 * Every value is from TDK InvenSense's ICM-42688-P datasheet, document
 * DS-000347 revision 1.6, in docs/icm42688p.pdf (gitignored), cited per
 * value by section or table. Nothing is shared with the ICM-45686 driver:
 * the two parts' register maps have nothing in common.
 *
 * Registers sit in user banks 0, 1, 2 and 4, chosen by REG_BANK_SEL, which
 * every bank has at 0x76 (14.59). Each name below says its bank.
 */
#ifndef ICM42688_REGS_H
#define ICM42688_REGS_H

/* ---- bank 0, section 13.1 and chapter 14 ---- */
#define ICM42688_DEVICE_CONFIG		0x11	/* 14.1 */
#define ICM42688_DRIVE_CONFIG		0x13	/* 14.2 */
#define ICM42688_INT_CONFIG		0x14	/* 14.3 */
#define ICM42688_TEMP_DATA1		0x1D	/* 14.5: first data byte */
#define ICM42688_INT_STATUS		0x2D	/* 14.21 */
#define ICM42688_INTF_CONFIG0		0x4C	/* 14.34 */
#define ICM42688_PWR_MGMT0		0x4E	/* 14.36 */
#define ICM42688_GYRO_CONFIG0		0x4F	/* 14.37 */
#define ICM42688_ACCEL_CONFIG0		0x50	/* 14.38 */
#define ICM42688_GYRO_CONFIG1		0x51	/* 14.39 */
#define ICM42688_GYRO_ACCEL_CONFIG0	0x52	/* 14.40 */
#define ICM42688_ACCEL_CONFIG1		0x53	/* 14.41 */
#define ICM42688_INT_CONFIG0		0x63	/* 14.49 */
#define ICM42688_INT_CONFIG1		0x64	/* 14.50 */
#define ICM42688_INT_SOURCE0		0x65	/* 14.51 */
#define ICM42688_WHO_AM_I		0x75	/* 14.58 */
#define ICM42688_REG_BANK_SEL		0x76	/* 14.59, in every bank */

/* ---- bank 1, section 13.2 and chapter 15 ---- */
#define ICM42688_GYRO_CONFIG_STATIC2	0x0B	/* 15.2 */
#define ICM42688_GYRO_CONFIG_STATIC3	0x0C	/* 15.3 */
#define ICM42688_GYRO_CONFIG_STATIC4	0x0D	/* 15.4 */
#define ICM42688_GYRO_CONFIG_STATIC5	0x0E	/* 15.5 */
#define ICM42688_INTF_CONFIG6		0x7C	/* 15.19 */

/* ---- bank 2, section 13.3 and chapter 16 ---- */
#define ICM42688_ACCEL_CONFIG_STATIC2	0x03	/* 16.1 */
#define ICM42688_ACCEL_CONFIG_STATIC3	0x04	/* 16.2 */
#define ICM42688_ACCEL_CONFIG_STATIC4	0x05	/* 16.3 */

#define ICM42688_BANK_0			0
#define ICM42688_BANK_1			1
#define ICM42688_BANK_2			2

#define ICM42688_WHO_AM_I_VALUE		0x47	/* 14.58, reset value */

/*
 * TEMP_DATA1 (0x1D) to GYRO_DATA_Z0 (0x2A): temperature, accel X..Z, gyro
 * X..Z, two bytes each (13.1), big endian as INTF_CONFIG0 resets it.
 */
#define ICM42688_BURST_LEN		14
#define ICM42688_BURST_ACCEL		2
#define ICM42688_BURST_GYRO		8

/* SPI: bit 7 of the address byte reads (9.6, item 5). */
#define ICM42688_SPI_READ		0x80

/* ---- DEVICE_CONFIG (0x11), 14.1 ---- */
#define ICM42688_SOFT_RESET		(1u << 0)

/* ---- DRIVE_CONFIG (0x13), 14.2, and the I2C settings of 12.3 ---- */
/* I2C_SLEW_RATE (bits 5:3) = 1, SPI_SLEW_RATE (bits 2:0) = 1 */
#define ICM42688_DRIVE_CONFIG_I2C	0x09

/* ---- INT_CONFIG (0x14), 14.3: INT1 ---- */
#define ICM42688_INT1_POLARITY_HIGH	(1u << 0)
#define ICM42688_INT1_PUSH_PULL		(1u << 1)
#define ICM42688_INT1_LATCHED		(1u << 2)

/* ---- INT_STATUS (0x2D), 14.21 ---- */
#define ICM42688_DATA_RDY_INT		(1u << 3)

/* ---- INTF_CONFIG0 (0x4C), 14.34 ---- */
#define ICM42688_SENSOR_DATA_BIG_ENDIAN	(1u << 4)

/* ---- PWR_MGMT0 (0x4E), 14.36 ---- */
#define ICM42688_ACCEL_MODE_SHIFT	0	/* bits 1:0 */
#define ICM42688_GYRO_MODE_SHIFT	2	/* bits 3:2 */
#define ICM42688_MODE_OFF		0x0
#define ICM42688_MODE_LN		0x3	/* low noise, both sensors */

/* ---- GYRO_CONFIG0 (0x4F) and ACCEL_CONFIG0 (0x50), 14.37-14.38 ---- */
#define ICM42688_FS_SHIFT		5	/* *_FS_SEL, bits 7:5 */
#define ICM42688_ODR_MASK		0x0F	/* *_ODR, bits 3:0 */

/* ---- GYRO_CONFIG1 (0x51), 14.39: GYRO_UI_FILT_ORD, bits 3:2 ---- */
#define ICM42688_GYRO_ORDER_SHIFT	2
/* ---- ACCEL_CONFIG1 (0x53), 14.41: ACCEL_UI_FILT_ORD, bits 4:3 ---- */
#define ICM42688_ACCEL_ORDER_SHIFT	3
#define ICM42688_ORDER_MASK		0x3

/* ---- GYRO_ACCEL_CONFIG0 (0x52), 14.40 ---- */
#define ICM42688_ACCEL_BW_SHIFT		4	/* ACCEL_UI_FILT_BW, bits 7:4 */
#define ICM42688_GYRO_BW_SHIFT		0	/* GYRO_UI_FILT_BW, bits 3:0 */

/* ---- INT_CONFIG0 (0x63), 14.49: UI_DRDY_INT_CLEAR, bits 5:4 ---- */
/* 11: clear on status bit read and on sensor register read */
#define ICM42688_DRDY_CLEAR_ON_EITHER	(0x3u << 4)

/*
 * ---- INT_CONFIG1 (0x64), 14.50 ----
 * INT_TPULSE_DURATION = 1 (8 us) and INT_TDEASSERT_DISABLE = 1: required
 * at 4 kHz and above, optional below, so right at every rate.
 * INT_ASYNC_RESET (bit 4) = 0: resets to 1, and 12.6 says to clear it "for
 * proper INT1 and INT2 pin operation".
 */
#define ICM42688_INT_CONFIG1_VALUE	((1u << 6) | (1u << 5))

/* ---- INT_SOURCE0 (0x65), 14.51 ---- */
#define ICM42688_UI_DRDY_INT1_EN	(1u << 3)

/* ---- GYRO_CONFIG_STATIC2 (bank 1, 0x0B), 15.2 ---- */
#define ICM42688_GYRO_AAF_DIS		(1u << 1)
/* ---- ACCEL_CONFIG_STATIC2 (bank 2, 0x03), 16.1 ---- */
#define ICM42688_ACCEL_AAF_DIS		(1u << 0)
#define ICM42688_ACCEL_AAF_DELT_SHIFT	1	/* bits 6:1 */
/* GYRO_AAF_DELT (bank 1, 0x0C) bits 5:0 (15.3); both DELTs are 6 bits */
#define ICM42688_AAF_DELT_MASK		0x3F
/* *_AAF_BITSHIFT bits 7:4, DELTSQR[11:8] bits 3:0 (15.5, 16.3) */
#define ICM42688_AAF_BITSHIFT_SHIFT	4

/* ---- INTF_CONFIG6 (bank 1, 0x7C), 15.19, and the I2C settings of 12.3 ---- */
#define ICM42688_I3C_SDR_EN		(1u << 0)
#define ICM42688_I3C_DDR_EN		(1u << 1)

/* ---- I2C addresses, 9.3: AP_AD0 low or high ---- */
#define ICM42688_I2C_ADDRESS_LOW	0x68
#define ICM42688_I2C_ADDRESS_HIGH	0x69

/* ---- timing ---- */
/* "After writing 1 ..., wait 1ms for soft reset to be effective" (14.1). */
#define ICM42688_RESET_WAIT_MS		1
/* From off to any mode, "do not issue any register writes for 200us". */
#define ICM42688_MODE_WAIT_US		200

#endif /* ICM42688_REGS_H */
