/*
 * BMI270 registers, bit fields and timings this driver uses.
 *
 * Every value is from Bosch Sensortec's BMI270 datasheet, document
 * BST-BMI270-DS000-08, revision 1.6 (March 2026), in docs/bmi270.pdf
 * (gitignored), cited per value by section or table. Where Bosch's own
 * BMI270 Sensor API (commit 41129fc, bmi2.c) is cited too, it is for a
 * choice the datasheet leaves open.
 */
#ifndef BMI270_REGS_H
#define BMI270_REGS_H

/* ---- registers, section 5.2 ---- */
#define BMI270_CHIP_ID		0x00	/* 5.2.1 */
#define BMI270_STATUS		0x03	/* 5.2.3 */
#define BMI270_DATA_8		0x0C	/* 5.2.12: ACC_X LSB, first data byte */
#define BMI270_INTERNAL_STATUS	0x21	/* 5.2.33 */
#define BMI270_TEMPERATURE_0	0x22	/* 5.2.34 */
#define BMI270_FEAT_PAGE	0x2F	/* 5.2.39 */
#define BMI270_GYR_CAS		0x3C	/* 5.2.40, feature page 0 */
#define BMI270_ACC_CONF		0x40	/* 5.2.41 */
#define BMI270_ACC_RANGE	0x41	/* 5.2.42 */
#define BMI270_GYR_CONF		0x42	/* 5.2.43 */
#define BMI270_GYR_RANGE	0x43	/* 5.2.44 */
#define BMI270_INT1_IO_CTRL	0x53	/* 5.2.58 */
#define BMI270_INT_LATCH	0x55	/* 5.2.60 */
#define BMI270_INT_MAP_DATA	0x58	/* 5.2.63 */
#define BMI270_INIT_CTRL	0x59	/* 5.2.64 */
#define BMI270_INIT_ADDR_0	0x5B	/* 5.2.65; INIT_ADDR_1 is 0x5C */
#define BMI270_INIT_DATA	0x5E	/* 5.2.67 */
#define BMI270_PWR_CONF		0x7C	/* 5.2.84 */
#define BMI270_PWR_CTRL		0x7D	/* 5.2.85 */
#define BMI270_CMD		0x7E	/* 5.2.86 */

#define BMI270_CHIP_ID_VALUE	0x24	/* 5.2.1, reset value */

/*
 * DATA_8 (0x0C) up to TEMPERATURE_1 (0x23): accel X..Z, gyro X..Z, each
 * LSB first (5.2.12-5.2.23), then sensortime, the event and interrupt status
 * registers, and temperature (5.2.24-5.2.35). One burst reads one sample
 * set; the side effects of reading the registers in between are in
 * bmi270.c.
 */
#define BMI270_BURST_LEN	24
#define BMI270_BURST_GYRO	6	/* DATA_14, gyro X LSB */
#define BMI270_BURST_TEMP	22	/* TEMPERATURE_0 */

/* SPI: bit 7 of the address byte reads (6.4, R/W bit). */
#define BMI270_SPI_READ		0x80

/* ---- STATUS (0x03), 5.2.3 ---- */
#define BMI270_STATUS_DRDY_GYR	(1u << 6)
#define BMI270_STATUS_DRDY_ACC	(1u << 7)

/* ---- INTERNAL_STATUS (0x21), 5.2.33: message, bits 3:0 ---- */
#define BMI270_MESSAGE_MASK	0x0F
#define BMI270_MESSAGE_INIT_OK	0x01

/* ---- GYR_CAS (0x3C, page 0), 5.2.40: factor_zx, a 7-bit signed value ---- */
#define BMI270_GYR_CAS_MASK	0x7F
#define BMI270_GYR_CAS_SIGN	0x40

/* ---- ACC_CONF (0x40), 5.2.41 ---- */
#define BMI270_ACC_ODR_SHIFT	0	/* acc_odr, bits 3:0 */
#define BMI270_ACC_BWP_SHIFT	4	/* acc_bwp, bits 6:4 */
#define BMI270_ACC_FILTER_PERF	(1u << 7)

/* ---- GYR_CONF (0x42), 5.2.43 ---- */
#define BMI270_GYR_ODR_SHIFT	0	/* gyr_odr, bits 3:0 */
#define BMI270_GYR_BWP_SHIFT	4	/* gyr_bwp, bits 5:4 */
#define BMI270_GYR_NOISE_PERF	(1u << 6)
#define BMI270_GYR_FILTER_PERF	(1u << 7)

/* ---- INT1_IO_CTRL (0x53), 5.2.58 ---- */
#define BMI270_INT_LVL_HIGH	(1u << 1)	/* lvl: active high */
#define BMI270_INT_OD		(1u << 2)	/* od: open drain */
#define BMI270_INT_OUTPUT_EN	(1u << 3)

/* ---- INT_LATCH (0x55), 5.2.60 ---- */
#define BMI270_INT_LATCHED	(1u << 0)

/* ---- INT_MAP_DATA (0x58), 5.2.63 ---- */
#define BMI270_DRDY_INT1	(1u << 2)

/* ---- INIT_CTRL (0x59), 5.2.64 and 4.4 ---- */
#define BMI270_INIT_CTRL_PREPARE	0x00
#define BMI270_INIT_CTRL_DONE		0x01

/*
 * ---- INIT_ADDR_0/1 (0x5B, 0x5C), 5.2.65-5.2.66 ----
 * The base address of a chunk, in 16-bit words: bits 3:0 in INIT_ADDR_0,
 * bits 11:4 in INIT_ADDR_1. Chunks are a whole number of words (4.4,
 * footnote 4: "incremented by the length ... in bytes/2").
 */
#define BMI270_INIT_ADDR_0_MASK	0x0F
#define BMI270_INIT_ADDR_1_SHIFT	4

/* ---- PWR_CONF (0x7C), 5.2.84 ---- */
#define BMI270_ADV_POWER_SAVE	(1u << 0)
#define BMI270_FIFO_SELF_WAKEUP	(1u << 1)

/* ---- PWR_CTRL (0x7D), 5.2.85 ---- */
#define BMI270_GYR_EN		(1u << 1)
#define BMI270_ACC_EN		(1u << 2)
#define BMI270_TEMP_EN		(1u << 3)

/* ---- CMD (0x7E), 5.2.86 ---- */
#define BMI270_CMD_SOFTRESET	0xB6

/* ---- I2C addresses, 6.5: SDO to GND or to VDDIO ---- */
#define BMI270_I2C_ADDRESS_LOW	0x68
#define BMI270_I2C_ADDRESS_HIGH	0x69

/* ---- timing ---- */
/*
 * Registers are accessible 450 us after POR (4.4); a soft reset has the same
 * timing (4.17). Bosch's bmi2_soft_reset() waits 2 ms; so does this driver.
 */
#define BMI270_RESET_WAIT_MS	2
/* After a CSB rising edge the interface is SPI "after 200 us" (6.3). */
#define BMI270_SPI_SWITCH_US	200
/* After PWR_CONF.adv_power_save = 0, wait at least 450 us (4.4). */
#define BMI270_APS_OFF_WAIT_US	450
/* Idle time after a write in normal mode (Table 19, Table 20). */
#define BMI270_WRITE_IDLE_US	2
/* INTERNAL_STATUS reads init_ok "after at most 20 msec" (4.4). */
#define BMI270_INIT_WAIT_MS	20

/* ---- the configuration file, 4.4 ---- */
#define BMI270_CONFIG_SIZE	8192

#endif /* BMI270_REGS_H */
