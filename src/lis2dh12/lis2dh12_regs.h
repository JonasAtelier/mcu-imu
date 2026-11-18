/*
 * LIS2DH12 registers, bit fields and values this driver uses.
 *
 * Source: ST's LIS2DH12 datasheet, DocID025056 (rev. 6 or later), which is
 * not in docs/ yet. Every value below was written from the register map as
 * remembered, NOT read off the PDF: each one is on the UNVERIFIED list in
 * docs/chips.md "LIS2DH12", and the section numbers (chapter 9, "Register
 * description", one section per register) are part of what is to be
 * checked. Nothing here has been run against a physical chip.
 *
 * The part has no soft reset of its user registers (CTRL_REG5.BOOT only
 * reloads the trimming), so the probe writes every control register it
 * relies on instead of trusting reset values a previous run may have left.
 */
#ifndef LIS2DH12_REGS_H
#define LIS2DH12_REGS_H

/* ---- register addresses, chapter 8 "Register mapping" ---- */
#define LIS2DH12_WHO_AM_I	0x0F	/* 9.4 */
#define LIS2DH12_CTRL_REG0	0x1E	/* 9.5 */
#define LIS2DH12_TEMP_CFG_REG	0x1F	/* 9.6 */
#define LIS2DH12_CTRL_REG1	0x20	/* 9.7 */
#define LIS2DH12_CTRL_REG2	0x21	/* 9.8 */
#define LIS2DH12_CTRL_REG3	0x22	/* 9.9 */
#define LIS2DH12_CTRL_REG4	0x23	/* 9.10 */
#define LIS2DH12_CTRL_REG5	0x24	/* 9.11 */
#define LIS2DH12_CTRL_REG6	0x25	/* 9.12 */
#define LIS2DH12_REFERENCE	0x26	/* 9.13 */
#define LIS2DH12_STATUS_REG	0x27	/* 9.14 */
#define LIS2DH12_OUT_X_L	0x28	/* 9.15, then Y_L, Z_L .. 0x2D */
#define LIS2DH12_FIFO_CTRL_REG	0x2E	/* 9.21 */
#define LIS2DH12_INT1_CFG	0x30	/* 9.23 */
#define LIS2DH12_INT1_SRC	0x31	/* 9.24 */
#define LIS2DH12_INT1_THS	0x32	/* 9.25 */
#define LIS2DH12_INT1_DURATION	0x33	/* 9.26 */
#define LIS2DH12_INT2_CFG	0x34	/* 9.27 */
#define LIS2DH12_CLICK_CFG	0x38	/* 9.31 */
#define LIS2DH12_ACT_THS	0x3E	/* 9.37 */

#define LIS2DH12_WHO_AM_I_VALUE	0x33	/* 9.4 */

/*
 * I2C: bit 7 of the sub-address asks for auto-increment, so a burst from
 * OUT_X_L reads X, Y and Z; without it every byte repeats the first
 * register (6.1.1, "I2C operation").
 */
#define LIS2DH12_AUTO_INCREMENT	0x80
#define LIS2DH12_BURST_LEN	6	/* OUT_X_L .. OUT_Z_H, little endian */

/* ---- I2C addresses, 6.1.1: SDO/SA0 low or high ---- */
#define LIS2DH12_I2C_ADDRESS_LOW	0x18
#define LIS2DH12_I2C_ADDRESS_HIGH	0x19

/*
 * ---- CTRL_REG0 (0x1E), 9.5 ----
 * Bits 6:0 "must be set to 0010000 for the correct operation of the
 * device"; bit 7, SDO_PU_DISC, disconnects the SDO/SA0 pull-up.
 */
#define LIS2DH12_CTRL_REG0_FIXED	0x10
#define LIS2DH12_SDO_PU_DISC		(1u << 7)

/* ---- TEMP_CFG_REG (0x1F), 9.6: TEMP_EN[1:0] in 7:6; 0 = sensor off ---- */
#define LIS2DH12_TEMP_OFF		0x00

/* ---- CTRL_REG1 (0x20), 9.7 ---- */
#define LIS2DH12_ODR_SHIFT		4	/* ODR[3:0] in bits 7:4 */
#define LIS2DH12_LPEN			(1u << 3)
#define LIS2DH12_XYZ_EN			0x07	/* Zen, Yen, Xen */

/*
 * ---- CTRL_REG2 (0x21), 9.8 ----
 * HPM[1:0] (7:6) = 00, "normal mode (reset by reading REFERENCE)";
 * HPCF[2:1] (5:4) the cut-off; FDS (3) = 0 keeps the output registers
 * unfiltered; HP_IA1 (0) filters what interrupt generator 1 sees.
 */
#define LIS2DH12_HPCF_SHIFT		4
#define LIS2DH12_HP_IA1			(1u << 0)

/* ---- CTRL_REG3 (0x22), 9.9 ---- */
#define LIS2DH12_I1_IA1			(1u << 6)

/* ---- CTRL_REG4 (0x23), 9.10 ---- */
#define LIS2DH12_BDU			(1u << 7)
#define LIS2DH12_FS_SHIFT		4	/* FS[1:0] in bits 5:4 */
#define LIS2DH12_HR			(1u << 3)

/* ---- CTRL_REG5 (0x24), 9.11 ---- */
#define LIS2DH12_LIR_INT1		(1u << 3)

/* ---- CTRL_REG6 (0x25), 9.12: 1 = INT1 and INT2 active low ---- */
#define LIS2DH12_INT_POLARITY		(1u << 1)

/* ---- STATUS_REG (0x27), 9.14 ---- */
#define LIS2DH12_ZYXDA			(1u << 3)

/* ---- INT1_CFG (0x30), 9.23: AOI = 0 and 6D = 0, the OR of the events ---- */
#define LIS2DH12_XHIE			(1u << 1)
#define LIS2DH12_YHIE			(1u << 3)
#define LIS2DH12_ZHIE			(1u << 5)

/* ---- INT1_SRC (0x31), 9.24: read-only, reading it clears a latched IA ---- */
#define LIS2DH12_IA			(1u << 6)
#define LIS2DH12_ZH			(1u << 5)
#define LIS2DH12_YH			(1u << 3)
#define LIS2DH12_XH			(1u << 1)

/* ---- INT1_THS (0x32) and INT1_DURATION (0x33), 9.25-9.26: bit 7 = 0 ---- */
#define LIS2DH12_7BIT_MAX		0x7F

#endif /* LIS2DH12_REGS_H */
