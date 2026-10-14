/*
 * icm42688 -- TDK InvenSense ICM-42688-P 6-axis IMU, as an imu chip driver.
 *
 * A 3-axis gyroscope to 2000 dps and a 3-axis accelerometer to 16 g in one
 * die, both sampled at one rate up to 32 kHz, over I2C or SPI, hence two
 * probes. Both sensors run in low-noise mode, with the datasheet's
 * anti-alias filter and UI filter set at probe, and an optional data-ready
 * interrupt on INT1. Samples come out of imu_read(): temperature,
 * accelerometer and gyroscope from one burst.
 *
 * The chip takes changes of rate, full scale and power mode while it runs,
 * and nothing else (datasheet 12.9): the filters and the interrupt are set
 * by the probe, with both sensors off, and stay as set.
 *
 * Everything below that is ICM-42688-P specific takes the struct imu_dev a
 * probe filled, and returns -EINVAL if handed a device some other driver
 * probed. Register provenance is in src/icm42688/icm42688_regs.h. Nothing in
 * this driver has been run against a physical chip.
 */
#ifndef ICM42688_H
#define ICM42688_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#else
#include CMSIS_device_header
#endif

#include "imu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* AP_AD0 low or high (datasheet 9.3). */
#define ICM42688_I2C_ADDR_LOW	0x68
#define ICM42688_I2C_ADDR_HIGH	0x69

/* ACCEL_FS_SEL (14.38). The codes run backwards: 0 is the widest. */
enum icm42688_accel_fs {
	ICM42688_ACCEL_FS_16G	= 0,
	ICM42688_ACCEL_FS_8G	= 1,
	ICM42688_ACCEL_FS_4G	= 2,
	ICM42688_ACCEL_FS_2G	= 3,
};

/* GYRO_FS_SEL (14.37). */
enum icm42688_gyro_fs {
	ICM42688_GYRO_FS_2000DPS	= 0,
	ICM42688_GYRO_FS_1000DPS	= 1,
	ICM42688_GYRO_FS_500DPS		= 2,
	ICM42688_GYRO_FS_250DPS		= 3,
	ICM42688_GYRO_FS_125DPS		= 4,
	ICM42688_GYRO_FS_62_5DPS	= 5,
	ICM42688_GYRO_FS_31_25DPS	= 6,
	ICM42688_GYRO_FS_15_625DPS	= 7,
};

/*
 * GYRO_ODR and ACCEL_ODR (14.37-14.38), the rates both sensors take in
 * low-noise mode. One rate for both: at two rates the slower sensor's
 * registers hold invalid samples between its own (14.34).
 */
enum icm42688_odr {
	ICM42688_ODR_32KHZ	= 0x1,
	ICM42688_ODR_16KHZ	= 0x2,
	ICM42688_ODR_8KHZ	= 0x3,
	ICM42688_ODR_4KHZ	= 0x4,
	ICM42688_ODR_2KHZ	= 0x5,
	ICM42688_ODR_1KHZ	= 0x6,
	ICM42688_ODR_200HZ	= 0x7,
	ICM42688_ODR_100HZ	= 0x8,
	ICM42688_ODR_50HZ	= 0x9,
	ICM42688_ODR_25HZ	= 0xA,
	ICM42688_ODR_12_5HZ	= 0xB,
	ICM42688_ODR_500HZ	= 0xF,
};

/*
 * The UI filter's 3 dB bandwidth, *_UI_FILT_BW in low-noise mode (14.40,
 * 5.5); max(400 Hz, ODR) is written M.
 */
enum icm42688_ui_bw {
	ICM42688_UI_BW_ODR_2	= 0,	/* ODR/2 */
	ICM42688_UI_BW_M_4	= 1,	/* M/4, the reset value */
	ICM42688_UI_BW_M_5	= 2,
	ICM42688_UI_BW_M_8	= 3,
	ICM42688_UI_BW_M_10	= 4,
	ICM42688_UI_BW_M_16	= 5,
	ICM42688_UI_BW_M_20	= 6,
	ICM42688_UI_BW_M_40	= 7,
	ICM42688_UI_BW_LOW_LATENCY_400HZ = 14,	/* Dec2 at max(400 Hz, ODR) */
	ICM42688_UI_BW_LOW_LATENCY_8ODR	= 15,	/* Dec2 at max(200 Hz, 8 ODR) */
};

/* *_UI_FILT_ORD (14.39, 14.41, 5.5). The reset value is 2nd order. */
enum icm42688_ui_order {
	ICM42688_UI_ORDER_1	= 0,
	ICM42688_UI_ORDER_2	= 1,
	ICM42688_UI_ORDER_3	= 2,
};

/*
 * The anti-alias filter of either sensor (5.3): ICM42688_AAF_DEFAULT keeps
 * the chip's reset setting (585 Hz gyro, 1163 Hz accel), 1..63 picks the
 * row of the datasheet's table with that DELT code -- 42 Hz to 3979 Hz,
 * see icm42688_aaf_bandwidth_hz() -- and ICM42688_AAF_OFF bypasses it.
 */
#define ICM42688_AAF_DEFAULT	0
#define ICM42688_AAF_OFF	0xFF

/*
 * The data-ready interrupt on the INT1 pin (INT_CONFIG 14.3, INT_CONFIG0
 * 14.49, INT_CONFIG1 14.50, INT_SOURCE0 14.51). Off leaves the pin as
 * reset. Not latched, it is an 8 us pulse; latched, it holds until
 * INT_STATUS or the sensor data is read, which every imu_read() does.
 */
struct icm42688_int_config {
	bool enable;
	bool active_high;	/* else active low */
	bool open_drain;	/* else push-pull */
	bool latched;
};

struct icm42688_config {
	enum icm42688_accel_fs accel_fs;
	enum icm42688_gyro_fs gyro_fs;
	enum icm42688_odr odr;			/* both sensors */
	enum icm42688_ui_bw accel_ui_bw;
	enum icm42688_ui_bw gyro_ui_bw;
	enum icm42688_ui_order accel_ui_order;
	enum icm42688_ui_order gyro_ui_order;
	uint8_t accel_aaf;			/* ICM42688_AAF_*, or 1..63 */
	uint8_t gyro_aaf;
	struct icm42688_int_config data_ready_int1;
};

#ifdef ESP_PLATFORM
/*
 * Your application owns the I2C bus or SPI host and initialises it before
 * probing; the driver only adds and removes its own device.
 */
struct icm42688_i2c_config {
	i2c_master_bus_handle_t bus;
	/* 0 selects 0x68. */
	uint8_t address;
	/* 0 selects 400 kHz; at most 1 MHz (9.3). */
	uint32_t clock_hz;
	/* 0 selects 100 ms. */
	int timeout_ms;
};

struct icm42688_spi_config {
	/* The host must already be initialised with spi_bus_initialize(). */
	spi_host_device_t host;
	int cs_pin;
	/* 0 selects 8 MHz; at most 24 MHz (9.6). */
	uint32_t clock_hz;
	/* 0 or 3, the modes the chip takes out of reset (14.1). */
	uint8_t mode;
};
#else
/*
 * Your application owns the bus and HAL_*_Init()s it before probing; the
 * driver only keeps the handle. Bus speed and SPI mode are the handle's.
 */
struct icm42688_i2c_config {
	/* At most 1 MHz (9.3). */
	I2C_HandleTypeDef *bus;
	/* 7-bit; 0 selects 0x68. */
	uint8_t address;
	/* 0 selects 100 ms. */
	int timeout_ms;
};

struct icm42688_spi_config {
	/* Mode 0 or 3 with 8-bit frames, at most 24 MHz (14.1, 9.6). */
	SPI_HandleTypeDef *bus;
	/* A GPIO output the app has set up, idle high. */
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
};
#endif

/*
 * The probe first writes REG_BANK_SEL = 0 (0x76), then reads WHO_AM_I.
 * Check WHO_AM_I (0x47), soft-reset, apply the I2C settings of section 12.3
 * on I2C, then with both sensors off: the anti-alias filters, the UI
 * filters, ranges and rate, the interrupt; then both sensors on in
 * low-noise mode. Fills imu->name, ops and priv; leaves imu->mount alone.
 *
 * -EBUSY if imu is already bound (see struct imu_dev). -EINVAL for a bad
 * bus or chip config. -ENXIO when nothing answers at the I2C address;
 * -ENODEV when the wrong identity answers -- on SPI also a miswired bus. A
 * bus error as the port gives it. Nothing stays bound after a failure.
 */
#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
int icm42688_probe_i2c(struct imu_dev *imu,
		       const struct icm42688_i2c_config *bus_config,
		       const struct icm42688_config *config);
#endif
#if defined(ESP_PLATFORM) || defined(HAL_SPI_MODULE_ENABLED)
int icm42688_probe_spi(struct imu_dev *imu,
		       const struct icm42688_spi_config *bus_config,
		       const struct icm42688_config *config);
#endif

/* Both sensors off and the device released. Safe on an unprobed one. */
void icm42688_remove(struct imu_dev *imu);

/* The same burst as imu_read(), as counts. temp may be NULL. */
int icm42688_read_raw(struct imu_dev *imu, int16_t accel[3], int16_t gyro[3],
		      int16_t *temp);

/* INT_STATUS.DATA_RDY_INT. Reading INT_STATUS clears it (14.21). */
int icm42688_data_ready(struct imu_dev *imu, bool *ready);

/* Ranges or the rate after probe; the chip takes them while running. */
int icm42688_set_ranges(struct imu_dev *imu, enum icm42688_accel_fs accel_fs,
			enum icm42688_gyro_fs gyro_fs);
int icm42688_set_odr(struct imu_dev *imu, enum icm42688_odr odr);

/*
 * Either sensor on (low noise) or off. The gyroscope must stay on at least
 * 45 ms once on (14.36).
 */
int icm42688_set_power(struct imu_dev *imu, bool accel_on, bool gyro_on);

/* Counts per g and per dps for the ranges set; 0 if not ours. */
float icm42688_accel_sensitivity(struct imu_dev *imu);
float icm42688_gyro_sensitivity(struct imu_dev *imu);

/* The 3 dB bandwidth of AAF row delt, 1..63 (5.3); 0 for any other code. */
unsigned int icm42688_aaf_bandwidth_hz(uint8_t delt);

/*
 * Escape hatch for the registers this driver does not model: the FIFO,
 * APEX, the notch filter, offsets, self-test. bank is 0, 1, 2 or 4, the
 * user banks of chapter 13; any other is -EINVAL. The driver returns
 * REG_BANK_SEL to bank 0 after, and tries to even when the access failed.
 * len is at most 14.
 */
int icm42688_reg_read(struct imu_dev *imu, uint8_t bank, uint8_t reg,
		      void *buf, size_t len);
int icm42688_reg_write(struct imu_dev *imu, uint8_t bank, uint8_t reg,
		       uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
