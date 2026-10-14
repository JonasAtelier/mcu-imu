/*
 * Shared by the chip drivers inside this library. Not public API.
 *
 * Two layers sit under the chip drivers, the way regmap sits under a kernel
 * driver:
 *
 *   - The bus kind is chosen at run time. A chip holds a struct imu_bus per
 *     device it talks to -- the BMI088 one per die -- and reaches it only
 *     through bus->ops->transfer(), so one driver can sit on I2C or SPI,
 *     and a command/response part on a UART. Every category library offers
 *     all three kinds, used or not; no imu chip uses the UART yet.
 *   - The platform is chosen at link time. imu_port_esp.c and
 *     imu_port_stm.c each define the ops tables, the attach functions, the
 *     clock and the waits below for their platform; the other one compiles
 *     to nothing.
 *
 * ESP_PLATFORM, which ESP-IDF defines for every component, and the HAL's own
 * module switches are defined or not, never 0 or 1, so they are the one place
 * #ifdef is right; this library's CONFIG_IMU_* options are always tested with
 * #if.
 */
#ifndef IMU_PRIV_H
#define IMU_PRIV_H

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define IMU_G			9.80665f	/* standard gravity, m/s^2 */
#define IMU_DEG_TO_RAD		0.017453292519943295f

/* The most bytes after cmd: tx on I2C and UART, tx and rx together on SPI. */
#define IMU_BUS_MAX_LEN		31

struct imu_bus;

enum imu_bus_kind {
	IMU_BUS_I2C,
	IMU_BUS_SPI,
	IMU_BUS_UART,
};

struct imu_bus_ops {
	enum imu_bus_kind kind;
	/*
	 * One transaction: cmd, then tx_len bytes of tx, then rx_len bytes
	 * into rx. cmd is the register address, with any SPI read bit already
	 * set by the chip, and a command byte on a UART.
	 *
	 *   I2C: rx_len ? cmd, repeated start, read : cmd and tx, one write.
	 *        Sending tx and then reading, or neither, is -EINVAL.
	 *   SPI: CS low, cmd, tx, rx, CS high -- high again even when the
	 *        transfer failed, because that rising edge is part of the
	 *        protocol: it latches a write, and it is what switches a BMI088
	 *        accelerometer to SPI.
	 *   UART: drop stale input, send cmd and tx, then wait for rx_len
	 *        bytes; fewer within the timeout is -ETIMEDOUT.
	 *
	 * On SPI more than IMU_BUS_MAX_LEN bytes after cmd is -EINVAL; on I2C
	 * and UART only tx is bounded so, and rx on the HAL by its 16-bit
	 * count. 0 or a negative errno.
	 */
	int (*transfer)(struct imu_bus *bus, uint8_t cmd, const uint8_t *tx,
			size_t tx_len, uint8_t *rx, size_t rx_len);
	/*
	 * UART only; NULL on I2C and SPI. Wait for rx_len bytes with nothing
	 * sent, for a part that streams on its own instead of answering a
	 * command. drop_stale first drops whatever is already waiting, to
	 * start on fresh data. Fewer than rx_len within the timeout is
	 * -ETIMEDOUT; rx is then undefined.
	 */
	int (*receive)(struct imu_bus *bus, uint8_t *rx, size_t rx_len,
		       bool drop_stale);
	/* Undo the attach. The bus itself is the app's and is left alone. */
	void (*detach)(struct imu_bus *bus);
};

#ifdef ESP_PLATFORM
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

/* The I2C argument of every I2C probe on this platform. */
typedef i2c_master_bus_handle_t imu_i2c_bus;

/*
 * One device on one bus, as its driver holds it. IMU_HAVE_BUS says the
 * platform has at least one bus kind, so the struct exists.
 */
#define IMU_HAVE_BUS
struct imu_bus {
	const struct imu_bus_ops *ops;
	uint32_t timeout_ms;
	union {
		i2c_master_dev_handle_t i2c;
		spi_device_handle_t spi;
		uart_port_t uart;
	};
};

#else /* STM32 HAL */
#include CMSIS_device_header

#if !defined(HAL_I2C_MODULE_ENABLED) && CONFIG_IMU_MPU6050
#error "IMU_MPU6050 needs the HAL I2C module: #define HAL_I2C_MODULE_ENABLED in the app's stm32f{4,7}xx_hal_conf.h, with stm32f{4,7}xx_hal_i2c.c in the build"
#endif
#if !defined(HAL_SPI_MODULE_ENABLED) && CONFIG_IMU_ICM45686
#error "IMU_ICM45686 needs the HAL SPI module: #define HAL_SPI_MODULE_ENABLED in the app's stm32f{4,7}xx_hal_conf.h, with stm32f{4,7}xx_hal_spi.c in the build"
#endif
#if !defined(HAL_UART_MODULE_ENABLED) && CONFIG_IMU_WT901
#error "IMU_WT901 needs the HAL UART module: #define HAL_UART_MODULE_ENABLED in the app's stm32f{4,7}xx_hal_conf.h, with stm32f{4,7}xx_hal_uart.c in the build"
#endif
#if (!defined(HAL_I2C_MODULE_ENABLED) || !defined(HAL_SPI_MODULE_ENABLED)) && \
    CONFIG_IMU_BMI088
#error "IMU_BMI088 carries both transports and needs both HAL modules: #define HAL_I2C_MODULE_ENABLED and HAL_SPI_MODULE_ENABLED in the app's stm32f{4,7}xx_hal_conf.h, with stm32f{4,7}xx_hal_i2c.c and _spi.c in the build"
#endif
/* One probe per transport, each built when its module is: one is enough. */
#if !defined(HAL_I2C_MODULE_ENABLED) && !defined(HAL_SPI_MODULE_ENABLED) && \
    CONFIG_IMU_BMI270
#error "IMU_BMI270 needs the HAL I2C or SPI module, for bmi270_probe_i2c() or bmi270_probe_spi(): #define HAL_I2C_MODULE_ENABLED or HAL_SPI_MODULE_ENABLED in the app's stm32f{4,7}xx_hal_conf.h, with stm32f{4,7}xx_hal_i2c.c or _spi.c in the build"
#endif

#ifdef HAL_I2C_MODULE_ENABLED
typedef I2C_HandleTypeDef *imu_i2c_bus;
#endif
#if defined(HAL_I2C_MODULE_ENABLED) || defined(HAL_SPI_MODULE_ENABLED) || \
    defined(HAL_UART_MODULE_ENABLED)
#define IMU_HAVE_BUS
struct imu_bus {
	const struct imu_bus_ops *ops;
	uint32_t timeout_ms;
	union {
#ifdef HAL_I2C_MODULE_ENABLED
		struct {
			I2C_HandleTypeDef *handle;
			uint8_t address;	/* 7-bit */
		} i2c;
#endif
#ifdef HAL_SPI_MODULE_ENABLED
		struct {
			SPI_HandleTypeDef *handle;
			GPIO_TypeDef *cs_port;	/* app's output, idle high */
			uint16_t cs_pin;
		} spi;
#endif
#ifdef HAL_UART_MODULE_ENABLED
		UART_HandleTypeDef *uart;
#endif
	};
};
#endif

#ifdef HAL_SPI_MODULE_ENABLED
/*
 * SPI mode 0-3 of a handle the app initialised, or -1 if it does not move
 * 8-bit frames. The handle's Init is what HAL_SPI_Init() programmed.
 */
static inline int imu_spi_mode(const SPI_HandleTypeDef *spi)
{
	if (spi->Init.DataSize != SPI_DATASIZE_8BIT)
		return -1;
	return (spi->Init.CLKPolarity == SPI_POLARITY_HIGH ? 2 : 0) |
	       (spi->Init.CLKPhase == SPI_PHASE_2EDGE ? 1 : 0);
}
#endif

#endif /* STM32 HAL */

#ifdef IMU_HAVE_BUS
/* Register helpers on top of transfer(). */
static inline int imu_reg_read(struct imu_bus *bus, uint8_t reg, void *buf,
			       size_t len)
{
	return bus->ops->transfer(bus, reg, NULL, 0, buf, len);
}

static inline int imu_reg_write(struct imu_bus *bus, uint8_t reg,
				uint8_t value)
{
	return bus->ops->transfer(bus, reg, &value, 1, NULL, 0);
}

/* bus->ops->receive(), or -EOPNOTSUPP on a kind that has none. */
static inline int imu_bus_receive(struct imu_bus *bus, uint8_t *rx,
				  size_t rx_len, bool drop_stale)
{
	if (!bus->ops->receive)
		return -EOPNOTSUPP;
	return bus->ops->receive(bus, rx, rx_len, drop_stale);
}

static inline void imu_bus_detach(struct imu_bus *bus)
{
	bus->ops->detach(bus);
}
#endif

#if defined(ESP_PLATFORM) || defined(HAL_I2C_MODULE_ENABLED)
/*
 * Is anything at this 7-bit address? -ENXIO when nothing ACKs -- nothing at
 * that address -- not -ENODEV, which means something answered but it is the
 * wrong chip. A NACK in the middle of a transfer is a plain -EIO on both
 * platforms, which is why every I2C probe decides presence here, never by
 * its first transfer. SPI has no ACK at all: an absent SPI chip reads back a
 * wrong ID, -ENODEV.
 */
int imu_i2c_probe(imu_i2c_bus handle, uint8_t address, uint32_t timeout_ms);

/*
 * Bind bus to the I2C device at address. On ESP-IDF the driver adds the
 * device at scl_hz, which the caller has defaulted; on the HAL the clock is
 * the handle's and scl_hz is ignored. Undo with imu_bus_detach(). Every
 * attach leaves bus->ops NULL when it fails, so a bus with ops is one with
 * something to detach.
 */
int imu_i2c_attach(struct imu_bus *bus, imu_i2c_bus handle, uint8_t address,
		   uint32_t scl_hz, uint32_t timeout_ms);
#endif

#ifdef ESP_PLATFORM
/*
 * Add the SPI device on host's cs_pin, in SPI mode 0-3 at clock_hz. The
 * host is the app's, already spi_bus_initialize()d.
 */
int imu_spi_attach(struct imu_bus *bus, spi_host_device_t host, int cs_pin,
		   uint32_t clock_hz, uint8_t mode);

/* port's driver is the app's, already installed and configured. */
int imu_uart_attach(struct imu_bus *bus, uart_port_t port,
		    uint32_t timeout_ms);
#else
#ifdef HAL_SPI_MODULE_ENABLED
/*
 * Bind bus to the SPI device behind cs_port/cs_pin, and deselect it, so the
 * first transfer starts on a falling edge. Mode, clock and frame are the
 * handle's; the chip select is an output the app has set up.
 */
int imu_spi_attach(struct imu_bus *bus, SPI_HandleTypeDef *handle,
		   GPIO_TypeDef *cs_port, uint16_t cs_pin,
		   uint32_t timeout_ms);
#endif
#ifdef HAL_UART_MODULE_ENABLED
/* handle is the app's, already HAL_UART_Init()ed. */
int imu_uart_attach(struct imu_bus *bus, UART_HandleTypeDef *handle,
		    uint32_t timeout_ms);
#endif
#endif

/* Block for at least ms milliseconds: long, never short. */
void imu_sleep_ms(uint32_t ms);

/* Wait at least usec microseconds; may be much longer below a millisecond. */
void imu_delay_us(uint32_t usec);

/*
 * Between two polls of a status bit: on ESP-IDF a yield of one tick, not a
 * timed wait; on the HAL, which has no scheduler, one millisecond.
 */
void imu_yield(void);

#endif /* IMU_PRIV_H */
