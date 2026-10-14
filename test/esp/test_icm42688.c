/*
 * The ICM-42688-P on ESP-IDF: icm42688.c through src/imu_port_esp.c against
 * fake I2C and SPI master drivers in front of the fake chip in
 * ../icm42688_cases.h. Here also: what the drivers are given -- address,
 * clock, SPI mode -- and the bus configs refused.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "icm42688.h"
#include "icm42688_math.h"
#include "icm42688_regs.h"
#include "imu.h"

enum bus { BUS_I2C, BUS_SPI };

static int allocs, frees;

#include "../icm42688_cases.h"

static int bus_token;
#define I2C_BUS	((i2c_master_bus_handle_t)(void *)&bus_token)
static bool absent;			/* nothing ACKs on I2C */
static uint16_t i2c_address;
static uint32_t i2c_hz;
static int spi_mode, spi_hz, devices;

/* Counts the driver's calloc() and free(); -Wl,--wrap in the Makefile. */
void *__real_calloc(size_t n, size_t size);
void *__wrap_calloc(size_t n, size_t size)
{
	allocs++;
	return __real_calloc(n, size);
}

void __real_free(void *p);
void __wrap_free(void *p)
{
	if (p)
		frees++;
	__real_free(p);
}

/* The fake clock: each wait moves it by the least it can last. */
void esp_rom_delay_us(uint32_t us)
{
	now_us += us;
}

/* n ticks at 100 Hz can be as little as n - 1 whole periods. */
void vTaskDelay(TickType_t ticks)
{
	if (ticks > 1)
		now_us += (int64_t)(ticks - 1) * 1000000 / configTICK_RATE_HZ;
}

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address,
			   int timeout_ms)
{
	(void)bus;
	(void)timeout_ms;
	i2c_address = address;
	return absent ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *config,
				    i2c_master_dev_handle_t *out)
{
	static int device;

	(void)bus;
	assert(config->device_address == i2c_address);
	i2c_hz = config->scl_speed_hz;
	*out = (i2c_master_dev_handle_t)(void *)&device;
	devices++;
	return ESP_OK;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
	(void)device;
	devices--;
	return ESP_OK;
}

esp_err_t i2c_master_transmit(i2c_master_dev_handle_t device,
			      const uint8_t *data, size_t length,
			      int timeout_ms)
{
	(void)device;
	(void)timeout_ms;
	assert(length >= 2);
	return chip_i2c_write(data[0], data + 1, length - 1) ? ESP_OK :
							     ESP_ERR_TIMEOUT;
}

esp_err_t i2c_master_transmit_receive(i2c_master_dev_handle_t device,
				      const uint8_t *tx, size_t tx_length,
				      uint8_t *rx, size_t rx_length,
				      int timeout_ms)
{
	(void)device;
	(void)timeout_ms;
	assert(tx_length == 1);
	return chip_i2c_read(tx[0], rx, rx_length) ? ESP_OK :
						      ESP_ERR_TIMEOUT;
}

esp_err_t spi_bus_add_device(spi_host_device_t host,
			     const spi_device_interface_config_t *config,
			     spi_device_handle_t *out)
{
	static int device;

	(void)host;
	spi_mode = config->mode;
	spi_hz = config->clock_speed_hz;
	*out = (spi_device_handle_t)(void *)&device;
	devices++;
	return ESP_OK;
}

esp_err_t spi_bus_remove_device(spi_device_handle_t device)
{
	(void)device;
	devices--;
	return ESP_OK;
}

/* spi_master drives CS itself: low for the frame, high after it. */
esp_err_t spi_device_polling_transmit(spi_device_handle_t device,
				      spi_transaction_t *t)
{
	bool ok;

	(void)device;
	ok = chip_spi_frame(t->tx_buffer, t->rx_buffer, t->length / 8);
	return ok ? ESP_OK : ESP_ERR_TIMEOUT;
}

static int probe_on(struct imu_dev *imu, enum bus bus,
		    const struct icm42688_config *config)
{
	if (bus == BUS_I2C) {
		struct icm42688_i2c_config c = { .bus = I2C_BUS };

		return icm42688_probe_i2c(imu, &c, config);
	} else {
		struct icm42688_spi_config c = {
			.host = SPI2_HOST, .cs_pin = 5,
		};

		return icm42688_probe_spi(imu, &c, config);
	}
}

/* What the I2C and SPI drivers are told, and the bus configs refused. */
static void test_bus_configs(void)
{
	struct imu_dev imu = { 0 };
	struct icm42688_i2c_config i2c = { .bus = I2C_BUS };
	struct icm42688_spi_config spi = { .host = SPI2_HOST, .cs_pin = 5 };

	/* Defaults: 0x68 at 400 kHz; SPI mode 0 at 8 MHz */
	chip_power_on();
	assert(icm42688_probe_i2c(&imu, &i2c, &base) == 0);
	assert(i2c_address == 0x68 && i2c_hz == 400000 && devices == 1);
	icm42688_remove(&imu);
	assert(devices == 0);
	chip_power_on();
	assert(icm42688_probe_spi(&imu, &spi, &base) == 0);
	assert(spi_mode == 0 && spi_hz == 8000000 && devices == 1);
	icm42688_remove(&imu);
	assert(devices == 0);

	/* The other address, Fast-mode Plus, SPI mode 3 at 24 MHz */
	chip_power_on();
	i2c.address = 0x69;
	i2c.clock_hz = 1000000;
	assert(icm42688_probe_i2c(&imu, &i2c, &base) == 0);
	assert(i2c_address == 0x69 && i2c_hz == 1000000);
	icm42688_remove(&imu);
	chip_power_on();
	spi.mode = 3;
	spi.clock_hz = 24000000;
	assert(icm42688_probe_spi(&imu, &spi, &base) == 0);
	assert(spi_mode == 3 && spi_hz == 24000000);
	icm42688_remove(&imu);

	/* Refused, nothing allocated or added */
	allocs = 0;
	i2c = (struct icm42688_i2c_config){ .bus = I2C_BUS, .address = 0x6A };
	assert(icm42688_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	i2c = (struct icm42688_i2c_config){
		.bus = I2C_BUS, .clock_hz = 1000001,
	};
	assert(icm42688_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	i2c = (struct icm42688_i2c_config){ .bus = NULL };
	assert(icm42688_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	assert(icm42688_probe_i2c(&imu, NULL, &base) == -EINVAL);
	spi = (struct icm42688_spi_config){ .cs_pin = 5, .mode = 1 };
	assert(icm42688_probe_spi(&imu, &spi, &base) == -EINVAL);
	spi = (struct icm42688_spi_config){ .cs_pin = 5, .clock_hz = 24000001 };
	assert(icm42688_probe_spi(&imu, &spi, &base) == -EINVAL);
	spi = (struct icm42688_spi_config){ .cs_pin = -1 };
	assert(icm42688_probe_spi(&imu, &spi, &base) == -EINVAL);
	assert(icm42688_probe_spi(&imu, NULL, &base) == -EINVAL);
	assert(allocs == 0 && devices == 0);

	/* Nothing at the address: -ENXIO, before anything is allocated */
	chip_power_on();
	absent = true;
	i2c = (struct icm42688_i2c_config){ .bus = I2C_BUS };
	assert(icm42688_probe_i2c(&imu, &i2c, &base) == -ENXIO);
	assert(allocs == 0 && devices == 0 && chip.transfers == 0);
	absent = false;
}

int main(void)
{
	run_icm42688_cases();
	test_bus_configs();
	printf("test_icm42688 (esp): all passed\n");
	return 0;
}
