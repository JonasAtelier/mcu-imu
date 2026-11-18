/*
 * The LIS2DH12 on ESP-IDF: lis2dh12.c through src/imu_port_esp.c against a
 * fake I2C master driver in front of the fake chip in ../lis2dh12_cases.h.
 * Here also: what the driver is given -- address, clock -- and the bus
 * configs refused. The driver never waits: the weak vTaskDelay() and
 * esp_rom_delay_us() of idf_stubs abort if it does.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#include "imu.h"
#include "lis2dh12.h"
#include "lis2dh12_math.h"
#include "lis2dh12_regs.h"

static int allocs, frees;

#include "../lis2dh12_cases.h"

static int bus_token;
#define I2C_BUS	((i2c_master_bus_handle_t)(void *)&bus_token)
static bool absent;			/* nothing ACKs */
static uint16_t i2c_address;
static uint32_t i2c_hz;
static int devices;

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

	assert(bus == I2C_BUS && config->device_address == i2c_address);
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

static int probe_on(struct imu_dev *imu, const struct lis2dh12_config *config)
{
	struct lis2dh12_i2c_config c = { .bus = I2C_BUS };

	return lis2dh12_probe_i2c(imu, &c, config);
}

static void test_bus_configs(void)
{
	struct imu_dev imu = { 0 };
	struct lis2dh12_i2c_config i2c = { .bus = I2C_BUS };

	/* Defaults: 0x18 at 400 kHz */
	chip_power_on();
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == 0);
	assert(i2c_address == 0x18 && i2c_hz == 400000 && devices == 1);
	lis2dh12_remove(&imu);
	assert(devices == 0);

	/* SA0 high, standard mode */
	chip_power_on();
	i2c.address = 0x19;
	i2c.clock_hz = 100000;
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == 0);
	assert(i2c_address == 0x19 && i2c_hz == 100000);
	lis2dh12_remove(&imu);

	/* Refused, nothing allocated or added */
	allocs = 0;
	i2c = (struct lis2dh12_i2c_config){ .bus = I2C_BUS, .address = 0x1A };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	i2c = (struct lis2dh12_i2c_config){ .bus = I2C_BUS,
					    .clock_hz = 400001 };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	i2c = (struct lis2dh12_i2c_config){ .bus = NULL };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	assert(lis2dh12_probe_i2c(&imu, NULL, &base) == -EINVAL);
	assert(allocs == 0 && devices == 0);

	/* Nothing at the address: -ENXIO, before anything is allocated */
	chip_power_on();
	absent = true;
	i2c = (struct lis2dh12_i2c_config){ .bus = I2C_BUS };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -ENXIO);
	assert(allocs == 0 && devices == 0 && chip.transfers == 0);
	absent = false;
}

int main(void)
{
	run_lis2dh12_cases();
	test_bus_configs();
	assert(devices == 0);
	printf("test_lis2dh12 (esp): all passed\n");
	return 0;
}
