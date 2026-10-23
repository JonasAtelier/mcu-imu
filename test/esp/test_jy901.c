/*
 * The JY901 driver through src/imu_port_esp.c: the glue from ESP-IDF's UART
 * and I2C master driver calls to the fake module in ../jy901_cases.h, which
 * holds the module and every shared case. Here also: the clock the I2C
 * device is added at, and the clocks refused.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

static int bus_token;
#define TEST_UART	1
#define TEST_I2C	((i2c_master_bus_handle_t)(void *)&bus_token)

#define DEVICES_PER_PROBE	1

#include "../jy901_cases.h"

static uint32_t i2c_hz;

bool uart_is_driver_installed(uart_port_t port)
{
	return port == TEST_UART;
}

esp_err_t uart_flush_input(uart_port_t port)
{
	assert(port == TEST_UART);
	line_drop_stale();
	return ESP_OK;
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size)
{
	assert(port == TEST_UART);
	line_sent(src, size);
	return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
		    TickType_t ticks_to_wait)
{
	assert(port == TEST_UART && ticks_to_wait > 0);
	return (int)line_take(buf, length);
}

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address,
			   int timeout_ms)
{
	assert(bus == TEST_I2C && timeout_ms > 0);
	i2c_address_seen = (uint8_t)address;
	return i2c_absent ? ESP_ERR_NOT_FOUND : ESP_OK;
}

esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *config,
				    i2c_master_dev_handle_t *out)
{
	static int device;

	assert(bus == TEST_I2C && config->device_address == i2c_address_seen);
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
	assert(length >= 1);
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

/* 400 kHz by default and at most (chapter 7, p.46). */
static void test_i2c_clock(void)
{
	struct jy901_i2c_config config = i2c_config(0);
	struct imu_dev imu = { 0 };

	chip_reset();
	assert(jy901_probe_i2c(&imu, &config) == 0 && i2c_hz == 400000);
	jy901_remove(&imu);

	config.clock_hz = 100000;
	assert(jy901_probe_i2c(&imu, &config) == 0 && i2c_hz == 100000);
	jy901_remove(&imu);

	config.clock_hz = 400001;
	assert(jy901_probe_i2c(&imu, &config) == -EINVAL);
	assert(devices == 0 && !imu.ops);

	/* A port with no UART driver installed. */
	assert(jy901_probe_uart(&imu, 2, NULL) == -EINVAL);
}

int main(void)
{
	run_cases();
	test_i2c_clock();
	printf("all jy901 driver checks passed\n");
	return 0;
}
