/*
 * Host-side check of the UART ops in src/imu_port_esp.c against
 * test/idf_stubs, framing by framing; I2C and SPI are covered by the chip
 * tests beside this one. Also that receive() is UART-only.
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "imu_priv.h"

#define UART_PORT 1

static esp_err_t flush_err;
static bool installed;
static char calls[8];		/* F flush, W write, R read, in order */
static uint8_t out[40];		/* what the last write sent */
static size_t out_len;
static uint8_t in[32];		/* what is waiting to be read */
static size_t in_len;
static uint8_t reply[32];	/* what the device sends after a write */
static size_t reply_len;
static TickType_t waited;	/* ticks_to_wait of the last read */
static int write_short, read_error;

static void reset(void)
{
	installed = true;
	calls[0] = '\0';
	out_len = in_len = reply_len = 0;
	write_short = read_error = 0;
	flush_err = ESP_OK;
}

static void called(char c)
{
	size_t n = strlen(calls);

	assert(n + 1 < sizeof(calls));
	calls[n] = c;
	calls[n + 1] = '\0';
}

bool uart_is_driver_installed(uart_port_t port)
{
	return installed && port == UART_PORT;
}

esp_err_t uart_flush_input(uart_port_t port)
{
	assert(port == UART_PORT);
	called('F');
	if (flush_err != ESP_OK)
		return flush_err;
	in_len = 0;
	return ESP_OK;
}

int uart_write_bytes(uart_port_t port, const void *src, size_t size)
{
	assert(port == UART_PORT && size <= sizeof(out));
	called('W');
	if (write_short)
		return 0;
	memcpy(out, src, size);
	out_len = size;
	/* The device answers after the command: queue its reply. */
	memcpy(in + in_len, reply, reply_len);
	in_len += reply_len;
	return (int)size;
}

int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
		    TickType_t ticks_to_wait)
{
	size_t n = length < in_len ? length : in_len;

	assert(port == UART_PORT);
	called('R');
	waited = ticks_to_wait;
	if (read_error)
		return -1;
	memcpy(buf, in, n);
	memmove(in, in + n, in_len - n);
	in_len -= n;
	return (int)n;
}

/* Enough of i2c_master to attach a bus and see it has no receive(). */
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
				    const i2c_device_config_t *config,
				    i2c_master_dev_handle_t *out_handle)
{
	(void)bus;
	(void)config;
	*out_handle = (i2c_master_dev_handle_t)(void *)calls;
	return ESP_OK;
}

esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t device)
{
	(void)device;
	return ESP_OK;
}

static void test_uart(void)
{
	static const uint8_t args[2] = { 0x11, 0x22 };
	uint8_t many[IMU_BUS_MAX_LEN + 1] = { 0 };
	struct imu_bus bus;
	uint8_t rx[2] = { 0 };

	/* The driver is the app's to install; without it, refuse. */
	reset();
	installed = false;
	assert(imu_uart_attach(&bus, UART_PORT, 100) == -EINVAL);
	installed = true;
	assert(imu_uart_attach(&bus, UART_PORT, 100) == 0);
	assert(bus.ops->kind == IMU_BUS_UART);

	/* A stale byte is dropped, then cmd and args go out, then the reply. */
	in[0] = 0xEE;
	in_len = 1;
	memcpy(reply, "\x12\x34", 2);
	reply_len = 2;
	assert(bus.ops->transfer(&bus, 0x13, args, 2, rx, 2) == 0);
	assert(strcmp(calls, "FWR") == 0);
	assert(out_len == 3 && out[0] == 0x13 && out[1] == 0x11 &&
	       out[2] == 0x22);
	assert(rx[0] == 0x12 && rx[1] == 0x34);
	/* 100 ms at the stub's 100 Hz: 10 ticks, plus one for the first. */
	assert(waited == 11);

	/* A command with no reply does not read. */
	reset();
	assert(bus.ops->transfer(&bus, 0x80, NULL, 0, NULL, 0) == 0);
	assert(strcmp(calls, "FW") == 0);

	/* A reply that stops short is a timeout. */
	reset();
	reply[0] = 0x12;
	reply_len = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -ETIMEDOUT);

	/* Driver failures. */
	reset();
	read_error = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EIO);
	reset();
	write_short = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EIO);
	assert(strcmp(calls, "FW") == 0);
	reset();
	flush_err = ESP_ERR_INVALID_ARG;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EINVAL);
	assert(strcmp(calls, "F") == 0);
	reset();
	assert(bus.ops->transfer(&bus, 0x13, many, sizeof(many), NULL, 0) ==
	       -EINVAL);
	assert(calls[0] == '\0');

	imu_bus_detach(&bus);
}

/* receive(): nothing sent, stale input dropped only when asked. */
static void test_receive(void)
{
	struct imu_bus bus, i2c;
	uint8_t rx[3] = { 0 };

	reset();
	assert(imu_uart_attach(&bus, UART_PORT, 100) == 0);

	in[0] = 0xEE;
	in[1] = 0x01;
	in_len = 2;
	assert(imu_bus_receive(&bus, rx, 2, false) == 0);
	assert(strcmp(calls, "R") == 0 && out_len == 0);
	assert(rx[0] == 0xEE && rx[1] == 0x01);
	assert(waited == 11);

	reset();
	in[0] = 0xEE;
	in_len = 1;
	assert(imu_bus_receive(&bus, rx, 1, true) == -ETIMEDOUT);
	assert(strcmp(calls, "FR") == 0);

	reset();
	in[0] = 0x55;
	in_len = 1;
	assert(imu_bus_receive(&bus, rx, 3, false) == -ETIMEDOUT);

	reset();
	read_error = 1;
	assert(imu_bus_receive(&bus, rx, 1, false) == -EIO);
	reset();
	flush_err = ESP_ERR_TIMEOUT;
	assert(imu_bus_receive(&bus, rx, 1, true) == -ETIMEDOUT);
	assert(strcmp(calls, "F") == 0);
	reset();
	assert(imu_bus_receive(&bus, rx, 0, false) == -EINVAL);
	assert(calls[0] == '\0');
	imu_bus_detach(&bus);

	/* I2C has no receive(). */
	assert(imu_i2c_attach(&i2c, (i2c_master_bus_handle_t)(void *)rx, 0x50,
			      400000, 10) == 0);
	assert(imu_bus_receive(&i2c, rx, 1, false) == -EOPNOTSUPP);
	imu_bus_detach(&i2c);
}

int main(void)
{
	test_uart();
	test_receive();
	printf("test_port_esp: all passed\n");
	return 0;
}
