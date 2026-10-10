/*
 * The WT901 driver through src/imu_port_esp.c: the glue from ESP-IDF's
 * UART driver calls to the fake line in ../wt901_cases.h, which holds the
 * module and every case.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_log.h"

#define TEST_UART	1

#include "../wt901_cases.h"

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
