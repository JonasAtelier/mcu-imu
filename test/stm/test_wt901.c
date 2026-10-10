/*
 * The WT901 driver through src/imu_port_stm.c: the glue from the HAL's UART
 * calls to the fake line in ../wt901_cases.h, which holds the module and
 * every case.
 */
#include <stdint.h>

#include "stm32_hal_stub.h"

static UART_HandleTypeDef uart_handle;
#define TEST_UART	(&uart_handle)

#include "../wt901_cases.h"

/* RXNE: a stale byte is waiting; reading the data register takes it. */
int stub_uart_get_flag(UART_HandleTypeDef *h, uint32_t flag)
{
	assert(h == &uart_handle && flag == UART_FLAG_RXNE);
	return line_has_stale();
}

void stub_uart_clear_ore(UART_HandleTypeDef *h)
{
	assert(h == &uart_handle);
}

void stub_uart_flush(UART_HandleTypeDef *h)
{
	uint8_t b;

	assert(h == &uart_handle);
	line_take(&b, 1);
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *data,
				    uint16_t size, uint32_t timeout)
{
	assert(h == &uart_handle && timeout > 0);
	line_sent(data, size);
	return HAL_OK;
}

/* Blocking receive: all size bytes, or HAL_TIMEOUT. */
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *data,
				   uint16_t size, uint32_t timeout)
{
	assert(h == &uart_handle && timeout > 0);
	return line_take(data, size) == size ? HAL_OK : HAL_TIMEOUT;
}
