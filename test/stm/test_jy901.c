/*
 * The JY901 driver through src/imu_port_stm.c: the glue from the HAL's UART
 * and I2C calls to the fake module in ../jy901_cases.h, which holds the
 * module and every shared case. Here also: the 7-bit address shifted for
 * the HAL, and the HAL's status as errno.
 */
#include <stdint.h>

#include "stm32_hal_stub.h"

static UART_HandleTypeDef uart_handle;
static I2C_HandleTypeDef i2c_handle;
#define TEST_UART	(&uart_handle)
#define TEST_I2C	(&i2c_handle)

#define DEVICES_PER_PROBE	0

#include "../jy901_cases.h"

static HAL_StatusTypeDef fail_status = HAL_TIMEOUT;

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

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr,
					uint32_t trials, uint32_t timeout)
{
	assert(h == &i2c_handle && trials >= 1 && timeout > 0);
	i2c_address_seen = (uint8_t)(addr >> 1);
	return i2c_absent ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr,
				   uint16_t reg, uint16_t reg_size,
				   uint8_t *data, uint16_t size,
				   uint32_t timeout)
{
	(void)timeout;
	assert(h == &i2c_handle && (addr >> 1) == i2c_address_seen &&
	       reg_size == I2C_MEMADD_SIZE_8BIT);
	return chip_i2c_read((uint8_t)reg, data, size) ? HAL_OK : fail_status;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr,
				    uint16_t reg, uint16_t reg_size,
				    uint8_t *data, uint16_t size,
				    uint32_t timeout)
{
	(void)timeout;
	assert(h == &i2c_handle && (addr >> 1) == i2c_address_seen &&
	       reg_size == I2C_MEMADD_SIZE_8BIT);
	return chip_i2c_write((uint8_t)reg, data, size) ? HAL_OK :
							  fail_status;
}

/* A HAL_ERROR mid-transfer is -EIO, not -ENXIO; HAL_BUSY is -EAGAIN. */
static void test_hal_status(void)
{
	struct jy901_i2c_config config = i2c_config(0);
	struct imu_dev imu = { 0 };

	fail_status = HAL_ERROR;
	chip_reset();
	i2c_fail_at = 1;
	assert(jy901_probe_i2c(&imu, &config) == -EIO && !imu.ops);
	fail_status = HAL_BUSY;
	chip_reset();
	i2c_fail_at = 2;
	assert(jy901_probe_i2c(&imu, &config) == -EAGAIN && !imu.ops);
	assert(allocs == frees);
	fail_status = HAL_TIMEOUT;

	/* No handle. */
	assert(jy901_probe_uart(&imu, NULL, NULL) == -EINVAL);
}

int main(void)
{
	run_cases();
	test_hal_status();
	printf("all jy901 driver checks passed\n");
	return 0;
}
