/*
 * The LIS2DH12 on the STM32 HAL: lis2dh12.c through src/imu_port_stm.c
 * against fake HAL I2C calls in front of the fake chip in
 * ../lis2dh12_cases.h. Here also: the 7-bit address shifted for the HAL,
 * and the HAL's status as errno.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "imu.h"
#include "lis2dh12.h"
#include "lis2dh12_math.h"
#include "lis2dh12_regs.h"

static int allocs, frees;

#include "../lis2dh12_cases.h"

static I2C_HandleTypeDef hi2c;
static bool absent;
static uint8_t i2c_address;
static HAL_StatusTypeDef fail_status = HAL_TIMEOUT;

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

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *h, uint16_t addr,
					uint32_t trials, uint32_t timeout)
{
	assert(h == &hi2c && trials >= 1 && timeout > 0);
	i2c_address = (uint8_t)(addr >> 1);
	return absent ? HAL_ERROR : HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *h, uint16_t addr,
				   uint16_t reg, uint16_t reg_size,
				   uint8_t *data, uint16_t size,
				   uint32_t timeout)
{
	(void)timeout;
	assert(h == &hi2c && (addr >> 1) == i2c_address &&
	       reg_size == I2C_MEMADD_SIZE_8BIT);
	return chip_i2c_read((uint8_t)reg, data, size) ? HAL_OK : fail_status;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t addr,
				    uint16_t reg, uint16_t reg_size,
				    uint8_t *data, uint16_t size,
				    uint32_t timeout)
{
	(void)timeout;
	assert(h == &hi2c && (addr >> 1) == i2c_address &&
	       reg_size == I2C_MEMADD_SIZE_8BIT);
	return chip_i2c_write((uint8_t)reg, data, size) ? HAL_OK :
							  fail_status;
}

static int probe_on(struct imu_dev *imu, const struct lis2dh12_config *config)
{
	struct lis2dh12_i2c_config c = { .bus = &hi2c };

	return lis2dh12_probe_i2c(imu, &c, config);
}

static void test_bus_configs(void)
{
	struct imu_dev imu = { 0 };
	struct lis2dh12_i2c_config i2c = { .bus = &hi2c };

	/* 0x18 by default, 0x19 if asked */
	chip_power_on();
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == 0);
	assert(i2c_address == 0x18);
	lis2dh12_remove(&imu);
	chip_power_on();
	i2c.address = 0x19;
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == 0);
	assert(i2c_address == 0x19);
	lis2dh12_remove(&imu);

	/* Refused, nothing allocated */
	allocs = 0;
	i2c.address = 0x68;
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	i2c = (struct lis2dh12_i2c_config){ .bus = &hi2c, .timeout_ms = -1 };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	i2c = (struct lis2dh12_i2c_config){ .bus = NULL };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EINVAL);
	assert(allocs == 0);

	/* Nothing at the address: -ENXIO, nothing allocated */
	chip_power_on();
	absent = true;
	i2c = (struct lis2dh12_i2c_config){ .bus = &hi2c };
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -ENXIO);
	assert(allocs == 0 && chip.transfers == 0);
	absent = false;

	/* The HAL's status as errno */
	chip_power_on();
	chip.fail_in = 0;
	fail_status = HAL_BUSY;
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EAGAIN);
	chip_power_on();
	chip.fail_in = 0;
	fail_status = HAL_ERROR;
	assert(lis2dh12_probe_i2c(&imu, &i2c, &base) == -EIO);
	fail_status = HAL_TIMEOUT;
}

int main(void)
{
	run_lis2dh12_cases();
	test_bus_configs();
	printf("test_lis2dh12 (stm): all passed\n");
	return 0;
}
