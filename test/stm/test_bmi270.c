/*
 * The BMI270 on the STM32 HAL: bmi270.c through src/imu_port_stm.c against
 * fake HAL I2C and SPI calls and a chip-select GPIO, in front of the fake
 * chip in ../bmi270_cases.h. Here also: the SPI handle and chip-select
 * checks, and the HAL's status as errno.
 */
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "bmi270.h"
#include "bmi270_priv.h"
#include "bmi270_regs.h"
#include "imu.h"

enum bus { BUS_I2C, BUS_SPI };

static int allocs, frees;

#include "../bmi270_cases.h"

static I2C_HandleTypeDef hi2c;
static SPI_HandleTypeDef hspi = {
	.Init = {
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_LOW,
		.CLKPhase = SPI_PHASE_1EDGE,
	},
};
static GPIO_TypeDef cs_port;
static bool cs_high = true;
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

/* At least Delay ms: the fake clock moves by exactly that. */
void HAL_Delay(uint32_t Delay)
{
	now_us += (int64_t)Delay * 1000;
}

/* Only the low-to-high edge of the chip select means anything. */
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
	assert(port == &cs_port && pin == GPIO_PIN_4);
	if (state == GPIO_PIN_SET && !cs_high)
		chip_cs_rise();
	cs_high = state == GPIO_PIN_SET;
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

/* The port sends one frame per chip-select window. */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *data,
				   uint16_t size, uint32_t timeout)
{
	(void)timeout;
	assert(h == &hspi && !cs_high);
	return chip_spi_frame(data, NULL, size) ? HAL_OK : fail_status;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,
					  const uint8_t *tx, uint8_t *rx,
					  uint16_t size, uint32_t timeout)
{
	(void)timeout;
	assert(h == &hspi && !cs_high);
	return chip_spi_frame(tx, rx, size) ? HAL_OK : fail_status;
}

static int probe_on(struct imu_dev *imu, enum bus bus,
		    const struct bmi270_config *config)
{
	if (bus == BUS_I2C) {
		struct bmi270_i2c_config c = { .bus = &hi2c };

		return bmi270_probe_i2c(imu, &c, config);
	} else {
		struct bmi270_spi_config c = {
			.bus = &hspi, .cs_port = &cs_port,
			.cs_pin = GPIO_PIN_4,
		};

		return bmi270_probe_spi(imu, &c, config);
	}
}

static void test_bus_configs(void)
{
	struct imu_dev imu = { 0 };
	struct bmi270_i2c_config i2c = { .bus = &hi2c };
	struct bmi270_spi_config spi = {
		.bus = &hspi, .cs_port = &cs_port, .cs_pin = GPIO_PIN_4,
	};

	/* 0x68 by default, 0x69 if asked */
	chip_power_on();
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == 0 && i2c_address == 0x68);
	bmi270_remove(&imu);
	chip_power_on();
	i2c.address = 0x69;
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == 0 && i2c_address == 0x69);
	bmi270_remove(&imu);

	/* SPI mode 3 goes too */
	chip_power_on();
	hspi.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
	assert(bmi270_probe_spi(&imu, &spi, NULL) == 0);
	bmi270_remove(&imu);

	/* Refused, nothing allocated */
	allocs = 0;
	i2c.address = 0x18;
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == -EINVAL);
	i2c = (struct bmi270_i2c_config){ .bus = &hi2c, .timeout_ms = -1 };
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == -EINVAL);
	i2c = (struct bmi270_i2c_config){ .bus = NULL };
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == -EINVAL);
	hspi.Init.CLKPhase = SPI_PHASE_1EDGE;		/* mode 2 */
	assert(bmi270_probe_spi(&imu, &spi, NULL) == -EINVAL);
	hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi.Init.CLKPhase = SPI_PHASE_2EDGE;		/* mode 1 */
	assert(bmi270_probe_spi(&imu, &spi, NULL) == -EINVAL);
	hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi.Init.DataSize = SPI_DATASIZE_16BIT;
	assert(bmi270_probe_spi(&imu, &spi, NULL) == -EINVAL);
	hspi.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.cs_port = NULL;
	assert(bmi270_probe_spi(&imu, &spi, NULL) == -EINVAL);
	spi.cs_port = &cs_port;
	spi.cs_pin = 0;
	assert(bmi270_probe_spi(&imu, &spi, NULL) == -EINVAL);
	spi.bus = NULL;
	assert(bmi270_probe_spi(&imu, &spi, NULL) == -EINVAL);
	assert(allocs == 0);

	/* Nothing at the address: -ENXIO, nothing allocated */
	chip_power_on();
	absent = true;
	i2c = (struct bmi270_i2c_config){ .bus = &hi2c };
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == -ENXIO);
	assert(allocs == 0 && chip.transfers == 0);
	absent = false;

	/* The HAL's status as errno */
	chip_power_on();
	chip.fail_in = 0;
	fail_status = HAL_BUSY;
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == -EAGAIN);
	chip_power_on();
	chip.fail_in = 0;
	fail_status = HAL_ERROR;
	assert(bmi270_probe_i2c(&imu, &i2c, NULL) == -EIO);
	fail_status = HAL_TIMEOUT;
}

int main(void)
{
	run_bmi270_cases();
	test_bus_configs();
	printf("test_bmi270 (stm): all passed\n");
	return 0;
}
