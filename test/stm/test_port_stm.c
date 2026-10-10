/*
 * Host-side check of the UART ops in src/imu_port_stm.c against
 * test/hal_stubs; I2C and SPI are covered by the chip tests beside this one.
 * Also that receive() is UART-only.
 */
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "stm32_hal_stub.h"

#include "imu_priv.h"

static UART_HandleTypeDef uart_handle;
static char calls[16];		/* O overrun clear, F flush, W send, R receive */
static HAL_StatusTypeDef next_status;
static int timeout_after_send;	/* the receive after a send times out */
static int stale_bytes;		/* bytes waiting in the data register */
static int ore_cleared;
static uint8_t sent[40];
static size_t sent_len;
static uint8_t reply[32];
static uint16_t last_size;

static void reset(void)
{
	calls[0] = '\0';
	next_status = HAL_OK;
	timeout_after_send = stale_bytes = ore_cleared = 0;
	sent_len = 0;
}

static void called(char c)
{
	size_t n = strlen(calls);

	assert(n + 1 < sizeof(calls));
	calls[n] = c;
	calls[n + 1] = '\0';
}

int stub_uart_get_flag(UART_HandleTypeDef *h, uint32_t flag)
{
	assert(h == &uart_handle && flag == UART_FLAG_RXNE);
	return stale_bytes > 0;
}

void stub_uart_clear_ore(UART_HandleTypeDef *h)
{
	assert(h == &uart_handle);
	called('O');
	ore_cleared++;
}

void stub_uart_flush(UART_HandleTypeDef *h)
{
	assert(h == &uart_handle);
	called('F');
	stale_bytes--;
}

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, const uint8_t *data,
				    uint16_t size, uint32_t t)
{
	(void)t;
	assert(h == &uart_handle && sent_len + size <= sizeof(sent));
	called('W');
	memcpy(sent + sent_len, data, size);
	sent_len += size;
	if (!timeout_after_send)
		return next_status;
	next_status = HAL_TIMEOUT;	/* for the receive that follows */
	return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *h, uint8_t *data,
				   uint16_t size, uint32_t t)
{
	(void)t;
	assert(h == &uart_handle);
	called('R');
	last_size = size;
	if (next_status == HAL_OK)
		memcpy(data, reply, size);
	return next_status;
}

static void test_uart(void)
{
	static const uint8_t args[2] = { 0x11, 0x22 };
	static const struct {
		HAL_StatusTypeDef status;
		int expected;
	} map[] = {
		{ HAL_OK,	0 },
		{ HAL_ERROR,	-EIO },
		{ HAL_BUSY,	-EAGAIN },
		{ HAL_TIMEOUT,	-ETIMEDOUT },
	};
	uint8_t rx[2] = { 0 }, many[IMU_BUS_MAX_LEN + 1] = { 0 };
	struct imu_bus bus;

	assert(imu_uart_attach(&bus, &uart_handle, 100) == 0);
	assert(bus.ops->kind == IMU_BUS_UART);

	/* Stale input dropped first: overrun cleared, two bytes read out. */
	reset();
	stale_bytes = 2;
	memcpy(reply, "\x12\x34", 2);
	assert(bus.ops->transfer(&bus, 0x13, args, 2, rx, 2) == 0);
	assert(strcmp(calls, "OFFWR") == 0);
	assert(stale_bytes == 0 && ore_cleared == 1);
	assert(sent_len == 3 && sent[0] == 0x13 && sent[1] == 0x11 &&
	       sent[2] == 0x22);
	assert(last_size == 2 && rx[0] == 0x12 && rx[1] == 0x34);

	/* No reply wanted: no receive. A failed send: no receive either. */
	reset();
	assert(bus.ops->transfer(&bus, 0x80, NULL, 0, NULL, 0) == 0);
	assert(strcmp(calls, "OW") == 0);
	reset();
	next_status = HAL_BUSY;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -EAGAIN);
	assert(strcmp(calls, "OW") == 0);

	/* A reply that stops short: the send passes, the receive times out. */
	reset();
	timeout_after_send = 1;
	assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 2) == -ETIMEDOUT);
	assert(strcmp(calls, "OWR") == 0);

	/* HAL status to errno, through a send and a receive. */
	for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
		reset();
		next_status = map[i].status;
		assert(bus.ops->transfer(&bus, 0x13, NULL, 0, rx, 1) ==
		       map[i].expected);
	}

	/* Too much to send is refused before the line is touched. */
	reset();
	assert(bus.ops->transfer(&bus, 0x13, many, sizeof(many), NULL, 0) ==
	       -EINVAL);
	assert(calls[0] == '\0');

	imu_bus_detach(&bus);
}

/* receive(): nothing sent, stale input dropped only when asked. */
static void test_receive(void)
{
	static I2C_HandleTypeDef i2c_handle;
	struct imu_bus bus, i2c;
	uint8_t rx[2] = { 0 };

	assert(imu_uart_attach(&bus, &uart_handle, 100) == 0);

	reset();
	stale_bytes = 1;
	memcpy(reply, "\x55\x51", 2);
	assert(imu_bus_receive(&bus, rx, 2, false) == 0);
	assert(strcmp(calls, "R") == 0 && sent_len == 0);
	assert(stale_bytes == 1 && last_size == 2);
	assert(rx[0] == 0x55 && rx[1] == 0x51);

	reset();
	stale_bytes = 2;
	assert(imu_bus_receive(&bus, rx, 2, true) == 0);
	assert(strcmp(calls, "OFFR") == 0 && stale_bytes == 0);

	reset();
	next_status = HAL_TIMEOUT;
	assert(imu_bus_receive(&bus, rx, 2, false) == -ETIMEDOUT);
	reset();
	assert(imu_bus_receive(&bus, rx, 0, false) == -EINVAL);
	assert(calls[0] == '\0');
	imu_bus_detach(&bus);

	/* I2C has no receive(). */
	assert(imu_i2c_attach(&i2c, &i2c_handle, 0x50, 0, 10) == 0);
	assert(imu_bus_receive(&i2c, rx, 1, false) == -EOPNOTSUPP);
	imu_bus_detach(&i2c);
}

int main(void)
{
	test_uart();
	test_receive();
	printf("test_port_stm: all passed\n");
	return 0;
}
