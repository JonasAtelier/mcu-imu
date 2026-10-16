# API usage

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Device lifecycle

Initialise the bus, probe once, then poll for data ready before reading.
The [README quick start](../README.md#quick-start) shows a complete STM32
helper with a bounded wait and cleanup. The [ESP-IDF example](../examples/esp/mpu6050/main/main.c)
shows the corresponding bus setup.

One `imu_log_sample()` line looks like this (values illustrative):

```
imu: mpu6050: t 1234000 us  a   0.012  -0.034   9.801 m/s^2  g   0.001  -0.002   0.000 rad/s   27.4 C
```

The other chips are the same after the probe:
`icm45686_probe(&imu, &cfg)`, `bmi088_probe_i2c(&imu, &bus_cfg, NULL)`,
`bmi088_probe_spi(...)` or `wt901_probe(&imu, uart, NULL)`. Tear down with
the matching `*_remove(&imu)`; the bus is left alone.

**Bus handles on ESP-IDF.** I2C chips take an `i2c_master_bus_handle_t` the
app has created, and the driver adds its own device at the config's clock
(`scl_hz` or `clock_hz`, 0 for 400 kHz); `*_remove()` takes it off again. SPI
chips take a host the app has `spi_bus_initialize()`d and a CS GPIO, and the
driver adds the device in the mode the chip needs.

**Bus handles on the HAL.** I2C chips take an `I2C_HandleTypeDef *` the app has
`HAL_I2C_Init()`ed, plus a **7-bit** address (`0x68`, not the HAL's shifted
`0xD0`; the driver shifts it). SPI chips take an `SPI_HandleTypeDef *` the
app has `HAL_SPI_Init()`ed and a chip select as a GPIO port and pin
(`GPIOD`, `GPIO_PIN_14`): chip select is software, driven by the driver around
each transfer, so the pin is an ordinary output, idle high. The probe checks
the handle's SPI mode and frame size and refuses a wrong one with `-EINVAL`.
Bus speed is the handle's business.

**A `struct imu_dev` must start zeroed** — static storage or `= { 0 }`, as
above. Probe on one that is already bound (non-NULL `ops` or `priv`) returns
`-EBUSY` and touches nothing, rather than leak the first binding. `*_remove()`
and a failed probe zero it again, so it can be probed afresh.

**Timestamps.** `s.t_us` comes from `imu_time_us()`, taken just before the
bus transfer. On ESP-IDF that is `esp_timer_get_time()`. On the HAL the
library's default is weak and only `HAL_GetTick() * 1000`:
whole milliseconds, wrapping after 49.7 days. **Define your own
`int64_t imu_time_us(void)` for real microseconds** and it replaces the
default at link time — the `DWT->CYCCNT` cycle counter (enable
`CoreDebug->DEMCR` TRCENA and `DWT->CTRL` CYCCNTENA, then divide by the core
clock in MHz and extend the 32-bit count to 64; an F7 may also need
`DWT->LAR` unlocked first), or a free-running timer at 1 MHz.

**Delays.** On ESP-IDF, millisecond waits use `vTaskDelay()`, rounded up
and plus one tick, so never shorter than asked; at the default 100 Hz tick
even a 2 ms wait takes 10–20 ms. The BMI088's sub-millisecond settles use
`esp_rom_delay_us()`. On the HAL they wait with
`HAL_Delay()`, which already
adds one tick to what it is asked, so no wait is ever shorter than the chip
needs; the BMI088's microsecond settles round up to 1 ms. An MPU-6050 probe
takes up to about 120 ms (its reset), busy-waiting on SysTick: probe from
thread context, never from an interrupt at or above SysTick's priority. A
WT901 probe waits on the stream instead: silence gives `-ENXIO` after
200 ms (the default `timeout_ms`), and `-ENODATA` can take about 1 s with
the default frame content while it looks through its 256-byte budget
(0.3–2.3 s depending on which frames are on). If your app
replaces the weak `HAL_Delay()` (an RTOS often does), the "never short"
guarantee is that replacement's.

**Logging.** On ESP-IDF everything goes under the log tag `imu`, prefixed
with the chip name, and `imu_dbg()` is `ESP_LOGD`. On the HAL every line goes
through `printf()` as `imu: <chip>: ...`. Your
own code can use the same `imu_err()`, `imu_warn()`, `imu_info()` and
`imu_dbg()` from `imu_log.h`. `imu_dbg()` is compiled out unless
`CONFIG_IMU_DEBUG` is 1 (stm-forge: `IMU_DEBUG = y`). There are no levels
beyond that: printf() is the one sink.

**Chip extras** all take the same `struct imu_dev *` and return `-EINVAL` if
another driver probed it. Each chip's header lists them: `mpu6050.h` (ranges,
rate/DLPF, sleep, I2C bypass, raw registers), `icm45686.h` (ranges, ODR,
power, raw registers), `bmi088.h` (single-die and raw reads, data ready),
`wt901.h` (the module's own angles, raw commands).

### Mount: get the axes into your board's frame

`imu.mount` is a 3×3 matrix from chip axes to board axes:
`board[i] = Σ mount[i][j] · chip[j]`, applied to accel and gyro. All zero,
the default, means identity. Set it before or after probe; probe leaves it
alone. A chip rotated 90° anticlockwise about Z, so chip X points along
board Y:

```c
static const int8_t rot_z90[3][3] = {
	{ 0, -1, 0 },
	{ 1,  0, 0 },
	{ 0,  0, 1 },
};
memcpy(imu.mount, rot_z90, sizeof(rot_z90));
```

## Errors

`-ENODEV` means something answered but it is not the chip you
asked for (WHO_AM_I or CHIP_ID mismatch; the log shows what was read against
what was expected). `-ENXIO` means nothing ACKed on I2C, checked with
`i2c_master_probe()` or `HAL_I2C_IsDeviceReady()` before anything is allocated
or written. On the HAL, a chip that
NACKs its address is `HAL_ERROR` there, `-ENXIO`, on the F4 and the F7. On the
F4, a bus with no pull-ups or SDA stuck low sets the peripheral's BUSY flag,
so the probe gives up after 25 ms with `-EAGAIN`; `-ETIMEDOUT` means a START
never completed. Check the wiring for either. SPI has no
ACK, so an absent SPI chip (ICM-45686, ICM-42688-P, BMI088 or BMI270 over
SPI) reads back a wrong ID
and is `-ENODEV`. `-EBUSY` from a probe means that `imu_dev` is already bound.
On ESP-IDF bus failures map from `esp_err_t`: `ESP_ERR_TIMEOUT` is
`-ETIMEDOUT`, `ESP_ERR_INVALID_ARG` `-EINVAL`, `ESP_ERR_INVALID_SIZE`
`-EMSGSIZE`, `ESP_ERR_NO_MEM` `-ENOMEM` and the rest, a mid-transfer NACK
included, `-EIO`. On the HAL bus failures map from its four answers: `HAL_TIMEOUT` is `-ETIMEDOUT`,
`HAL_BUSY` is `-EAGAIN` (the bus or the handle is in use, or the handle was
never `HAL_*_Init()`ed) — never `-EBUSY`, which a probe keeps for "already
bound" — and `HAL_ERROR`
is `-EIO` — including a NACK in the middle of a transfer. `-EINVAL` and
`-ENOMEM` mean what they say. A failed range or rate change leaves the
driver's idea of the scale matching whatever the chip actually took, so
`imu_read()` stays correct even if only half of it landed. The BMI088 and
the BMI270 read an invalid temperature code as a NaN temperature and keep
the rest of the sample. A BMI270 probe adds `-ETIMEDOUT` (the
configuration file never reports loaded within the datasheet's 20 ms) and
`-EPROTO` (the chip reports the load failed); see the
[chip reference](chips.md#bmi270).
The WT901 adds `-EBADMSG` (bytes arrive, but no frame's SUM ever matches)
and `-ENODATA` (valid frames, but not the ones asked for); see the
[chip reference](chips.md#wt901c-ttl).
`strerror(-ret)` names any of them.
