# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Platform ports

It replaces esp-imu and stm-imu. The chip
drivers in `src/` are the same C on both; only the bus, the clock and the
waits under them differ, in `src/imu_port_esp.c` (ESP-IDF `i2c_master`,
`spi_master`, `esp_timer`, FreeRTOS) and `src/imu_port_stm.c` (HAL). Each
port is empty on the other platform, chosen by `ESP_PLATFORM`, which ESP-IDF
defines for every component. Probes and configs take the platform's own
handles, exactly as the two old libraries did, so apps of either build
unchanged: on ESP-IDF an `i2c_master_bus_handle_t`, an SPI host and CS pins,
and clocks the driver sets; on the HAL an `I2C_HandleTypeDef *`, an
`SPI_HandleTypeDef *` and CS port/pin pairs, with the clock the handle's.

## Bus abstraction

Under the chips sits a regmap-style ops table, `struct
imu_bus_ops` in `src/imu_priv.h`: each chip holds one `struct imu_bus` per
device and talks to it only through `transfer(cmd, tx, rx)`. The kind —
I2C, SPI or UART — is picked at run time by which attach the probe calls.
The BMI088 holds two, one per die: a die is just an I2C address or a chip
select, which is what a bus instance is. Its accelerometer's SPI dummy byte
stays in the chip driver.

`imu_uart_attach()` takes a `uart_port_t` whose driver the app has installed
on ESP-IDF, or a `UART_HandleTypeDef *` it has initialised on the HAL. Each
transfer drops stale input, sends the command byte and its arguments, then
waits for the whole answer; one that stops short is `-ETIMEDOUT`. A part that
streams on its own instead of answering, like the WT901, has one more op,
`receive(rx, n, drop_stale)`: wait for n bytes with nothing sent, dropping
whatever was already waiting first if asked. It is UART-only; I2C and SPI
leave it NULL and `imu_bus_receive()` says `-EOPNOTSUPP`. On the HAL the UART
code sits under `HAL_UART_MODULE_ENABLED`, so an app without the UART module
builds as before.

## Adding a chip

A driver is a probe, a remove and one read function:

1. `include/foo.h`: `int foo_probe(struct imu_dev *imu, <bus>, const struct foo_config *cfg);`
   and `void foo_remove(struct imu_dev *imu);`, with the platform's handle
   types behind `#ifdef ESP_PLATFORM`, as `mpu6050.h` and `icm45686.h` do.
2. `src/foo.c`, flat in `src/` — stm-forge does not glob subdirectories —
   with private headers beside it. Wrap the whole driver in
   `#if CONFIG_IMU_FOO` (never `#ifdef`); the HAL module check goes in
   `src/imu_priv.h`. Keep a `struct imu_bus` per device in your state and
   talk to the chip only through `imu_reg_read()`/`imu_reg_write()` or
   `bus.ops->transfer()`. Give it a
   `static const struct imu_ops foo_ops = { .read = foo_read };`, where
   `foo_read()` fills `accel`, `gyro` and `temp_c` in the chip frame and SI
   units. It must not touch `t_us` or apply the mount; `imu_read()` does both.
3. In probe: return `-EBUSY` if `imu->ops` or `imu->priv` is already set,
   before touching anything. Then set `imu->name` so failures log with it,
   check presence with `imu_i2c_probe()` on I2C before allocating, allocate
   your state, `imu_i2c_attach()` or `imu_spi_attach()` its bus (the only
   lines that unpack the platform's config, behind `#ifdef ESP_PLATFORM`),
   put it into `imu->priv`, set `imu->ops = &foo_ops`, and on failure
   `imu_bus_detach()` and put both back to NULL; remove detaches too. Use
   `imu_err()` for anything the user needs to see.
4. Guard your chip-specific calls with `imu->ops == &foo_ops`.
5. Add `CONFIG_IMU_FOO` to the no-chip checks in `CMakeLists.txt` and
   `src/imu_port_stm.c`, a `src/foo.c` line to `CMakeLists.txt`, a
   `config IMU_FOO` bool to `Kconfig`, `# IMU_FOO is not set` to both forges'
   defconfigs, and `test/esp/test_foo.c` and `test/stm/test_foo.c` targets
   to `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL or ESP-IDF needed. Every test of both old
libraries runs, against its own stubs: `test/esp/` through
`src/imu_port_esp.c` and `test/idf_stubs/`, `test/stm/` through
`src/imu_port_stm.c` and `test/hal_stubs/`. The two maths tests touch no
platform and run once. Nothing is checked against real silicon. The table
describes the `test/stm/` binaries; each `test/esp/` one checks the same
chip through ESP-IDF's calls instead (`i2c_master_*`, `spi_device_*`,
`esp_rom_delay_us()`, `vTaskDelay()`).

| Binary | Checks |
|---|---|
| `test_port_esp`, `test_port_stm` | the UART ops of each port, framing case by case: an ESP-IDF port without an installed driver refused; stale input dropped before the send (on the HAL: overrun cleared, data register read empty); command and arguments in one send; no read for a command without an answer, none after a failed send; a short answer as `-ETIMEDOUT`; the read wait in ticks rounded up; driver and HAL failures mapped to errno; too much to send refused before the line is touched; `receive()` sends nothing, drops stale input only when asked, a short count is `-ETIMEDOUT`, and I2C has none (`-EOPNOTSUPP`) |
| `test_core` | mount matrix is `M·v`, not the transpose; all-zero is identity; errno passes through and the sample is untouched on error; the default clock is `HAL_GetTick()` in µs; log prefix and `imu_log_sample()` format, including 64-bit timestamps without `%lld` |
| `test_mpu6050_math` | scale tables, the 36.53 °C offset, and the rate/filter interaction — the same divider means different rates under different filters, and unreachable rates are refused |
| `test_mpu6050` | a fake MPU-6050 behind a fake HAL: probe, reset-then-wake-onto-PLL, the burst layout with distinct values per axis so a wrong offset cannot pass by luck, the 7-bit address shifted for the HAL, `-ENXIO`/`-EAGAIN`/`-ETIMEDOUT`/`-EIO` mapping, no allocation on a refused probe, teardown leaves it asleep, waits ask for at least what the chip needs, another chip's `imu_dev` is refused |
| `test_icm45686_math` | scale tables run backwards and each step halves; the ODR table |
| `test_icm45686` | a fake ICM-45686 modelling the SPI read bit, the software chip select, soft reset, the indirect register path and the 14-byte burst — accel and gyro must come from **one** transaction; a handle not in mode 3 or not 8-bit is refused |
| `test_bmi088_core` | the chip half against a mock bus: write sequence and delays, scaling, temperature decode and `-ERANGE`, outputs untouched on error, `imu_read()` in SI, an app's `imu_time_us()` replacing the weak default |
| `test_wt901_math` | one accelerometer frame worked by hand, SUM included; any changed byte, or an unknown type even with its SUM made to match, rejected; low-byte-first signed values; the §6.1.2-6.1.4 full scales |
| `test_wt901` | one fake module in `test/wt901_cases.h`, run through both ports by a little glue each: probe writes nothing; silence `-ENXIO`, bad SUMs only `-EBADMSG`, no gyroscope frames `-ENODATA`, a stream that stops `-ETIMEDOUT`, none of them allocating; scales and temperature; stale input dropped; frames in any order with other types between; a read starting mid-frame; noise full of 0x55s; a bad-SUM frame skipped; angles; command bytes; `-EBUSY` and foreign devices refused |
| `test_bmi088` | the HAL transport against fake dies: the accelerometer's SPI dummy byte, its switch to SPI on a chip-select rising edge (at power-on and again after soft reset), errno mapping, a failed probe leaks nothing, no 0 ms delays, bad bus configs refused |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic`, with
`-Wl,--wrap` (GNU ld) to count allocations and capture `printf()`.
