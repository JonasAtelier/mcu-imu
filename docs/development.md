# Architecture, driver development and tests

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

## Repository layout

```
include/                    public API, the only directory apps include
  imu.h                     struct imu_dev, imu_read(), imu_time_us()
  imu_log.h                 imu_err/warn/info/dbg, imu_log_sample()
  mpu6050.h icm45686.h bmi088.h wt901.h bmi270.h icm42688.h jy901.h
  lis2dh12.h                one header per chip: probe, remove, extras
src/                        private to the library
  imu.c                     the core: timestamp, ops.read, mount rotation
  imu_log.c                 imu_log_sample()
  imu_priv.h                bus ops, attach functions, waits; shared by all chips
  imu_port_esp.c            that header on ESP-IDF; empty elsewhere
  imu_port_stm.c            that header on the STM32 HAL; empty under ESP-IDF
  mpu6050/                  mpu6050.c   mpu6050_regs.h   mpu6050_math.h   mpu6050_priv.h
  icm45686/                 icm45686.c  icm45686_regs.h  icm45686_math.h  icm45686_priv.h
  bmi088/                   bmi088.c  bmi088_bus.c  bmi088_regs.h  bmi088_priv.h
  wt901/                    wt901.c     wt901_proto.h
  jy901/                    jy901.c     jy901_regs.h: registers and the WT901-style frames, one address space
  icm42688/                 icm42688.c  icm42688_regs.h  icm42688_math.h  icm42688_priv.h
  bmi270/                   bmi270.c  bmi270_regs.h  bmi270_math.h  bmi270_priv.h
                            bmi270_config_file.c: Bosch's configuration file (BSD-3-Clause, see NOTICE)
  lis2dh12/                 lis2dh12.c  lis2dh12_regs.h  lis2dh12_math.h  lis2dh12_priv.h
test/                       host tests, `make -C test test`
  test_<chip>_math.c        maths tests: no bus and no platform API
  wt901_cases.h             WT901 cases, shared by esp/ and stm/
  jy901_cases.h             a fake JY901, its UART line and its I2C registers, shared by esp/ and stm/
  bmi270_cases.h            a fake BMI270 and its cases, shared by esp/ and stm/, each run on I2C and SPI
  icm42688_cases.h          a fake ICM-42688-P with its register banks, the same way
  lis2dh12_cases.h          a fake LIS2DH12 and its cases, shared by esp/ and stm/, I2C only
  esp/                      suites on idf_stubs/, through imu_port_esp.c
  stm/                      suites on hal_stubs/, through imu_port_stm.c
  idf_stubs/  hal_stubs/    the slice of ESP-IDF and of the HAL the code calls
examples/
  esp/<chip-or-board>/      ESP-IDF projects: bmi088, bmi088_spi, bmi270, icm42688, icm45686, jy901, lis2dh12, mpu6050, wt901
  stm/<chip-or-board>/      STM32 snippets: bmi088, bmi088_spi, bmi270, icm42688, icm45686, jy901, lis2dh12, mpu6050, wt901
docs/                       the five guides; datasheet PDFs here are gitignored
assets/                     README and chip-reference figures
CMakeLists.txt              ESP-IDF component; elsewhere a static library for add_subdirectory(); stm-forge globs src/**/*.c instead
                            the static library's options are Kconfig's bool entries, found by the type line directly under each `config`
Kconfig                     IMU and IMU_<CHIP>, for ESP-IDF and both forges' menus
idf_component.yml           ESP-IDF component manifest
NOTICE                      third-party notices: Bosch's BMI270 configuration file
```

Naming inside `src/<chip>/`, the same in every category library:

| File | Holds |
|---|---|
| `<chip>.c` | the driver: probe, remove, the `ops.read`, the chip's extras; may split into `<chip>_<part>.c` (e.g. `bmi088_bus.c`) |
| `<chip>_regs.h` | register map: addresses, bit fields, values, each with its datasheet citation |
| `<chip>_proto.h` | serial protocol: command bytes, frames, checksums, for parts without registers |
| `<chip>_math.h` | pure conversions, compiled and asserted on the host by `test/<chip>_math` tests |
| `<chip>_priv.h` | driver state, only if needed |

A chip has `_regs.h` or `_proto.h`, never both; `_math.h` and `_priv.h` only
when it needs them. Everything in `src/` is private: apps see `include/` only.
`bmi270_config_file.c` is the one file not written here: Bosch's array,
copied byte for byte, renamed `bmi270_config_data` so it does not clash with
Bosch's own API in the same app, its origin, SHA-256 and licence in its
header.

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
2. `src/foo/foo.c`, in a folder of its own with its private headers
   (`foo_regs.h`, `foo_priv.h`, ...); common files and the platform ports
   stay at the top of `src/`. Include siblings and `imu_priv.h` by plain
   name, never with `../`: siblings resolve from the chip's own folder, and
   `imu_priv.h` through `src/`, which the ESP-IDF CMakeLists has as
   `PRIV_INCLUDE_DIRS` and stm-forge's `libs.cmake` puts on this library's
   include path while it compiles every `.c` under `src/`. Wrap the whole
   driver in
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
5. Add `CONFIG_IMU_FOO` to the no-chip checks in `CMakeLists.txt` (its
   ESP-IDF part; the `add_subdirectory()` part reads `Kconfig`) and
   `src/imu_port_stm.c`, a `src/foo/foo.c` line to `CMakeLists.txt`, a
   `config IMU_FOO` bool to `Kconfig`, `# IMU_FOO is not set` to both forges'
   defconfigs, and `test/esp/test_foo.c` and `test/stm/test_foo.c` (and
   `test/test_foo_math.c` for its maths header) targets to `test/Makefile`.

## Tests

```sh
make -C test test
```

Host-only; no toolchain, HAL or ESP-IDF needed. Every test of both old
libraries runs, against its own stubs, in the layout of
[Repository layout](#repository-layout): `test/esp/` through
`src/imu_port_esp.c` and `test/idf_stubs/`, `test/stm/` through
`src/imu_port_stm.c` and `test/hal_stubs/`. The maths tests in `test/` touch
no bus or platform API and run once. Nothing is checked against real silicon. The table
describes the `test/stm/` binaries; each `test/esp/` one checks the same
chip through ESP-IDF's calls instead (`i2c_master_*`, `spi_device_*`,
`esp_rom_delay_us()`, `vTaskDelay()`).

| Binary | Checks |
|---|---|
| `esp/test_port_esp`, `stm/test_port_stm` | the UART ops of each port, framing case by case: an ESP-IDF port without an installed driver refused; stale input dropped before the send (on the HAL: overrun cleared, data register read empty); command and arguments in one send; no read for a command without an answer, none after a failed send; a short answer as `-ETIMEDOUT`; the read wait in ticks rounded up; driver and HAL failures mapped to errno; too much to send refused before the line is touched; `receive()` sends nothing, drops stale input only when asked, a short count is `-ETIMEDOUT`, and I2C has none (`-EOPNOTSUPP`) |
| `{esp,stm}/test_core` | mount matrix is `M·v`, not the transpose; all-zero is identity; errno passes through and the sample is untouched on error; the default clock is `HAL_GetTick()` in µs; log prefix and `imu_log_sample()` format, including 64-bit timestamps without `%lld` |
| `test_mpu6050_math` | scale tables, the 36.53 °C offset, and the rate/filter interaction — the same divider means different rates under different filters, and unreachable rates are refused |
| `{esp,stm}/test_mpu6050` | a fake MPU-6050 behind a fake HAL: probe, reset-then-wake-onto-PLL, the burst layout with distinct values per axis so a wrong offset cannot pass by luck, the 7-bit address shifted for the HAL, `-ENXIO`/`-EAGAIN`/`-ETIMEDOUT`/`-EIO` mapping, no allocation on a refused probe, teardown leaves it asleep, waits ask for at least what the chip needs, another chip's `imu_dev` is refused |
| `test_icm45686_math` | scale tables run backwards and each step halves; the ODR table |
| `{esp,stm}/test_icm45686` | a fake ICM-45686 modelling the SPI read bit, the software chip select, soft reset, the indirect register path and the 14-byte burst — accel and gyro must come from **one** transaction; a handle not in mode 3 or not 8-bit is refused |
| `{esp,stm}/test_bmi088_core` | the chip half against a mock bus: write sequence and delays, scaling, temperature decode, NaN for the invalid code, outputs untouched on error, `imu_read()` in SI, an app's `imu_time_us()` replacing the weak default |
| `test_wt901_math` | one accelerometer frame worked by hand, SUM included; any changed byte, or an unknown type even with its SUM made to match, rejected; low-byte-first signed values; the §6.1.2-6.1.4 full scales |
| `{esp,stm}/test_wt901` | one fake module in `test/wt901_cases.h`, run through both ports by a little glue each: probe writes nothing; silence `-ENXIO`, bad SUMs only `-EBADMSG`, no gyroscope frames `-ENODATA`, a stream that stops `-ETIMEDOUT`, none of them allocating; scales and temperature; stale input dropped; frames in any order with other types between; a read starting mid-frame; noise full of 0x55s; a bad-SUM frame skipped; angles; command bytes; `-EBUSY` and foreign devices refused |
| `test_jy901_math` | the WT901 frame checks again, on the JY901's own header; §7.2's worked read from 0x3d, -176.33°, -1.19°, 51.89°, as printed; the AX-to-TEMP burst layout and register addresses of chapter 7 |
| `{esp,stm}/test_jy901` | one fake module in `test/jy901_cases.h` with a UART line and an I2C register file. UART: the WT901 cases (silence, bad SUMs, missing types, a stopped stream, stale input, any order, mid-frame start, noise, angles, command bytes). I2C: 0x50 by default, **`IICADDR` checked, a wrong value or high byte `-ENODEV`**, another address only when `IICADDR` holds it; nothing written at probe; a bus error at either probe transfer ends it with nothing allocated or added; **`imu_read()` one 26-byte read from 0x34**, with distinct values in the registers between so a wrong offset fails; a failed read leaves the sample alone; angles by §7.2's example; `jy901_command()` a two-byte register write; foreign devices refused. Per platform: the ESP-IDF clock, 400 kHz by default and no more; the HAL's `HAL_ERROR` as `-EIO` and `HAL_BUSY` as `-EAGAIN` |
| `{esp,stm}/test_bmi088` | the HAL transport against fake dies: the accelerometer's SPI dummy byte, its switch to SPI on a chip-select rising edge (at power-on and again after soft reset), errno mapping, a failed probe leaks nothing, no 0 ms delays, bad bus configs refused |
| `test_icm42688_math` | Tables 1 and 2 and 14.37-14.38: every full scale and sensitivity as printed, the rate codes with the low-power-only ones refused; 14.6's temperature; the UI filter codes; anti-alias rows typed in by hand from 5.3, both ends and the rows where DELTSQR is not DELT squared, every row rising and fitting its register |
| `{esp,stm}/test_icm42688` | one fake ICM-42688-P in `test/icm42688_cases.h`, each case on I2C and on SPI: **banks 0, 1, 2 and 4 kept apart, so a register written in the wrong bank is caught**, bank 0 again after every banked access; nothing heard for 1 ms after the soft reset; **a write other than rate, scale or power mode while a sensor runs (12.9), or any write within 200 us of a sensor coming on, counted and failed**; the 12.3 I2C settings on I2C only; the anti-alias rows 1, 13, 20, 24, 40 and 63 in the right registers with reserved bits kept, bypass alone; UI filter order and bandwidth; the interrupt both ways with `INT_ASYNC_RESET` cleared, the latched pin released by a sample read; the big-endian burst and temperature; rate, scale and power changes after probe; the bank escape hatch; refused codes before any allocation or traffic; a bus error at every transfer of a full probe ends it with nothing bound or leaked |
| `test_bmi270_math` | Tables 2 to 4: every range's sensitivity, the rate codes and which are valid, temperature at 23 °C, both ends and the invalid code; `GYR_CAS` sign extension, the cross-axis correction with its integer division and 16-bit limit |
| `{esp,stm}/test_bmi270` | one fake BMI270 in `test/bmi270_cases.h`, each case on I2C and on SPI: the interface I2C after power-up and every soft reset until a CSB edge and 200 us; the SPI dummy byte; a write lost if it comes before the idle time of the power mode it follows (450 us, 2 us); the upload only to a prepared chip, at its `INIT_ADDR`, **byte for byte equal to Bosch's file** (274 pieces, none over 30 bytes), `INIT_CTRL` = 1 once per reset; `-EPROTO` for a reported load error or one corrupted byte, `-ETIMEDOUT` no earlier than 20 ms, `-ENODEV` with nothing written; a bus error at every 37th transfer and the last one ends the probe with nothing bound or leaked; every register of a full config and of the interrupt both ways; the burst, cross-axis correction, NaN temperature, `INT_STATUS_1` cleared; rate and range changes keeping the filter bits; refused configs before any allocation or bus traffic. Per platform: the address, clock and SPI mode passed or refused, `-ENXIO` before allocation, the HAL's status as errno |
| `test_lis2dh12_math` | the mg/count table per mode and full scale, 12/48/192 at 16 g; left-justified words cut to 8, 10 and 12 bits, worked by hand, signs and both ends; which rate codes each mode runs; the motion threshold's step per full scale, rounding and its 1..127 limits; `INT1_SRC` and `INT1_CFG` bit by bit. Checked against the remembered register map, which is itself UNVERIFIED (`docs/chips.md`) |
| `{esp,stm}/test_lis2dh12` | one fake LIS2DH12 in `test/lis2dh12_cases.h`, on I2C: **a multi-byte read without the auto-increment bit repeats the first register**, so a burst without it fails; **`LPen` and `HR` set together counted and failed**, from every starting mode; `CTRL_REG0`'s fixed bits and the 7-bit registers' bit 7 kept; no write to a read-only address; a probe over a chip left dirty by an earlier run writes every control register and reads the old latched event away; every mode and full scale scaled, gyroscope and temperature NaN; the motion detector's registers, the filter reset by a `REFERENCE` read after the filter is on, the event decoded and released, the threshold rewritten on a full-scale change and a change it would not fit refused; refused configs before any allocation or traffic; a bus error at every transfer of a full probe and of the motion setup, nothing bound or leaked and the retry clean. Per platform: the address and clock passed or refused, `-ENXIO` before allocation, the HAL's status as errno; on ESP-IDF the driver never waits |

All build under `-std=gnu11 -Wall -Wextra -Werror -pedantic`, with
`-Wl,--wrap` (GNU ld) to count allocations and capture `printf()`.
