# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-imu`. The existing forge package key, C API and
configuration options retain the `imu` name. The clone command below
also preserves `imu` as the ESP-IDF component directory.

## With forge

stm-forge must be at `ee921c3` or later ("build: libraries may keep sources
in src/ subfolders"); older ones only compile `src/*.c`, so the chip drivers
in `src/<chip>/` are never built and the link fails with undefined
references to the chip probes. esp-forge is unaffected: the component's
CMakeLists lists its sources.

With [esp-forge](https://github.com/JonasAtelier/esp-forge) or
[stm-forge](https://github.com/JonasAtelier/stm-forge). The catalog line in
`profiles/pkgversion` is the same for both:

```make
imu_GIT=https://github.com/JonasAtelier/mcu-imu@stable
```

and a target turns the library and its chips on in `configs/<target>_defconfig`:

```make
PKG_imu             = y
IMU_MPU6050         = y
# IMU_ICM45686 is not set
# IMU_BMI088 is not set
# IMU_WT901 is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_IMU_<CHIP>=y` in
`sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_IMU_<CHIP>=1`.
`IMU_DEBUG = y` turns on `imu_dbg()` lines on the HAL; on ESP-IDF the log
level decides.

## ESP-IDF

Clone into your project's `components/` directory. A local
component is named after its directory, and that name is what you
`REQUIRES`:

```sh
git clone https://github.com/JonasAtelier/mcu-imu components/imu
```

```cmake
idf_component_register(SRCS "main.c" REQUIRES imu driver)
```

Enable the chips under `idf.py menuconfig` → **Component config → IMU
subsystem**, or put `CONFIG_IMU_<CHIP>=y` in your `sdkconfig.defaults` as each
example in `examples/esp/` does. With the subsystem on and no chip picked,
the build warns. ESP-IDF 5.2 or newer is required for the `i2c_master` API.

## STM32 HAL

Copy `include/` and `src/` into an STM32 project, compile every `.c` under
`src/`, chip folders included, add `include/` and `src/` to the include path
(`src/` only needs to reach this library's own sources), and define what the
forge would: `CMSIS_device_header` as your family header (`"stm32f4xx.h"` or
`"stm32f7xx.h"`), `USE_HAL_DRIVER`, and `CONFIG_IMU_<CHIP>=1` for each chip
you want.

**No chip driver is built by default.** Like a kernel subsystem, the core is
always built and each driver is opt-in: a chip whose option is absent or 0
compiles to nothing. With the library on and no chip picked, the ESP-IDF
CMakeLists warns and `src/imu_port_stm.c` emits a `#warning`.

**The HAL modules are yours to enable.** The drivers call the HAL's I2C, SPI
and UART, so your app's `stm32f4xx_hal_conf.h` (or `stm32f7xx_hal_conf.h`) must
define the module each chip uses, and its `.c` must be in the build:

| Option | Needs in hal_conf | HAL source |
|---|---|---|
| `IMU_MPU6050` | `HAL_I2C_MODULE_ENABLED` | `stm32f{4,7}xx_hal_i2c.c` |
| `IMU_ICM45686` | `HAL_SPI_MODULE_ENABLED` | `stm32f{4,7}xx_hal_spi.c` |
| `IMU_BMI088` | both | both |
| `IMU_WT901` | `HAL_UART_MODULE_ENABLED` | `stm32f{4,7}xx_hal_uart.c` |

stm-forge's `mcu/stm32f4.cmake` and `mcu/stm32f7.cmake` already list all
three sources, plus `_hal_i2c_ex.c` (for the `HAL_I2CEx_ConfigAnalogFilter()` that
CubeMX-style init calls) and the F7's `_hal_spi_ex.c`; each compiles to
nothing until hal_conf enables its module. A chip
whose module is off stops the build with an `#error` that says which define
is missing.

**printf() is the log.** Lines go out through `printf()`, so retarget
newlib's `_write()` to a UART. stm-forge links newlib-nano
(`--specs=nano.specs`), whose `printf()` drops `%f` unless the app links with
`-u _printf_float`: add `target_link_options(firmware PRIVATE -u _printf_float)`
to your app's `CMakeLists.txt`, or `imu_log_sample()` prints blanks where the
numbers go.
