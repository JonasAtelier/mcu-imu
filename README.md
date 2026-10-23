<p align="center">
  <img src="assets/icm45686-imu.svg"
       alt="An ICM-45686 reading a board that is being rotated and shaken"
       width="100%">
</p>

# mcu-imu

<p align="center">
  <img alt="platform: ESP-IDF | STM32 HAL" src="https://img.shields.io/badge/platform-ESP--IDF%20%7C%20STM32%20HAL-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

A small C IMU library for **ESP-IDF 5.2+** and **STM32 HAL (F4/F7)**.
Seven chip drivers share one API for timestamped samples of acceleration in
m/s², angular rate in rad/s and die temperature in °C, with board-axis
mapping. This library replaces `esp-imu` and `stm-imu`.

The repository is named `mcu-imu`; the C API, `CONFIG_IMU_*` options, and
forge package key remain `imu`. The clone command below keeps the ESP-IDF
component directory named `imu` as well.

> **Hardware unverified.** Host tests use simulated devices and platform stubs.
> Nothing here has run on a physical chip.

[Supported hardware](#supported-hardware) · [Install](#install) ·
[Quick start](#quick-start) · [Examples](#examples) ·
[Documentation](#documentation) · [Tests](#tests)

## Supported hardware

All seven drivers support both platforms. Enable only the drivers you need.

| Chip | Bus | Notes |
|---|---|---|
| **MPU-6050** | I2C | Address `0x68` or `0x69` by AD0; found on GY-521 boards |
| **ICM-45686** | SPI | SPI only in this driver, mode 3; full-scale codes run backwards |
| **BMI088** | I2C or SPI | Two dies, each with its own address or chip select; on STM32 both HAL modules are needed |
| **WT901C-TTL** | UART | Serial only; the module streams frames, each checked by its SUM |
| **ICM-42688-P** | I2C or SPI | Register banks; anti-alias and UI filters set at probe; data ready on INT1 if asked; on STM32 one HAL module per probe |
| **BMI270** | I2C or SPI | Bosch's 8 KB configuration file uploaded at every probe; data ready on INT1 if asked; on STM32 one HAL module per probe |
| **JY901** | UART or I2C | WitMotion module, its own driver: the WT901's frame stream on the UART, one register burst on I2C at `0x50`; on STM32 one HAL module per probe |

Wiring for every chip is in the [wiring guide](docs/hardware.md); the
WT901's serial protocol is in the [chip reference](docs/chips.md#wt901c-ttl),
the JY901's two buses [there too](docs/chips.md#jy901).

## Install

### ESP-IDF

Clone into your project's components directory:

```sh
git clone https://github.com/JonasAtelier/mcu-imu components/imu
```

Enable your chips in `idf.py menuconfig` → **Component config → IMU
subsystem**, or add this to `sdkconfig.defaults` for the MPU-6050:

```ini
CONFIG_IMU=y
CONFIG_IMU_MPU6050=y
```

The `main` component sees the library automatically. Other components declare
`REQUIRES imu driver` in `idf_component_register()`.

### esp-forge / stm-forge

stm-forge must be at `ee921c3` or later, which builds the chip folders in
`src/`; older ones fail at link with undefined chip probes.

Add to `profiles/pkgversion`:

```make
imu_GIT=https://github.com/JonasAtelier/mcu-imu@stable
```

Enable the package and your chip in `configs/<target>_defconfig`:

```make
PKG_imu            = y
IMU_MPU6050        = y
```

### STM32 HAL without forge

Compile every `.c` under `src/`, chip folders included, add `include/` and
`src/` to the include path, and define
`CMSIS_device_header` as your family header (`"stm32f4xx.h"` or
`"stm32f7xx.h"`), `USE_HAL_DRIVER`, and `CONFIG_IMU_<CHIP>=1` for each
selected chip. Enable and compile its required HAL modules.

**No chip driver is enabled by default.** See the
[installation guide](docs/installation.md) for all chip options, HAL module
requirements and float logging.

## Quick start

Initialise the bus in your application, then probe the sensor, wait for a
fresh sample, and read it. This STM32 helper assumes the passed I2C handle is
already initialised. It polls for up to 200 ms, allowing time for the
MPU-6050's default 100 Hz sample rate:

```c
#include <errno.h>
#include <stdbool.h>

#include "imu.h"
#include "imu_log.h"
#include "mpu6050.h"

int read_once(I2C_HandleTypeDef *bus)
{
    static const struct mpu6050_config cfg = { 0 }; /* 0x68, defaults */
    struct imu_dev imu = { 0 }; /* Must start zeroed. */
    struct imu_sample sample;
    bool ready = false;
    int ret = mpu6050_probe(&imu, bus, &cfg);
    if (ret)
        return ret;

    for (unsigned int attempt = 0; attempt < 20; ++attempt) {
        HAL_Delay(10);
        ret = mpu6050_data_ready(&imu, &ready);
        if (ret || ready)
            break;
    }
    if (!ret)
        ret = ready ? imu_read(&imu, &sample) : -ETIMEDOUT;
    if (!ret)
        imu_log_sample(&imu, &sample);

    mpu6050_remove(&imu);
    return ret;
}
```

Call `read_once(&hi2c1)` from thread context. On ESP-IDF, use an
`i2c_master_bus_handle_t` and a FreeRTOS task delay; the
[MPU-6050 example](examples/esp/mpu6050/main/main.c) shows the bus setup.
For continuous sampling, probe once, poll and read in your loop, and remove
when finished.

- `imu_read()` returns board-frame acceleration in **m/s²**, angular rate in
  **rad/s** and temperature in **°C**, plus `t_us`.
- Integer-returning API calls return **0 or a negative errno**. Use a sample
  only on success; errors leave it unchanged.
- Set `imu.mount` to map chip axes into board axes; all zero means identity.

See [API usage](docs/usage.md) for device lifetime, bus handles, timestamps,
axis mapping, logging, and error codes.

## Examples

| Sensor / task | ESP-IDF | STM32 HAL |
|---|---|---|
| MPU-6050 | [mpu6050](examples/esp/mpu6050/) | [mpu6050](examples/stm/mpu6050/main.c) |
| ICM-45686 | [icm45686](examples/esp/icm45686/) | [icm45686](examples/stm/icm45686/main.c) |
| BMI088 over I2C | [bmi088](examples/esp/bmi088/) | [bmi088](examples/stm/bmi088/main.c) |
| BMI088 over SPI | [bmi088_spi](examples/esp/bmi088_spi/) | [bmi088_spi](examples/stm/bmi088_spi/main.c) |
| WT901C-TTL | — | — |
| ICM-42688-P | [icm42688](examples/esp/icm42688/), on SPI | [icm42688](examples/stm/icm42688/main.c), on I2C with INT1 |
| BMI270 | [bmi270](examples/esp/bmi270/), on I2C | [bmi270](examples/stm/bmi270/main.c), on SPI with INT1 |
| JY901 | [jy901](examples/esp/jy901/), on the UART | [jy901](examples/stm/jy901/main.c), on I2C |

ESP-IDF examples already locate this library through `EXTRA_COMPONENT_DIRS`.
With ESP-IDF activated, build from an example directory:

```sh
cd examples/esp/mpu6050
idf.py set-target esp32 # Replace with your board's target.
idf.py build
```

STM32 examples are snippets
to integrate into your application, with clock setup and logging retargeting
provided by you. Check the example's pin definitions against your board and
read the [wiring guide](docs/hardware.md).

## Scope and limitations

The library provides a shared sample format and synchronous chip drivers.
Your application owns bus setup, scheduling, and locking.

- **Calibration:** bias and scale correction belong above `imu_read()`.
- **Blocking bus calls:** no FIFO, DMA, DMP/APEX or self-test, and no
  driver waits on an interrupt; the ICM-42688-P and the BMI270 can only
  route data ready to their INT1 pin for the app to wait on.
  The register chips poll a status register over the bus, and on the
  MPU-6050 and ICM-45686 the raw register calls reach the rest; the WT901,
  and the JY901 on its UART, wait for their own stream.
- **WT901 and JY901 serial rate:** `imu_read()` blocks up to one output
  period plus two frames, about 125 ms at the defaults; the effective sample rate never
  exceeds the module's output rate, and a faster loop blocks.
- **No unified range or rate API:** the steps do not line up — the BMI088's
  accelerometer runs 3/6/12/24 g, the MPU-6050 and ICM-45686 2 g upwards, and
  the rate
  schemes differ — so a common one would have to round.
- **Not one instant:** the BMI088's accelerometer, gyroscope and temperature
  are three transfers on two dies; the WT901's, and the JY901's on its UART,
  are two consecutive frames.
- **Concurrency:** use one thread per device and bus handle, or supply a mutex.
  The HAL's own lock answers a clash with `HAL_BUSY`, which comes back as
  `-EAGAIN`.
- **No bus setup:** pins, clocks, speed and SPI mode are the app's; the
  drivers only check what they can see in the handle.
- **STM32 timestamps:** the default clock has millisecond resolution and wraps
  after 49.7 days. Override `imu_time_us()` if needed.

## Documentation

| Guide | Contents |
|---|---|
| [Installation](docs/installation.md) | Forge, ESP-IDF, STM32 HAL, build options and logging setup |
| [Hardware](docs/hardware.md) | Wiring and pins for each chip |
| [API usage](docs/usage.md) | Lifecycle, bus handles, timing, mounting and errors |
| [Chip reference](docs/chips.md) | Ranges, rates, protocol details, provenance and datasheet assumptions |
| [Development](docs/development.md) | Platform ports, bus abstraction, adding a driver and test coverage |

Public interfaces live in [include/](include/); chip drivers and platform ports
live in [src/](src/). The [repository layout](docs/development.md#repository-layout)
maps every directory and the file-naming rule for chip folders.

## Tests

```sh
make -C test test
```

Host-only tests use stubbed HAL and ESP-IDF APIs; no embedded toolchain or
physical sensor is needed. See the [test reference](docs/development.md#tests)
for compiler requirements and coverage. Error logs during the suite are
expected: tests deliberately exercise failure paths.

## License

[MIT](LICENSE). The BMI270 driver carries Bosch Sensortec's configuration
file for the chip under the BSD 3-Clause licence; [NOTICE](NOTICE) has its
origin and Bosch's notice, which a firmware built with `IMU_BMI270` must ship
with.
