# mcu-imu

Read acceleration, angular velocity and temperature from a motion sensor, with timestamps and optional mapping into your board’s axes. Accelerometer-only devices report unavailable quantities as NaN.

**Platforms** · ESP-IDF 5.2+ and STM32 HAL (F4/F7). Hardware testing is pending. The host tests use simulated devices.

## Supported hardware

| Chip | Bus | Notes |
|---|---|---|
| **MPU-6050** | I2C | InvenSense 6-axis IMU |
| **ICM-45686** | SPI | TDK InvenSense 6-axis IMU |
| **BMI088** | I2C or SPI | Bosch 6-axis IMU on two dies |
| **WT901C-TTL** | UART | WitMotion attitude module |
| **ICM-42688-P** | I2C or SPI | TDK InvenSense 6-axis IMU |
| **BMI270** | I2C or SPI | Bosch 6-axis IMU |
| **JY901** | UART or I2C | WitMotion attitude module |
| **LIS2DH12** | I2C | ST 3-axis accelerometer, motion interrupt |

## Quick start

Connect an MPU-6050 over I2C and match `PIN_SDA` and `PIN_SCL` to the wiring. The example uses address 0x68.

In a terminal with ESP-IDF activated, get the example and select the target for an ESP32. Change the target if your board uses another ESP32 chip.

```sh
git clone https://github.com/JonasAtelier/mcu-imu.git imu
cd imu/examples/esp/mpu6050
idf.py set-target esp32
```

Check the [wiring guide](docs/hardware.md) and edit the settings in [main.c](examples/esp/mpu6050/main/main.c) to match your board. Connect the board by USB, then build, upload and open the serial monitor.

```sh
idf.py build flash monitor
```

Rotate the board to see acceleration in m/s² and angular velocity in rad/s change. Apply sensor bias and scale calibration in your application.

<details>
<summary>STM32 HAL example for an initialised bus</summary>

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

</details>

## Using the library in your project

- Add the component and enable the driver for your hardware. [Installation and build options](docs/installation.md)
- Start from an example for your chip. The STM32 examples fit into a project with board setup already in place. [ESP-IDF examples](examples/esp/) · [STM32 examples](examples/stm/)
- Look up calls and chip-specific settings. [API usage](docs/usage.md) · [Hardware reference](docs/chips.md)

To run the simulated tests on your computer, run `make -C test test` from the library root. [Test requirements and coverage](docs/development.md)

## License

[MIT](LICENSE). The BMI270 driver carries Bosch Sensortec's configuration
file for the chip under the BSD 3-Clause licence; [NOTICE](NOTICE) has its
origin and Bosch's notice, which a firmware built with `IMU_BMI270` must ship
with.

<p align="center">
  <img src="assets/icm45686-imu.svg"
       alt="An ICM-45686 reading a board that is being rotated and shaken"
       width="100%">
</p>
