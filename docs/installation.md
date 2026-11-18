# Installation

[Back to README](../README.md)

Paths mentioned in this guide are relative to the repository root.

The repository is `mcu-imu`. The existing forge package key, C API and
configuration options retain the `imu` name. The clone command below
also preserves `imu` as the ESP-IDF component directory.

Three step-by-step guides come first: [ESP-IDF](#esp-idf-step-by-step),
[STM32 with CMake](#with-cmake-a-cubemx-cmake-project) and
[STM32 without CMake](#without-cmake-stm32cubeide-a-makefile). None
needs a forge; if you build with esp-forge or stm-forge, see
[With forge](#with-forge). The steps use the MPU-6050; for another chip,
change the chip name. The reference follows the guides.

## ESP-IDF, step by step

**You need** ESP-IDF 5.2 or newer, with `idf.py` working in your terminal, and
an ESP-IDF project: a folder with a `CMakeLists.txt` and a `main/` folder.
`idf.py create-project my-project` makes one.

ESP-IDF calls a library a **component**: a folder with its own
`CMakeLists.txt` that the build finds and compiles. This library is the
component `imu`.

1. **Get the library.** Pick one way.

   - **Component manager**, the simplest: the first build downloads the
     library into `managed_components/imu`. Create `main/idf_component.yml`:

     ```yaml
     dependencies:
       imu:
         git: https://github.com/JonasAtelier/mcu-imu.git
         version: stable
     ```

   - **Clone**, if you want to edit the library or build offline. In the
     project folder:

     ```sh
     git clone https://github.com/JonasAtelier/mcu-imu components/imu
     ```

     Keep the folder name `imu`: ESP-IDF names a component after its folder.

   ```
   my-project/
   ├── CMakeLists.txt
   ├── sdkconfig.defaults       step 2
   ├── main/
   │   ├── CMakeLists.txt       step 3
   │   ├── idf_component.yml    step 1, component manager
   │   └── my-project.c         step 4
   └── components/
       └── imu/                 step 1, clone
   ```

2. **Turn your chip on.** No chip driver is built until you do. Create
   `sdkconfig.defaults` in the project folder, a text file of settings you
   choose, with one line per chip:

   ```ini
   CONFIG_IMU_MPU6050=y
   ```

   The other chips are `CONFIG_IMU_ICM45686`, `_BMI088`, `_WT901`,
   `_ICM42688`, `_BMI270`, `_JY901` and `_LIS2DH12`. ESP-IDF copies these
   lines into `sdkconfig`, the project's full settings file, only when it
   creates that file: **if `sdkconfig` already exists, delete it** so your
   line is read. `idf.py menuconfig` → **Component config → IMU subsystem**
   does the same from a menu.

3. **Let your code see the library.** A new project's `main/CMakeLists.txt`
   has no `REQUIRES` and already sees every component. If yours has a
   `REQUIRES` list, add `imu` and `driver` to it:

   ```cmake
   idf_component_register(SRCS "main.c" REQUIRES imu driver)
   ```

4. **Write the smallest test.** Copy
   [examples/esp/mpu6050/main/main.c](../examples/esp/mpu6050/main/main.c)
   over your main source file and set `PIN_SDA` and `PIN_SCL` to your
   wiring. [API usage](usage.md) explains the calls.

5. **Build.**

   ```sh
   idf.py set-target esp32    # your chip: esp32s3, esp32c3, ...
   idf.py build
   ```

   A good build ends with `Project build complete.` Flash it as any ESP-IDF
   app and open the serial monitor: the driver's first line ends in
   `imu: mpu6050: at 0x68: 4 g, 500 dps, 100.00 Hz`.

**If it fails**

| The build prints | Why | Fix |
|---|---|---|
| `Failed to resolve component 'imu' required by component 'main': unknown name.` | The clone is not in a folder named `imu` | Rename it to `components/imu` |
| `fatal error: imu.h: No such file or directory` | `main/CMakeLists.txt` has a `REQUIRES` list without `imu` | Step 3 |
| `imu: CONFIG_IMU is on but no chip driver is selected` (a CMake warning) | No chip turned on | Step 2 |
| ``undefined reference to `mpu6050_probe'`` | That chip is not turned on, often because `sdkconfig` existed before you edited `sdkconfig.defaults` | Delete `sdkconfig`, build again |

## STM32, step by step

**You need** a project for an STM32F4 or F7 that uses ST's **HAL**, the
vendor library behind `HAL_Init()`, `HAL_I2C_Init()` and friends, with the
I2C, SPI or UART your chip is wired to already set up. STM32CubeMX makes such
a project; with **Toolchain / IDE: CMake** it makes a CMake one, the main
case below.

Two terms. A **HAL module** is the part of the HAL for one peripheral: I2C,
SPI, UART. It is compiled only when it is switched on in **hal_conf**, the
file `Core/Inc/stm32f4xx_hal_conf.h` (`stm32f7xx_hal_conf.h` on an F7).
CubeMX switches a module on when you enable that peripheral.

### With CMake (a CubeMX CMake project)

1. **Get the library.** In the project folder, next to the top-level
   `CMakeLists.txt`:

   ```sh
   git clone https://github.com/JonasAtelier/mcu-imu
   ```

   ```
   my-board/
   ├── CMakeLists.txt                  step 2
   ├── cmake/stm32cubemx/              made by CubeMX
   ├── Core/
   │   ├── Inc/stm32f4xx_hal_conf.h    hal_conf, step 3
   │   └── Src/main.c                  step 4
   ├── Drivers/                        the HAL, copied in by CubeMX
   └── mcu-imu/                        step 1
   ```

2. **Add it to the build.** In the top `CMakeLists.txt`, after the line
   `add_subdirectory(cmake/stm32cubemx)`, add:

   ```cmake
   set(CONFIG_IMU_MPU6050 ON)          # your chip; one line per chip
   set(MCU_HAL_TARGET stm32cubemx)
   set(MCU_DEVICE_HEADER stm32f4xx.h)  # stm32f7xx.h on an F7
   add_subdirectory(mcu-imu)
   ```

   and add `imu` to the link line CubeMX already wrote, so it reads:

   ```cmake
   target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx imu)
   ```

   Do not add `PRIVATE`: CubeMX's line has no such keyword, and CMake refuses
   to mix the two forms on one target.

   `stm32cubemx` is a CMake *target*, a named bundle of settings, that CubeMX
   makes to carry the HAL's include folders and defines; `MCU_HAL_TARGET`
   lends those to the library, which works with any CubeMX version.
   `MCU_DEVICE_HEADER` is your chip family's main header.

3. **Check the HAL module.** The MPU-6050 talks I2C, so hal_conf must have
   `#define HAL_I2C_MODULE_ENABLED`; enabling I2C1 (or any I2C) in CubeMX
   does that. The [HAL module table](#stm32-hal) lists what every chip
   needs.

4. **Write the smallest test.** It is
   [examples/stm/mpu6050/main.c](../examples/stm/mpu6050/main.c) fitted into
   CubeMX's `Core/Src/main.c`. Paste each part between the `USER CODE BEGIN`
   and `END` comments named, so a regenerate keeps it. The example's
   `i2c1_init()` is left out: CubeMX's `MX_I2C1_Init()` already does it.

   After `/* USER CODE BEGIN Includes */`:

   ```c
   #include "imu.h"
   #include "imu_log.h"
   #include "mpu6050.h"
   ```

   After `/* USER CODE BEGIN 2 */`, which comes after `MX_I2C1_Init()`:

   ```c
     /* 4 g, 500 dps, 100 Hz, as in examples/stm/mpu6050/main.c. */
     struct mpu6050_config cfg = {
       .accel_fs = MPU6050_ACCEL_FS_4G,
       .gyro_fs = MPU6050_GYRO_FS_500DPS,
       .dlpf = MPU6050_DLPF_44HZ,
       .sample_rate_hz = 100.0f,
     };
     static struct imu_dev imu; /* must start zeroed */

     if (mpu6050_probe(&imu, &hi2c1, &cfg))
       Error_Handler(); /* the probe has logged why */
   ```

   After `/* USER CODE BEGIN 3 */`, inside the `while (1)` loop:

   ```c
       struct imu_sample sample;
       bool ready = false;

       if (mpu6050_data_ready(&imu, &ready) || !ready) {
         HAL_Delay(1);
         continue;
       }
       if (imu_read(&imu, &sample) == 0)
         imu_log_sample(&imu, &sample);
   ```

   The lines go out through `printf()`: send that to a UART by writing
   newlib's `_write()`, and add this to `CMakeLists.txt` so numbers are
   printed:

   ```cmake
   target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)
   ```

   [API usage](usage.md) explains the calls.

5. **Build** as before, for example `cmake --build` with your build folder.
   A good build shows a `Linking C executable` line and no error. On the
   UART, the driver's first line reads
   `imu: mpu6050: at 0x68: 4 g, 500 dps, 100.00 Hz`.

**If it fails**

| You see | Why | Fix |
|---|---|---|
| `imu: no chip driver is selected; set(CONFIG_IMU_<CHIP> ON) before add_subdirectory()` (a CMake warning), then ``undefined reference to `mpu6050_probe'`` | No chip turned on, or turned on after `add_subdirectory(mcu-imu)` | Step 2 |
| `#error "IMU_MPU6050 needs the HAL I2C module: ...` | The HAL module is off | Enable the peripheral in CubeMX and regenerate |
| `#error "imu: CMSIS_device_header is not defined; ...` | `MCU_DEVICE_HEADER` is missing | Step 2 |
| `fatal error: stm32f4xx.h: No such file or directory` | `MCU_HAL_TARGET` is missing | Step 2 |
| `Target "stm32cubmx" not found.` (a CMake error) | `MCU_HAL_TARGET` is misspelt | Step 2 |
| `The plain signature for target_link_libraries has already been used with the target` | `PRIVATE` added to the link line | Step 2 |
| No error, but the numbers print as blanks | The C library (newlib-nano) leaves out `%f` | The `-u _printf_float` line of step 4 |

### Without CMake (STM32CubeIDE, a Makefile)

1. Copy `include/` and `src/` of this repository into your project, for
   example to `Middlewares/mcu-imu/`, keeping the chip folders inside `src/`.
2. Add every `.c` file under `src/` to the build, chip folders included.
3. Add two include paths, `Middlewares/mcu-imu/include` and
   `Middlewares/mcu-imu/src`, and two defines:
   `CMSIS_device_header="stm32f4xx.h"`, quotes included, and
   `CONFIG_IMU_MPU6050=1`, one per chip. CubeMX already defines
   `USE_HAL_DRIVER` and your device, such as `STM32F405xx`.
4. Steps 3 to 5 of the CMake guide apply as they are; `-u _printf_float`
   goes in the linker flags.

Where these settings usually live: in STM32CubeIDE, **Project → Properties →
C/C++ Build → Settings → Tool Settings → MCU GCC Compiler** (Include paths,
Preprocessor); in a CubeMX Makefile, `C_SOURCES`, `C_INCLUDES` and `C_DEFS`. The build errors are the ones in the table above;
the fix is the matching define or include path instead of a CMake line. One
more: `#include expects "FILENAME" or <FILENAME>` means `CMSIS_device_header`
lost its quotes.

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
# IMU_BMI270 is not set
# IMU_ICM42688 is not set
# IMU_JY901 is not set
# IMU_LIS2DH12 is not set
```

or with `make menu`. esp-forge turns each line into `CONFIG_IMU_<CHIP>=y` in
`sdkconfig.defaults`; stm-forge passes it as `-DCONFIG_IMU_<CHIP>=1`.
`IMU_DEBUG = y` turns on `imu_dbg()` lines on the HAL; on ESP-IDF the log
level decides.

## ESP-IDF

Setup: [ESP-IDF, step by step](#esp-idf-step-by-step). For reference:

- In `main/idf_component.yml`, `version` is a git ref; `stable` is the
  released one.
- A clone in `components/` is a local component, named after its folder;
  that name is what other components `REQUIRES`. The examples in
  `examples/esp/` need no name: their `main` lists no `REQUIRES`, and ESP-IDF
  gives `main` every component, so they build from any checkout name.
- With the subsystem on and no chip picked, the build warns.
- ESP-IDF 5.2 or newer is required for the `i2c_master` API.

## STM32 HAL

Setup: [With CMake](#with-cmake-a-cubemx-cmake-project) or
[Without CMake](#without-cmake-stm32cubeide-a-makefile). The
`add_subdirectory()` route builds a static library `imu`. For reference:

- Each `CONFIG_IMU_<CHIP>` (and `CONFIG_IMU_DEBUG`) that is true reaches the
  compiler as `CONFIG_IMU_<CHIP>=1`, for the library and your app.
- `MCU_HAL_TARGET` is a target that carries your `USE_HAL_DRIVER`, device
  define (`STM32F405xx`, ...) and the include dirs of the HAL, CMSIS and your
  `stm32f4xx_hal_conf.h`. The library copies only those include dirs and
  defines, never the target's sources, so this works with any CubeMX
  version. `MCU_DEVICE_HEADER` becomes `CMSIS_device_header`, which CubeMX
  does not define. Every `mcu-*` library reads these two, so set them once.
  Leave them unset if your project already provides both.
- The CPU flags (`-mcpu`, `-mfloat-abi`, ...) must reach the library too;
  CubeMX's toolchain file sets them for every target.
- Without CMake, `src/` only needs to be on the include path of this
  library's own sources; the defines are what stm-forge passes.

**No chip driver is built by default.** Like a kernel subsystem, the core is
always built and each driver is opt-in: a chip whose option is absent or 0
compiles to nothing. With the library on and no chip picked, the CMakeLists
warns (ESP-IDF and `add_subdirectory()`) and `src/imu_port_stm.c` emits a
`#warning`.

**The HAL modules are yours to enable.** The drivers call the HAL's I2C, SPI
and UART, so your app's `stm32f4xx_hal_conf.h` (or `stm32f7xx_hal_conf.h`) must
define the module each chip uses, and its `.c` must be in the build:

| Option | Needs in hal_conf | HAL source |
|---|---|---|
| `IMU_MPU6050` | `HAL_I2C_MODULE_ENABLED` | `stm32f{4,7}xx_hal_i2c.c` |
| `IMU_ICM45686` | `HAL_SPI_MODULE_ENABLED` | `stm32f{4,7}xx_hal_spi.c` |
| `IMU_BMI088` | both | both |
| `IMU_WT901` | `HAL_UART_MODULE_ENABLED` | `stm32f{4,7}xx_hal_uart.c` |
| `IMU_ICM42688` | `HAL_I2C_MODULE_ENABLED`, `HAL_SPI_MODULE_ENABLED` or both: each probe is built with its own module | the matching ones |
| `IMU_BMI270` | `HAL_I2C_MODULE_ENABLED`, `HAL_SPI_MODULE_ENABLED` or both: each probe is built with its own module | the matching ones |
| `IMU_JY901` | `HAL_UART_MODULE_ENABLED`, `HAL_I2C_MODULE_ENABLED` or both: each probe is built with its own module | the matching ones |
| `IMU_LIS2DH12` | `HAL_I2C_MODULE_ENABLED` | `stm32f{4,7}xx_hal_i2c.c` |

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
to your app's `CMakeLists.txt` (in a CubeMX project the target is
`${CMAKE_PROJECT_NAME}`), or `imu_log_sample()` prints blanks where the
numbers go.
