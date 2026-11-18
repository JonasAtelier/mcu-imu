# Read by ESP-IDF (and so esp-forge) as a component, and by any other CMake
# project that add_subdirectory()s this repo, such as a CubeMX-generated STM32
# one. stm-forge never reads it: it compiles every .c under src/, chip folders
# included, puts src/ on the include path of this library's sources and adds
# include/ -- which is why each port file is empty on the other platform.
# ESP_PLATFORM tells the two apart; IDF sets it in its early requirements pass
# as well as in the build.

if(NOT ESP_PLATFORM)
    # A static library `imu`. Every .c under src/, as stm-forge does: a chip
    # file compiles to nothing unless its CONFIG_IMU_<CHIP> is 1, so there is
    # no source list here to keep in step with the IDF one below.
    file(GLOB_RECURSE imu_sources CONFIGURE_DEPENDS src/*.c)
    add_library(imu STATIC ${imu_sources})
    # include/ for the app; src/ only for this library's own sources, so a
    # chip folder reaches imu_priv.h by plain name and no private header is
    # ever on the app's include path.
    target_include_directories(imu PUBLIC include PRIVATE src)
    # roundf() in the chip drivers.
    target_link_libraries(imu PUBLIC m)

    # The options are the Kconfig bools, read from Kconfig so a new chip needs
    # no line here: a true CMake variable CONFIG_IMU_<CHIP> (or
    # CONFIG_IMU_DEBUG) becomes CONFIG_IMU_<CHIP>=1, what stm-forge passes.
    # Only bools: an int option cannot be 1-or-absent. Each `config` line in
    # Kconfig is followed directly by its type line, which is what the match
    # relies on. PUBLIC, because imu_log.h reads CONFIG_IMU_DEBUG in the app's
    # sources.
    file(READ Kconfig imu_kconfig)
    string(REGEX MATCHALL "config IMU_[A-Z0-9_]+[ \t\r\n]+bool" imu_options
           "${imu_kconfig}")
    list(TRANSFORM imu_options REPLACE "config (IMU_[A-Z0-9_]+)[ \t\r\n]+bool"
         "CONFIG_\\1")
    set(imu_chips_on "")
    foreach(option IN LISTS imu_options)
        if(${option})
            target_compile_definitions(imu PUBLIC ${option}=1)
            if(NOT option STREQUAL "CONFIG_IMU_DEBUG")
                list(APPEND imu_chips_on ${option})
            endif()
        endif()
    endforeach()
    if(NOT imu_chips_on)
        message(WARNING "imu: no chip driver is selected; "
                        "set(CONFIG_IMU_<CHIP> ON) before add_subdirectory()")
    endif()

    # The HAL is the app's. MCU_HAL_TARGET names a target carrying its include
    # dirs and defines (USE_HAL_DRIVER, the device, the hal_conf.h directory)
    # -- stm32cubemx in a CubeMX-generated project; MCU_DEVICE_HEADER is the
    # family header the public headers include as CMSIS_device_header, which
    # CubeMX does not define. Both are shared by every mcu-* library, so an
    # app sets them once. Left unset, the app provides them its own way.
    #
    # Only the target's include dirs and defines are copied, the target is
    # never linked: CubeMX up to 6.13 also hangs main.c, the HAL and the
    # startup file on stm32cubemx as INTERFACE sources, which linking would
    # compile into libimu.a. A misspelt name fails at generate time.
    if(MCU_HAL_TARGET)
        target_include_directories(imu PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_INCLUDE_DIRECTORIES>)
        target_compile_definitions(imu PRIVATE
            $<TARGET_PROPERTY:${MCU_HAL_TARGET},INTERFACE_COMPILE_DEFINITIONS>)
    endif()
    if(MCU_DEVICE_HEADER)
        target_compile_definitions(imu PUBLIC
            "CMSIS_device_header=\"${MCU_DEVICE_HEADER}\"")
    endif()
    return()
endif()

# ESP-IDF: the chip files are listed one by one, and PRIV_INCLUDE_DIRS "src"
# lets a chip folder reach imu_priv.h by plain name.
#
# The core is always built, even with CONFIG_IMU off; each chip driver only
# when its Kconfig option is on. Kconfig values are available here during the
# build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why IMU off still builds the core:
# an empty SRCS would need a second, differently-shaped register call, and an
# unreferenced core costs nothing once --gc-sections has run.
set(srcs "src/imu.c" "src/imu_log.c" "src/imu_port_esp.c")

if(CONFIG_IMU AND NOT (CONFIG_IMU_MPU6050 OR CONFIG_IMU_ICM45686 OR
                       CONFIG_IMU_BMI088 OR CONFIG_IMU_WT901 OR
                       CONFIG_IMU_BMI270 OR CONFIG_IMU_ICM42688 OR
                       CONFIG_IMU_JY901 OR CONFIG_IMU_LIS2DH12))
    message(WARNING "imu: CONFIG_IMU is on but no chip driver is "
                    "selected; set CONFIG_IMU_<CHIP>=y "
                    "(esp-forge: IMU_<CHIP> = y)")
endif()

if(CONFIG_IMU_MPU6050)
    list(APPEND srcs "src/mpu6050/mpu6050.c")
endif()
if(CONFIG_IMU_ICM45686)
    list(APPEND srcs "src/icm45686/icm45686.c")
endif()
if(CONFIG_IMU_BMI088)
    list(APPEND srcs "src/bmi088/bmi088.c" "src/bmi088/bmi088_bus.c")
endif()
if(CONFIG_IMU_WT901)
    list(APPEND srcs "src/wt901/wt901.c")
endif()
if(CONFIG_IMU_ICM42688)
    list(APPEND srcs "src/icm42688/icm42688.c")
endif()
if(CONFIG_IMU_JY901)
    list(APPEND srcs "src/jy901/jy901.c")
endif()
if(CONFIG_IMU_BMI270)
    list(APPEND srcs "src/bmi270/bmi270.c" "src/bmi270/bmi270_config_file.c")
endif()
if(CONFIG_IMU_LIS2DH12)
    list(APPEND srcs "src/lis2dh12/lis2dh12.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver
                       PRIV_REQUIRES esp_timer)
