# ESP-IDF only. stm-forge never reads this file: it compiles every src/*.c of
# the library and adds include/, which is why the sources sit flat in src/ and
# each port file is empty on the other platform.
#
# The core is always built, even with CONFIG_IMU off; each chip driver only
# when its Kconfig option is on. Kconfig values are available here during the
# build phase, and only SRCS depends on them -- never REQUIRES, which IDF
# resolves before sdkconfig. That is also why IMU off still builds the core:
# an empty SRCS would need a second, differently-shaped register call, and an
# unreferenced core costs nothing once --gc-sections has run.
set(srcs "src/imu.c" "src/imu_log.c" "src/imu_port_esp.c")

if(CONFIG_IMU AND NOT (CONFIG_IMU_MPU6050 OR CONFIG_IMU_ICM45686 OR
                       CONFIG_IMU_BMI088))
    message(WARNING "imu: CONFIG_IMU is on but no chip driver is "
                    "selected; set CONFIG_IMU_<CHIP>=y "
                    "(esp-forge: IMU_<CHIP> = y)")
endif()

if(CONFIG_IMU_MPU6050)
    list(APPEND srcs "src/mpu6050.c")
endif()
if(CONFIG_IMU_ICM45686)
    list(APPEND srcs "src/icm45686.c")
endif()
if(CONFIG_IMU_BMI088)
    list(APPEND srcs "src/bmi088.c" "src/bmi088_bus.c")
endif()

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "src"
                       REQUIRES driver
                       PRIV_REQUIRES esp_timer)
