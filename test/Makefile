CC ?= cc
# gnu11, not c11: the imu_* log macros use `, ##__VA_ARGS__`.
CFLAGS += -std=gnu11 -Wall -Wextra -Werror -pedantic
# src/ for the common private header, and every chip folder, because the
# tests include a chip's private headers by plain name.
CPPFLAGS += -I../include -I../src $(addprefix -I,$(wildcard ../src/*/))

# Two full suites, one per platform: the same chip drivers through
# src/imu_port_esp.c against idf_stubs/ (esp/), and through src/imu_port_stm.c
# against hal_stubs/ (stm/). The maths tests touch no platform, so they run
# once.

# What ESP-IDF passes: ESP_PLATFORM, and every chip on.
ESP_CPPFLAGS := -Iidf_stubs -DESP_PLATFORM \
		-DCONFIG_IMU_MPU6050=1 -DCONFIG_IMU_ICM45686=1 \
		-DCONFIG_IMU_BMI088=1 -DCONFIG_IMU_WT901=1 \
		-DCONFIG_IMU_BMI270=1 -DCONFIG_IMU_ICM42688=1 \
		-DCONFIG_IMU_JY901=1 -DCONFIG_IMU_LIS2DH12=1
# What stm-forge passes: the family header the code includes, the HAL modules
# an app's hal_conf would enable, and every chip option on (IMU_<CHIP> = y).
STM_CPPFLAGS := -Ihal_stubs \
		-DCMSIS_device_header='"stm32_hal_stub.h"' \
		-DHAL_I2C_MODULE_ENABLED -DHAL_SPI_MODULE_ENABLED \
		-DHAL_UART_MODULE_ENABLED \
		-DCONFIG_IMU_MPU6050=1 -DCONFIG_IMU_ICM45686=1 \
		-DCONFIG_IMU_BMI088=1 -DCONFIG_IMU_WT901=1 \
		-DCONFIG_IMU_BMI270=1 -DCONFIG_IMU_ICM42688=1 \
		-DCONFIG_IMU_JY901=1 -DCONFIG_IMU_LIS2DH12=1 \
		-DCONFIG_IMU_DEBUG=1

BMI270   := ../src/bmi270/bmi270.c ../src/bmi270/bmi270_config_file.c

# The core and the stubs every binary of a suite links.
CORE     := ../src/imu.c ../src/imu_log.c
ESP_CORE := $(CORE) ../src/imu_port_esp.c idf_stubs/idf_stubs.c
STM_CORE := $(CORE) ../src/imu_port_stm.c hal_stubs/hal_stubs.c

MATH_TESTS := test_mpu6050_math test_icm45686_math test_wt901_math \
	      test_bmi270_math test_icm42688_math test_jy901_math \
	      test_lis2dh12_math
SUITE      := test_core test_mpu6050 test_icm45686 test_bmi088_core \
	      test_bmi088 test_wt901 test_bmi270 test_icm42688 test_jy901 \
	      test_lis2dh12
# Plus one port test each, for the UART ops no chip uses yet.
ESP_TESTS  := $(addprefix esp/,$(SUITE) test_port_esp)
STM_TESTS  := $(addprefix stm/,$(SUITE) test_port_stm)
TESTS      := $(MATH_TESTS) $(ESP_TESTS) $(STM_TESTS)

.PHONY: all clean test

all: $(TESTS)

test_mpu6050_math: test_mpu6050_math.c
test_icm45686_math: test_icm45686_math.c
test_wt901_math: test_wt901_math.c
test_bmi270_math: test_bmi270_math.c
test_icm42688_math: test_icm42688_math.c
test_jy901_math: test_jy901_math.c
test_lis2dh12_math: test_lis2dh12_math.c

esp/test_core: esp/test_core.c $(ESP_CORE)
esp/test_mpu6050: esp/test_mpu6050.c ../src/mpu6050/mpu6050.c \
		  $(ESP_CORE)
esp/test_icm45686: esp/test_icm45686.c ../src/icm45686/icm45686.c \
		   $(ESP_CORE)
esp/test_bmi088_core: esp/test_bmi088_core.c ../src/bmi088/bmi088.c \
		      $(ESP_CORE)
esp/test_bmi088: esp/test_bmi088.c ../src/bmi088/bmi088.c \
		 ../src/bmi088/bmi088_bus.c \
		 $(ESP_CORE)
esp/test_wt901: esp/test_wt901.c ../src/wt901/wt901.c $(ESP_CORE)
esp/test_port_esp: esp/test_port_esp.c $(ESP_CORE)
esp/test_bmi270: esp/test_bmi270.c $(BMI270) $(ESP_CORE)
esp/test_icm42688: esp/test_icm42688.c ../src/icm42688/icm42688.c \
		   $(ESP_CORE)
esp/test_jy901: esp/test_jy901.c ../src/jy901/jy901.c $(ESP_CORE)
esp/test_lis2dh12: esp/test_lis2dh12.c ../src/lis2dh12/lis2dh12.c \
		   $(ESP_CORE)

stm/test_core: stm/test_core.c $(STM_CORE)
stm/test_mpu6050: stm/test_mpu6050.c ../src/mpu6050/mpu6050.c \
		  $(STM_CORE)
stm/test_icm45686: stm/test_icm45686.c ../src/icm45686/icm45686.c \
		   $(STM_CORE)
stm/test_bmi088_core: stm/test_bmi088_core.c ../src/bmi088/bmi088.c \
		      $(STM_CORE)
stm/test_bmi088: stm/test_bmi088.c ../src/bmi088/bmi088.c \
		 ../src/bmi088/bmi088_bus.c \
		 $(STM_CORE)
stm/test_wt901: stm/test_wt901.c ../src/wt901/wt901.c $(STM_CORE)
stm/test_port_stm: stm/test_port_stm.c $(STM_CORE)
stm/test_bmi270: stm/test_bmi270.c $(BMI270) $(STM_CORE)
stm/test_icm42688: stm/test_icm42688.c ../src/icm42688/icm42688.c \
		   $(STM_CORE)
stm/test_jy901: stm/test_jy901.c ../src/jy901/jy901.c $(STM_CORE)
stm/test_lis2dh12: stm/test_lis2dh12.c ../src/lis2dh12/lis2dh12.c \
		   $(STM_CORE)

$(ESP_TESTS): CPPFLAGS += $(ESP_CPPFLAGS)
$(STM_TESTS): CPPFLAGS += $(STM_CPPFLAGS)

# Lets the driver tests count calloc() calls, to prove a refused probe
# allocates nothing -- and on the HAL free() too, that a failed one leaks
# nothing. GNU ld; only the drivers' own references are wrapped.
esp/test_mpu6050 esp/test_icm45686 esp/test_bmi088: \
	LDFLAGS += -Wl,--wrap=calloc
# Every printf() in a binary that links the HAL stubs also lands in
# stub_log_last, so a test can check what a driver logged.
$(STM_TESTS): LDFLAGS += -Wl,--wrap=printf
stm/test_mpu6050 stm/test_icm45686 stm/test_bmi088: \
	LDFLAGS += -Wl,--wrap=calloc -Wl,--wrap=free
# The BMI270 and ICM-42688-P cases are shared too, and count both on both
# platforms.
esp/test_bmi270 stm/test_bmi270 esp/test_icm42688 stm/test_icm42688: \
	LDFLAGS += -Wl,--wrap=calloc -Wl,--wrap=free
# The WT901 and JY901 cases are shared, so their binaries count both.
esp/test_wt901 stm/test_wt901 esp/test_jy901 stm/test_jy901: LDFLAGS += -Wl,--wrap=calloc -Wl,--wrap=free
# The LIS2DH12 cases are shared as well.
esp/test_lis2dh12 stm/test_lis2dh12: LDFLAGS += -Wl,--wrap=calloc -Wl,--wrap=free

$(TESTS):
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) -o $@ -lm

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "./$$t"; ./$$t; done

clean:
	rm -f $(TESTS)

# Headers are prerequisites so that editing one rebuilds. Without this, `make
# test` reruns a stale binary and reports green for code it never compiled.
# Recipes use $(filter %.c,$^) so the headers never reach the compiler.
HDRS  := $(wildcard ../include/*.h ../src/*.h ../src/*/*.h)
STUBS := $(wildcard idf_stubs/*.h idf_stubs/*/*.h hal_stubs/*.h) \
	 $(wildcard *_cases.h)
$(TESTS): $(HDRS) $(STUBS)
