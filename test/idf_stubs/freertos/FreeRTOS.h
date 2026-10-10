#ifndef FREERTOS_H
#define FREERTOS_H

#include <stdint.h>

/*
 * The default ESP-IDF tick rate. Deliberately kept at 100 Hz so the stub
 * reproduces pdMS_TO_TICKS() truncating a 1 ms delay to zero ticks.
 */
#define configTICK_RATE_HZ	100

typedef uint32_t TickType_t;

#define pdMS_TO_TICKS(msec) \
	((TickType_t)(((uint32_t)(msec) * configTICK_RATE_HZ) / 1000U))

#endif
