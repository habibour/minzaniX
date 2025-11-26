/**
 * @file platform_baro.h
 * @brief Platform abstraction for barometer sensor
 */

#pragma once

#include "core/fc_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize barometer
 * @return true if successful
 */
bool platform_baro_init(void);

/**
 * @brief Read barometer data
 * @param sample Output barometer sample
 * @return true if read successful
 */
bool platform_baro_read(baro_sample_t *sample);

#ifdef __cplusplus
}
#endif
