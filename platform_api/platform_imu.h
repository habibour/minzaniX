/**
 * @file platform_imu.h
 * @brief Platform-independent IMU interface
 * 
 * Abstract interface for reading IMU data. Each platform (sim, STM32, etc.)
 * must implement these functions.
 */

#pragma once

#include "../core/fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the IMU
 * @return true if successful
 */
bool platform_imu_init(void);

/**
 * @brief Read IMU sample
 * @param out Output IMU sample
 * @return true if successful
 */
bool platform_imu_read(imu_sample_t *out);

#ifdef __cplusplus
}
#endif
