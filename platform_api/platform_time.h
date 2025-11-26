/**
 * @file platform_time.h
 * @brief Platform-independent time interface
 * 
 * Abstract interface for time and delay functions.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Get current time in milliseconds
 * @return Current time [ms]
 */
uint32_t platform_millis(void);

/**
 * @brief Get current time in microseconds
 * @return Current time [us]
 */
uint64_t platform_micros(void);

/**
 * @brief Sleep for specified milliseconds
 * @param ms Milliseconds to sleep
 */
void platform_sleep_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif
