/**
 * @file platform_pwm.h
 * @brief Platform-independent PWM output interface
 * 
 * Abstract interface for controlling motor PWM outputs.
 */

#pragma once

#include "../core/fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize PWM outputs
 * @return true if successful
 */
bool platform_pwm_init(void);

/**
 * @brief Write motor commands to PWM outputs
 * @param motors Motor output values [0.0 to 1.0]
 * @return true if successful
 */
bool platform_pwm_write(const motor_output_t *motors);

#ifdef __cplusplus
}
#endif
