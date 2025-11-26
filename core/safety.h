/**
 * @file safety.h
 * @brief Safety and arming logic interface
 */

#pragma once

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set armed state
 * @param armed true to arm, false to disarm
 */
void safety_set_armed(bool armed);

/**
 * @brief Check if system is armed
 * @return true if armed
 */
bool safety_is_armed(void);

/**
 * @brief Apply safety checks to motor output
 * @param out Motor output (modified in place)
 */
void safety_apply(motor_output_t *out);

#ifdef __cplusplus
}
#endif
