/**
 * @file altitude_controller.h
 * @brief Altitude PID controller
 */

#pragma once

#include "fc_types.h"
#include <stdbool.h>

/**
 * @brief Initialize altitude controller
 * @param params PID parameters
 */
void altitude_controller_init(const pid_params_t *params);

/**
 * @brief Update altitude controller
 * @param altitude_sp Altitude setpoint
 * @param altitude_est Current altitude estimate
 * @param dt Time since last update [s]
 * @return Vertical thrust command [0.0 to 1.0]
 */
float altitude_controller_update(
    const altitude_setpoint_t *altitude_sp,
    const altitude_est_t *altitude_est,
    float dt
);

/**
 * @brief Reset controller integrator
 */
void altitude_controller_reset(void);

/**
 * @brief Enable/disable altitude hold mode
 */
void altitude_controller_set_enabled(bool enabled);

/**
 * @brief Check if altitude hold is enabled
 */
bool altitude_controller_is_enabled(void);
