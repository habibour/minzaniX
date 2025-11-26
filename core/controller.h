/**
 * @file controller.h
 * @brief PID attitude controller interface
 */

#pragma once

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize/reset controller
 */
void controller_reset(void);

/**
 * @brief Set attitude setpoint
 * @param sp Attitude setpoint
 */
void controller_set_sp(const attitude_setpoint_t *sp);

/**
 * @brief Update controller and compute motor outputs
 * @param att Current attitude
 * @param dt Time step [seconds]
 * @param out Output motor commands
 */
void controller_update(const attitude_t *att, float dt, motor_output_t *out);

/**
 * @brief Set PID parameters
 * @param params PID parameters
 */
void controller_set_params(const pid_params_t *params);

#ifdef __cplusplus
}
#endif
