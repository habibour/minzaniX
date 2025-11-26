/**
 * @file fc_interface.h
 * @brief High-level flight controller API
 * 
 * Main interface for integrating the flight controller into different platforms.
 */

#pragma once

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the flight controller
 */
void fc_init(void);

/**
 * @brief Set armed state
 * @param arm true to arm, false to disarm
 */
void fc_set_armed(bool arm);

/**
 * @brief Set attitude setpoint
 * @param sp Pointer to attitude setpoint
 */
void fc_set_attitude_setpoint(const attitude_setpoint_t *sp);

/**
 * @brief Set altitude setpoint for altitude hold mode
 * @param sp Pointer to altitude setpoint
 */
void fc_set_altitude_setpoint(const altitude_setpoint_t *sp);

/**
 * @brief Enable/disable altitude hold mode
 * @param enabled true to enable altitude hold
 */
void fc_set_altitude_hold_enabled(bool enabled);

/**
 * @brief Main flight controller step
 * @param imu IMU sample
 * @param baro Barometer sample (can be NULL if no barometer)
 * @param dt Time step [seconds]
 * @param out Output motor commands
 */
void fc_step(const imu_sample_t *imu, const baro_sample_t *baro, float dt, motor_output_t *out);

/**
 * @brief Get current attitude estimate
 * @param att Output attitude
 */
void fc_get_attitude(attitude_t *att);

/**
 * @brief Get current altitude estimate
 * @param alt Output altitude estimate
 */
void fc_get_altitude(altitude_est_t *alt);

#ifdef __cplusplus
}
#endif
