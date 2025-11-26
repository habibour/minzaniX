/**
 * @file fc_interface.c
 * @brief Flight controller main interface implementation
 */

#include "fc_interface.h"
#include "estimator.h"
#include "controller.h"
#include "altitude_estimator.h"
#include "altitude_controller.h"
#include "safety.h"
#include "params.h"
#include <stddef.h>

// Static altitude state
static altitude_est_t s_altitude_est = {0};
static altitude_setpoint_t s_altitude_sp = {.altitude_sp = 1.0f, .velocity_sp = 0.0f};
static bool s_altitude_initialized = false;
static attitude_setpoint_t s_cached_att_sp = {0};

void fc_init(void) {
    estimator_reset();
    controller_reset();
    
    // Initialize altitude control with default PID params
    pid_params_t params;
    params_get_pid(&params);
    altitude_controller_init(&params);
    altitude_controller_set_enabled(false);
    
    safety_set_armed(false);
    s_altitude_initialized = false;
}

void fc_set_armed(bool arm) {
    safety_set_armed(arm);
}

void fc_set_attitude_setpoint(const attitude_setpoint_t *sp) {
    s_cached_att_sp = *sp;
    controller_set_sp(sp);
}

void fc_set_altitude_setpoint(const altitude_setpoint_t *sp) {
    s_altitude_sp = *sp;
}

void fc_set_altitude_hold_enabled(bool enabled) {
    altitude_controller_set_enabled(enabled);
}

void fc_step(const imu_sample_t *imu, const baro_sample_t *baro, float dt, motor_output_t *out) {
    // Update attitude estimator with IMU data
    estimator_update(imu, dt);
    
    // Get current attitude
    attitude_t att;
    estimator_get_attitude(&att);
    
    // Update altitude estimator if barometer available
    if (baro != NULL) {
        if (!s_altitude_initialized) {
            altitude_estimator_init(baro->pressure);
            s_altitude_initialized = true;
        }
        altitude_estimator_update(baro, imu, &att, dt, &s_altitude_est);
    }
    
    // Prepare attitude setpoint
    attitude_setpoint_t att_sp_working = s_cached_att_sp;
    
    // If altitude hold is enabled, override thrust from altitude controller
    if (altitude_controller_is_enabled() && baro != NULL) {
        float altitude_thrust = altitude_controller_update(&s_altitude_sp, &s_altitude_est, dt);
        att_sp_working.thrust_sp = altitude_thrust;
    }
    
    controller_set_sp(&att_sp_working);
    controller_update(&att, dt, out);
    
    // Apply safety checks
    safety_apply(out);
}

void fc_get_attitude(attitude_t *att) {
    estimator_get_attitude(att);
}

void fc_get_altitude(altitude_est_t *alt) {
    *alt = s_altitude_est;
}
