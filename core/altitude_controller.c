/**
 * @file altitude_controller.c
 * @brief Cascaded PID controller for altitude hold
 * 
 * Two-loop control:
 * 1. Outer loop: Position (altitude) -> velocity setpoint
 * 2. Inner loop: Velocity -> thrust command
 */

#include "altitude_controller.h"
#include <math.h>

// Limits
#define MAX_VERTICAL_VELOCITY 1.0f   // m/s (allow faster climb now)
#define MIN_VERTICAL_VELOCITY -1.0f  // m/s
#define MAX_THRUST 0.90f             // Maximum thrust command
#define MIN_THRUST 0.70f             // Minimum thrust command
#define HOVER_THRUST 0.80f           // Nominal hover thrust (found experimentally)

// PID state - Position loop
static float s_alt_kp = 0.5f;
static float s_alt_ki = 0.1f;
static float s_alt_kd = 0.2f;
static float s_alt_integrator = 0.0f;
static float s_alt_prev_error = 0.0f;

// PID state - Velocity loop
static float s_vel_kp = 0.4f;
static float s_vel_ki = 0.15f;
static float s_vel_kd = 0.05f;
static float s_vel_integrator = 0.0f;
static float s_vel_prev_error = 0.0f;

static bool s_enabled = false;
static bool s_initialized = false;

// Integrator anti-windup limits
#define MAX_ALT_INTEGRATOR 1.0f
#define MAX_VEL_INTEGRATOR 0.5f

/**
 * @brief Clamp value between min and max
 */
static float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void altitude_controller_init(const pid_params_t *params) {
    s_alt_kp = params->alt_kp;
    s_alt_ki = params->alt_ki;
    s_alt_kd = params->alt_kd;
    s_vel_kp = params->alt_vel_kp;
    s_vel_ki = params->alt_vel_ki;
    s_vel_kd = params->alt_vel_kd;
    
    altitude_controller_reset();
    s_initialized = true;
}

float altitude_controller_update(
    const altitude_setpoint_t *altitude_sp,
    const altitude_est_t *altitude_est,
    float dt
) {
    if (!s_enabled || !s_initialized) {
        return HOVER_THRUST;
    }

    // === Outer Loop: Altitude Position Control ===
    float altitude_error = altitude_sp->altitude_sp - altitude_est->altitude;
    
    // Proportional
    float alt_p_term = s_alt_kp * altitude_error;
    
    // Integral (with anti-windup)
    s_alt_integrator += altitude_error * dt;
    s_alt_integrator = clamp(s_alt_integrator, -MAX_ALT_INTEGRATOR, MAX_ALT_INTEGRATOR);
    float alt_i_term = s_alt_ki * s_alt_integrator;
    
    // Derivative
    float altitude_error_rate = 0.0f;
    if (dt > 0.001f) {  // Avoid division by zero
        altitude_error_rate = (altitude_error - s_alt_prev_error) / dt;
    }
    s_alt_prev_error = altitude_error;
    float alt_d_term = s_alt_kd * altitude_error_rate;
    
    // Output: Desired vertical velocity
    float velocity_sp = alt_p_term + alt_i_term + alt_d_term;
    velocity_sp = clamp(velocity_sp, MIN_VERTICAL_VELOCITY, MAX_VERTICAL_VELOCITY);
    
    // === Inner Loop: Velocity Control ===
    float velocity_error = velocity_sp - altitude_est->velocity;
    
    // Proportional
    float vel_p_term = s_vel_kp * velocity_error;
    
    // Integral (with anti-windup)
    s_vel_integrator += velocity_error * dt;
    s_vel_integrator = clamp(s_vel_integrator, -MAX_VEL_INTEGRATOR, MAX_VEL_INTEGRATOR);
    float vel_i_term = s_vel_ki * s_vel_integrator;
    
    // Derivative
    float velocity_error_rate = 0.0f;
    if (dt > 0.001f) {  // Avoid division by zero
        velocity_error_rate = (velocity_error - s_vel_prev_error) / dt;
    }
    s_vel_prev_error = velocity_error;
    float vel_d_term = s_vel_kd * velocity_error_rate;
    
    // Output: Thrust adjustment
    float thrust_adjustment = vel_p_term + vel_i_term + vel_d_term;
    
    // Final thrust = hover + adjustment
    float thrust_cmd = HOVER_THRUST + thrust_adjustment;
    thrust_cmd = clamp(thrust_cmd, MIN_THRUST, MAX_THRUST);
    
    return thrust_cmd;
}

void altitude_controller_reset(void) {
    s_alt_integrator = 0.0f;
    s_alt_prev_error = 0.0f;
    s_vel_integrator = 0.0f;
    s_vel_prev_error = 0.0f;
}

void altitude_controller_set_enabled(bool enabled) {
    if (enabled && !s_enabled) {
        // Enabling: reset integrators
        altitude_controller_reset();
    }
    s_enabled = enabled;
}

bool altitude_controller_is_enabled(void) {
    return s_enabled;
}
