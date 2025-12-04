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
#define MAX_VERTICAL_VELOCITY 0.8f   // m/s - slow gentle approach
#define MIN_VERTICAL_VELOCITY -0.8f  // m/s - slow gentle descent
#define MAX_THRUST 0.85f             // Maximum thrust command (1.7x hover)
#define MIN_THRUST 0.15f             // Minimum thrust command (0.3x hover)
#define HOVER_THRUST 0.50f           // Nominal hover thrust

// PID state - Position loop (Gentle to avoid saturation)
static float s_alt_kp = 0.02f;  // Very gentle P
static float s_alt_ki = 0.001f; // Minimal I
static float s_alt_kd = 0.01f;  // Light D
static float s_alt_integrator = 0.0f;
static float s_alt_prev_error = 0.0f;

// PID state - Velocity loop (Gentle)
static float s_vel_kp = 0.04f;  // Gentle P
static float s_vel_ki = 0.002f; // Minimal I
static float s_vel_kd = 0.01f;  // Light damping
static float s_vel_integrator = 0.0f;
static float s_vel_prev_error = 0.0f;

static bool s_enabled = false;
static bool s_initialized = false;

// Integrator anti-windup limits
#define MAX_ALT_INTEGRATOR 0.2f
#define MAX_VEL_INTEGRATOR 0.1f

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
    // dt now used for integral term
    
    if (!s_enabled || !s_initialized) {
        return HOVER_THRUST;
    }

    // === SIMPLIFIED: Single-loop PID controller ===
    // Direct altitude error to thrust (no velocity cascade)
    float altitude_error = altitude_sp->altitude_sp - altitude_est->altitude;
    
    // Dead-band for P and D (but NOT I - integrator needs all error)
    const float dead_band = 0.5f;  // ±0.5m no action zone
    float error_for_pd = altitude_error;
    if (altitude_error > -dead_band && altitude_error < dead_band) {
        error_for_pd = 0.0f;
    }
    
    // PID control: P for position, I for steady-state, D for damping
    const float simple_kp = 0.020f;     // Position gain
    const float simple_ki = 0.002f;     // Integral gain (eliminates steady-state error)
    const float simple_kd = 0.080f;     // Velocity damping
    
    float p_term = simple_kp * error_for_pd;
    
    // Integral term (no dead-band - accumulates all error)
    s_alt_integrator += altitude_error * dt;
    s_alt_integrator = clamp(s_alt_integrator, -5.0f, 5.0f);  // Anti-windup
    float i_term = simple_ki * s_alt_integrator;
    
    float d_term = -simple_kd * altitude_est->velocity;  // Opposes motion
    
    float thrust_adjustment = p_term + i_term + d_term;
    
    // Rate limiting: max 0.08 change per iteration
    static float last_thrust = HOVER_THRUST;
    float thrust_cmd = HOVER_THRUST + thrust_adjustment;
    
    const float max_thrust_delta = 0.08f;
    if (thrust_cmd - last_thrust > max_thrust_delta) {
        thrust_cmd = last_thrust + max_thrust_delta;
    } else if (last_thrust - thrust_cmd > max_thrust_delta) {
        thrust_cmd = last_thrust - max_thrust_delta;
    }
    last_thrust = thrust_cmd;
    
    // Original cascaded code commented out for now
    /*
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
    float thrust_adjustment_old = vel_p_term + vel_i_term + vel_d_term;
    */
    
    // Final thrust already computed above in simplified controller
    // float thrust_cmd = HOVER_THRUST + thrust_adjustment;
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
