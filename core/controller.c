/**
 * @file controller.c
 * @brief PID attitude controller implementation
 */

#include "controller.h"
#include "mixer.h"

// Default PID gains
static pid_params_t g_params = {
    .roll_kp = 1.5f, .roll_ki = 0.0f, .roll_kd = 0.3f,
    .pitch_kp = 1.5f, .pitch_ki = 0.0f, .pitch_kd = 0.3f,
    .yaw_kp = 2.0f, .yaw_ki = 0.0f, .yaw_kd = 0.5f
};

static attitude_setpoint_t g_sp = {0};
static float g_roll_integral = 0.0f;
static float g_pitch_integral = 0.0f;
static float g_yaw_integral = 0.0f;

void controller_reset(void) {
    g_sp.roll_sp = 0.0f;
    g_sp.pitch_sp = 0.0f;
    g_sp.yaw_sp = 0.0f;
    g_sp.thrust_sp = 0.0f;
    
    g_roll_integral = 0.0f;
    g_pitch_integral = 0.0f;
    g_yaw_integral = 0.0f;
}

void controller_set_sp(const attitude_setpoint_t *sp) {
    if (sp) {
        g_sp = *sp;
    }
}

void controller_update(const attitude_t *att, float dt, motor_output_t *out) {
    // Calculate errors
    float e_roll = g_sp.roll_sp - att->roll;
    float e_pitch = g_sp.pitch_sp - att->pitch;
    float e_yaw = g_sp.yaw_sp - att->yaw;
    
    // Update integrals
    g_roll_integral += e_roll * dt;
    g_pitch_integral += e_pitch * dt;
    g_yaw_integral += e_yaw * dt;
    
    // PD control (no integral for now to avoid windup)
    float roll_cmd = 
        g_params.roll_kp * e_roll - 
        g_params.roll_kd * att->roll_rate;
    
    float pitch_cmd = 
        g_params.pitch_kp * e_pitch - 
        g_params.pitch_kd * att->pitch_rate;
    
    float yaw_cmd = 
        g_params.yaw_kp * e_yaw - 
        g_params.yaw_kd * att->yaw_rate;
    
    // Mix commands into motor outputs
    mixer_mix(g_sp.thrust_sp, roll_cmd, pitch_cmd, yaw_cmd, out);
}

void controller_set_params(const pid_params_t *params) {
    if (params) {
        g_params = *params;
    }
}
