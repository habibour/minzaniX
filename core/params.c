/**
 * @file params.c
 * @brief Parameter storage and management implementation
 */

#include "params.h"

// Global PID parameters (accessible for runtime tuning)
// Tuned for simulation - very gentle gains to prevent oscillations
pid_params_t g_pid_params = {
    .roll_kp = 1.5f, .roll_ki = 0.0f, .roll_kd = 0.3f,
    .pitch_kp = 1.5f, .pitch_ki = 0.0f, .pitch_kd = 0.3f,
    .yaw_kp = 2.0f, .yaw_ki = 0.0f, .yaw_kd = 0.5f,
    // Altitude control - reduced from ZN tuning for stability
    .alt_kp = 0.15f, .alt_ki = 0.005f, .alt_kd = 0.08f,
    .alt_vel_kp = 0.3f, .alt_vel_ki = 0.01f, .alt_vel_kd = 0.05f
};

void params_init(void) {
    // Already initialized with defaults above
}

bool params_load(void) {
    // Platform-specific implementation (EEPROM, flash, file, etc.)
    // For now, just use defaults
    return true;
}

bool params_save(void) {
    // Platform-specific implementation
    return true;
}

void params_get_pid(pid_params_t *params) {
    if (params) {
        *params = g_pid_params;
    }
}

void params_set_pid(const pid_params_t *params) {
    if (params) {
        g_pid_params = *params;
    }
}

pid_params_t* params_get_pid_ptr(void) {
    return &g_pid_params;
}
