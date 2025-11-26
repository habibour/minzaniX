/**
 * @file pid_tuning.h
 * @brief PID tuning utilities including Ziegler-Nichols method
 */

#ifndef PID_TUNING_H
#define PID_TUNING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID controller type
 */
typedef enum {
    PID_ROLL = 0,
    PID_PITCH,
    PID_YAW,
    PID_ALTITUDE,
    PID_ALTITUDE_VELOCITY
} pid_controller_t;

/**
 * @brief Update PID gains for a specific controller at runtime
 * @param controller Which PID controller to update
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @return true if successful
 */
bool pid_update_gains(pid_controller_t controller, float kp, float ki, float kd);

/**
 * @brief Get current PID gains for a controller
 * @param controller Which PID controller to query
 * @param kp Output: Proportional gain
 * @param ki Output: Integral gain
 * @param kd Output: Derivative gain
 * @return true if successful
 */
bool pid_get_gains(pid_controller_t controller, float* kp, float* ki, float* kd);

/**
 * @brief Ziegler-Nichols tuning parameters
 */
typedef struct {
    float ku;           // Ultimate gain (critical gain where system oscillates)
    float tu;           // Ultimate period (oscillation period at Ku)
    bool use_classic;   // true for classic ZN, false for "some overshoot" variant
} ziegler_nichols_params_t;

/**
 * @brief Calculate PID gains using Ziegler-Nichols method
 * 
 * Classic Ziegler-Nichols tuning rules:
 * - P controller:   Kp = 0.5*Ku
 * - PI controller:  Kp = 0.45*Ku,  Ki = 0.54*Ku/Tu
 * - PID controller: Kp = 0.6*Ku,   Ki = 1.2*Ku/Tu,  Kd = 0.075*Ku*Tu
 * 
 * "Some overshoot" variant:
 * - PID controller: Kp = 0.33*Ku,  Ki = 0.66*Ku/Tu, Kd = 0.11*Ku*Tu
 * 
 * @param controller Which PID controller to tune
 * @param params Ziegler-Nichols tuning parameters (Ku and Tu)
 * @return true if successful
 */
bool pid_tune_ziegler_nichols(pid_controller_t controller, const ziegler_nichols_params_t* params);

/**
 * @brief Auto-tune using relay feedback method (simplified implementation)
 * 
 * This function will:
 * 1. Apply relay feedback to induce oscillations
 * 2. Measure the critical gain (Ku) and period (Tu)
 * 3. Apply Ziegler-Nichols rules to calculate PID gains
 * 
 * WARNING: This will cause the system to oscillate! Use with caution.
 * Recommended to use in a safe test environment.
 * 
 * @param controller Which PID controller to auto-tune
 * @param relay_amplitude Amplitude of relay feedback (e.g., ±20% for attitude)
 * @param timeout_ms Maximum time to wait for oscillations (milliseconds)
 * @return true if tuning successful, false if failed or timed out
 */
bool pid_auto_tune_relay(pid_controller_t controller, float relay_amplitude, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // PID_TUNING_H
