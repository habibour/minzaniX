/**
 * @file pid_tuning.c
 * @brief PID tuning utilities implementation
 */

#include "pid_tuning.h"
#include "params.h"
#include <math.h>
#include <stdio.h>

// External access to PID parameters (defined in params.c)
extern pid_params_t g_pid_params;

bool pid_update_gains(pid_controller_t controller, float kp, float ki, float kd) {
    if (kp < 0 || ki < 0 || kd < 0) {
        printf("[PID_TUNING] Error: Gains must be non-negative\n");
        return false;
    }
    
    switch (controller) {
        case PID_ROLL:
            g_pid_params.roll_kp = kp;
            g_pid_params.roll_ki = ki;
            g_pid_params.roll_kd = kd;
            printf("[PID_TUNING] Roll gains updated: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
            break;
            
        case PID_PITCH:
            g_pid_params.pitch_kp = kp;
            g_pid_params.pitch_ki = ki;
            g_pid_params.pitch_kd = kd;
            printf("[PID_TUNING] Pitch gains updated: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
            break;
            
        case PID_YAW:
            g_pid_params.yaw_kp = kp;
            g_pid_params.yaw_ki = ki;
            g_pid_params.yaw_kd = kd;
            printf("[PID_TUNING] Yaw gains updated: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
            break;
            
        case PID_ALTITUDE:
            g_pid_params.alt_kp = kp;
            g_pid_params.alt_ki = ki;
            g_pid_params.alt_kd = kd;
            printf("[PID_TUNING] Altitude gains updated: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
            break;
            
        case PID_ALTITUDE_VELOCITY:
            g_pid_params.alt_vel_kp = kp;
            g_pid_params.alt_vel_ki = ki;
            g_pid_params.alt_vel_kd = kd;
            printf("[PID_TUNING] Altitude velocity gains updated: Kp=%.4f Ki=%.4f Kd=%.4f\n", kp, ki, kd);
            break;
            
        default:
            printf("[PID_TUNING] Error: Invalid controller type\n");
            return false;
    }
    
    return true;
}

bool pid_get_gains(pid_controller_t controller, float* kp, float* ki, float* kd) {
    if (!kp || !ki || !kd) {
        return false;
    }
    
    switch (controller) {
        case PID_ROLL:
            *kp = g_pid_params.roll_kp;
            *ki = g_pid_params.roll_ki;
            *kd = g_pid_params.roll_kd;
            break;
            
        case PID_PITCH:
            *kp = g_pid_params.pitch_kp;
            *ki = g_pid_params.pitch_ki;
            *kd = g_pid_params.pitch_kd;
            break;
            
        case PID_YAW:
            *kp = g_pid_params.yaw_kp;
            *ki = g_pid_params.yaw_ki;
            *kd = g_pid_params.yaw_kd;
            break;
            
        case PID_ALTITUDE:
            *kp = g_pid_params.alt_kp;
            *ki = g_pid_params.alt_ki;
            *kd = g_pid_params.alt_kd;
            break;
            
        case PID_ALTITUDE_VELOCITY:
            *kp = g_pid_params.alt_vel_kp;
            *ki = g_pid_params.alt_vel_ki;
            *kd = g_pid_params.alt_vel_kd;
            break;
            
        default:
            return false;
    }
    
    return true;
}

bool pid_tune_ziegler_nichols(pid_controller_t controller, const ziegler_nichols_params_t* params) {
    if (!params) {
        printf("[PID_TUNING] Error: NULL parameters\n");
        return false;
    }
    
    if (params->ku <= 0 || params->tu <= 0) {
        printf("[PID_TUNING] Error: Ku and Tu must be positive\n");
        return false;
    }
    
    float kp, ki, kd;
    
    if (params->use_classic) {
        // Classic Ziegler-Nichols PID tuning rules
        kp = 0.6f * params->ku;
        ki = 1.2f * params->ku / params->tu;
        kd = 0.075f * params->ku * params->tu;
        
        printf("[PID_TUNING] Using classic Ziegler-Nichols method\n");
    } else {
        // "Some overshoot" variant - more conservative
        kp = 0.33f * params->ku;
        ki = 0.66f * params->ku / params->tu;
        kd = 0.11f * params->ku * params->tu;
        
        printf("[PID_TUNING] Using conservative Ziegler-Nichols method (some overshoot)\n");
    }
    
    printf("[PID_TUNING] Ziegler-Nichols tuning:\n");
    printf("  Ku (ultimate gain) = %.4f\n", params->ku);
    printf("  Tu (ultimate period) = %.4f s\n", params->tu);
    printf("  Calculated Kp = %.4f\n", kp);
    printf("  Calculated Ki = %.4f\n", ki);
    printf("  Calculated Kd = %.4f\n", kd);
    
    return pid_update_gains(controller, kp, ki, kd);
}

bool pid_auto_tune_relay(pid_controller_t controller, float relay_amplitude, uint32_t timeout_ms) {
    // This is a simplified stub for relay auto-tuning
    // Full implementation would require:
    // 1. Control loop integration
    // 2. Real-time oscillation detection
    // 3. Peak detection and period measurement
    // 4. FFT or zero-crossing analysis
    
    printf("[PID_TUNING] Auto-tuning with relay feedback method\n");
    printf("[PID_TUNING] WARNING: This feature is not fully implemented\n");
    printf("[PID_TUNING] Please use manual Ziegler-Nichols tuning with measured Ku and Tu\n");
    
    // To implement this properly, you would:
    // 1. Save current gains
    // 2. Set controller to relay mode (on/off control)
    // 3. Monitor system response for sustained oscillations
    // 4. Measure oscillation amplitude and period
    // 5. Calculate Ku = (4 * relay_amplitude) / (PI * oscillation_amplitude)
    // 6. Calculate Tu = oscillation_period
    // 7. Apply Ziegler-Nichols tuning rules
    // 8. Restore normal control mode
    
    return false;
}
