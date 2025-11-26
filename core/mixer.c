/**
 * @file mixer.c
 * @brief Motor mixer implementation (X-configuration quadcopter)
 * 
 * Motor layout:
 *      FRONT
 *   M0     M1
 *     \   /
 *      \ /
 *      X
 *     / \
 *    /   \
 *   M3     M2
 *      REAR
 * 
 * M0: Front-Left  (CW)
 * M1: Front-Right (CCW)
 * M2: Rear-Right  (CW)
 * M3: Rear-Left   (CCW)
 */

#include "mixer.h"

#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

void mixer_mix(float thrust, float roll, float pitch, float yaw, motor_output_t *out) {
    // X-frame mixing matrix
    // Motor = Thrust + Roll * roll_coeff + Pitch * pitch_coeff + Yaw * yaw_coeff
    
    out->motor[0] = thrust + roll + pitch + yaw;  // Front-Left
    out->motor[1] = thrust - roll + pitch - yaw;  // Front-Right
    out->motor[2] = thrust - roll - pitch + yaw;  // Rear-Right
    out->motor[3] = thrust + roll - pitch - yaw;  // Rear-Left
    
    // Clamp outputs to valid range [0.0, 1.0]
    for (int i = 0; i < 4; i++) {
        out->motor[i] = CLAMP(out->motor[i], 0.0f, 1.0f);
    }
}
