/**
 * @file mixer.h
 * @brief Motor mixer interface
 */

#pragma once

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Mix control commands into motor outputs (X-frame)
 * @param thrust Total thrust [0.0 to 1.0]
 * @param roll Roll command [-1.0 to 1.0]
 * @param pitch Pitch command [-1.0 to 1.0]
 * @param yaw Yaw command [-1.0 to 1.0]
 * @param out Output motor commands
 */
void mixer_mix(float thrust, float roll, float pitch, float yaw, motor_output_t *out);

#ifdef __cplusplus
}
#endif
