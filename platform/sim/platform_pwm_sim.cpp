/**
 * @file platform_pwm_sim.cpp
 * @brief Simulated PWM output implementation
 * 
 * Prints motor commands to console or sends to Gazebo.
 */

#include <cstdio>

extern "C" {
    #include "platform_api/platform_pwm.h"
}

extern "C" bool platform_pwm_init() {
    printf("[SIM_PWM] Initialized\n");
    return true;
}

extern "C" bool platform_pwm_write(const motor_output_t *motors) {
    if (!motors) {
        return false;
    }
    
    // Print motor outputs (can be replaced with Gazebo transport)
    // Uncomment for detailed motor output logging
    // printf("Motors: M0=%.3f M1=%.3f M2=%.3f M3=%.3f\n",
    //        motors->motor[0], motors->motor[1], 
    //        motors->motor[2], motors->motor[3]);
    
    return true;
}
