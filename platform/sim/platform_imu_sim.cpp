/**
 * @file platform_imu_sim.cpp
 * @brief Simulated IMU implementation
 * 
 * Provides fake IMU data or reads from Gazebo transport.
 */

#include <cstdio>
#include <cmath>

extern "C" {
    #include "platform_api/platform_imu.h"
}

// Simulated IMU state
static float sim_roll = 0.0f;
static float sim_pitch = 0.0f;
static float sim_yaw = 0.0f;

extern "C" bool platform_imu_init() {
    printf("[SIM_IMU] Initialized\n");
    return true;
}

extern "C" bool platform_imu_read(imu_sample_t *out) {
    if (!out) {
        return false;
    }
    
    // Simulate gravity vector (perfect level)
    out->ax = 0.0f;
    out->ay = 0.0f;
    out->az = 9.81f;
    
    // Simulate small drift in yaw gyro
    out->gx = 0.0f;
    out->gy = 0.0f;
    out->gz = 0.03f;  // 0.03 rad/s yaw drift
    
    return true;
}
