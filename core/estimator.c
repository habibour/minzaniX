/**
 * @file estimator.c
 * @brief Attitude estimator implementation
 * 
 * Simple complementary filter for roll, pitch, yaw estimation.
 */

#include "estimator.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#define ALPHA 0.98f  // Complementary filter coefficient

static attitude_t g_att = {0};

// Calibration state
static bool g_calibrated = false;
static bool g_calibrating = false;
static uint32_t g_calib_samples = 0;
static uint32_t g_calib_target_samples = 0;

// Accumulated values for averaging
static float g_calib_acc_ax = 0.0f;
static float g_calib_acc_ay = 0.0f;
static float g_calib_acc_az = 0.0f;
static float g_calib_acc_gx = 0.0f;
static float g_calib_acc_gy = 0.0f;
static float g_calib_acc_gz = 0.0f;

// Calibration offsets (bias)
static float g_offset_ax = 0.0f;
static float g_offset_ay = 0.0f;
static float g_offset_az = 0.0f;
static float g_offset_gx = 0.0f;
static float g_offset_gy = 0.0f;
static float g_offset_gz = 0.0f;

void estimator_reset(void) {
    g_att.roll = 0.0f;
    g_att.pitch = 0.0f;
    g_att.yaw = 0.0f;
    g_att.roll_rate = 0.0f;
    g_att.pitch_rate = 0.0f;
    g_att.yaw_rate = 0.0f;
}

void estimator_start_calibration(uint32_t duration_ms) {
    // Assume 100Hz update rate -> 100 samples per second
    g_calib_target_samples = (duration_ms * 100) / 1000;
    if (g_calib_target_samples < 10) g_calib_target_samples = 10;  // Minimum 10 samples
    
    g_calibrating = true;
    g_calibrated = false;
    g_calib_samples = 0;
    
    // Reset accumulators
    g_calib_acc_ax = 0.0f;
    g_calib_acc_ay = 0.0f;
    g_calib_acc_az = 0.0f;
    g_calib_acc_gx = 0.0f;
    g_calib_acc_gy = 0.0f;
    g_calib_acc_gz = 0.0f;
    
    printf("[ESTIMATOR] Starting calibration: collecting %u samples...\n", g_calib_target_samples);
}

bool estimator_calibrate_sample(const imu_sample_t *imu) {
    if (!g_calibrating) {
        return g_calibrated;  // Already done or not started
    }
    
    // Accumulate samples
    g_calib_acc_ax += imu->ax;
    g_calib_acc_ay += imu->ay;
    g_calib_acc_az += imu->az;
    g_calib_acc_gx += imu->gx;
    g_calib_acc_gy += imu->gy;
    g_calib_acc_gz += imu->gz;
    g_calib_samples++;
    
    // Check if done
    if (g_calib_samples >= g_calib_target_samples) {
        // Calculate averages (negative offset means sensor reads negative, so add to correct)
        float n = (float)g_calib_samples;
        float avg_ax = g_calib_acc_ax / n;
        float avg_ay = g_calib_acc_ay / n;
        float avg_az = g_calib_acc_az / n;
        float avg_gx = g_calib_acc_gx / n;
        float avg_gy = g_calib_acc_gy / n;
        float avg_gz = g_calib_acc_gz / n;
        
        // Offset is negative of average (if sensor reads +0.1, offset is -0.1 to subtract it)
        g_offset_ax = -avg_ax;
        g_offset_ay = -avg_ay;
        g_offset_az = -(avg_az - 9.81f);  // Z should be 9.81 when level
        g_offset_gx = -avg_gx;
        g_offset_gy = -avg_gy;
        g_offset_gz = -avg_gz;
        
        g_calibrating = false;
        g_calibrated = true;
        
        printf("[ESTIMATOR] Calibration complete!\n");
        printf("  Accel offsets: ax=%.4f, ay=%.4f, az=%.4f\n", g_offset_ax, g_offset_ay, g_offset_az);
        printf("  Gyro offsets:  gx=%.4f, gy=%.4f, gz=%.4f\n", g_offset_gx, g_offset_gy, g_offset_gz);
        
        return true;
    }
    
    return false;
}

bool estimator_is_calibrated(void) {
    return g_calibrated;
}

void estimator_update(const imu_sample_t *imu, float dt) {
    // Apply calibration offsets (add because offset is already negated)
    float ax = imu->ax + g_offset_ax;
    float ay = imu->ay + g_offset_ay;
    float az = imu->az + g_offset_az;
    float gx = imu->gx + g_offset_gx;
    float gy = imu->gy + g_offset_gy;
    float gz = imu->gz + g_offset_gz;
    
    // Integrate gyroscope (high-pass)
    float roll_gyro = g_att.roll + gx * dt;
    float pitch_gyro = g_att.pitch + gy * dt;
    float yaw_gyro = g_att.yaw + gz * dt;
    
    // Calculate angles from accelerometer (low-pass)
    float roll_accel = atan2f(ay, az);
    float pitch_accel = atan2f(-ax, sqrtf(ay * ay + az * az));
    
    // Complementary filter
    g_att.roll = ALPHA * roll_gyro + (1.0f - ALPHA) * roll_accel;
    g_att.pitch = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;
    g_att.yaw = yaw_gyro;  // No accel correction for yaw
    
    // Update rates (use calibrated values)
    g_att.roll_rate = gx;
    g_att.pitch_rate = gy;
    g_att.yaw_rate = gz;
}

void estimator_get_attitude(attitude_t *att) {
    if (att) {
        *att = g_att;
    }
}
