/**
 * @file altitude_estimator.c
 * @brief Altitude estimation using barometer and accelerometer fusion
 * 
 * Uses complementary filter to fuse:
 * - Barometric altitude (low-pass, slow but accurate long-term)
 * - Accelerometer vertical velocity integration (high-pass, fast but drifts)
 */

#include "altitude_estimator.h"
#include <math.h>
#include <stdio.h>

// Barometric formula constants
#define SEA_LEVEL_PRESSURE 101325.0f  // Pa
#define TEMPERATURE_LAPSE_RATE 0.0065f // K/m
#define GAS_CONSTANT 8.31447f
#define GRAVITATIONAL_ACCEL 9.80665f
#define MOLAR_MASS_AIR 0.0289644f

// Complementary filter weight for barometer (higher = smoother altitude)
#define BARO_WEIGHT 0.995f  // 99.5% barometer for smoother readings
#define ACCEL_WEIGHT (1.0f - BARO_WEIGHT)

// State variables
static float s_reference_pressure = SEA_LEVEL_PRESSURE;
static float s_estimated_altitude = 0.0f;
static float s_estimated_velocity = 0.0f;
static float s_prev_baro_alt = 0.0f;
static float s_velocity_filtered = 0.0f;  // Low-pass filtered velocity
static bool s_initialized = false;

// Calibration state
static bool s_calibrated = false;
static bool s_calibrating = false;
static uint32_t s_calib_samples = 0;
static uint32_t s_calib_target_samples = 0;
static float s_calib_acc_pressure = 0.0f;
static float s_pressure_offset = 0.0f;

/**
 * @brief Convert pressure to altitude using barometric formula
 */
static float pressure_to_altitude(float pressure, float ref_pressure) {
    // Simplified barometric formula
    // h = (T0/L) * (1 - (P/P0)^(R*L/(g*M)))
    // For simplicity, use: h ≈ 44330 * (1 - (P/P0)^0.19)
    float pressure_ratio = pressure / ref_pressure;
    return 44330.0f * (1.0f - powf(pressure_ratio, 0.1903f));
}

void altitude_estimator_init(float initial_pressure) {
    s_reference_pressure = initial_pressure;
    s_estimated_altitude = 0.0f;
    s_estimated_velocity = 0.0f;
    s_prev_baro_alt = 0.0f;
    s_initialized = true;
}

void altitude_estimator_start_calibration(uint32_t duration_ms) {
    // Assume 100Hz update rate
    s_calib_target_samples = (duration_ms * 100) / 1000;
    if (s_calib_target_samples < 10) s_calib_target_samples = 10;
    
    s_calibrating = true;
    s_calibrated = false;
    s_calib_samples = 0;
    s_calib_acc_pressure = 0.0f;
    
    printf("[ALTITUDE] Starting barometer calibration: collecting %u samples...\n", s_calib_target_samples);
}

bool altitude_estimator_calibrate_sample(const baro_sample_t *baro) {
    if (!s_calibrating) {
        return s_calibrated;
    }
    
    s_calib_acc_pressure += baro->pressure;
    s_calib_samples++;
    
    if (s_calib_samples >= s_calib_target_samples) {
        float avg_pressure = s_calib_acc_pressure / (float)s_calib_samples;
        // Offset is negative of difference (if pressure reads high, offset is negative to subtract)
        s_pressure_offset = -(avg_pressure - SEA_LEVEL_PRESSURE);
        s_reference_pressure = avg_pressure;
        
        s_calibrating = false;
        s_calibrated = true;
        
        printf("[ALTITUDE] Barometer calibration complete!\n");
        printf("  Average pressure: %.2f Pa (offset: %.2f Pa)\n", avg_pressure, s_pressure_offset);
        
        return true;
    }
    
    return false;
}

bool altitude_estimator_is_calibrated(void) {
    return s_calibrated;
}

void altitude_estimator_update(
    const baro_sample_t *baro,
    const imu_sample_t *imu,
    const attitude_t *att,
    float dt,
    altitude_est_t *alt_out
) {
    if (!s_initialized) {
        altitude_estimator_init(baro->pressure);
    }

    // 1. Apply calibration offset and get barometric altitude
    float corrected_pressure = baro->pressure + s_pressure_offset;
    float baro_alt = pressure_to_altitude(corrected_pressure, s_reference_pressure);
    
    // 2. Compute barometric vertical velocity (derivative with clamping)
    float baro_vel = 0.0f;
    if (dt > 0.001f) {  // Avoid division by zero
        baro_vel = (baro_alt - s_prev_baro_alt) / dt;
        // Clamp velocity to physically reasonable values
        if (baro_vel > 10.0f) baro_vel = 10.0f;
        if (baro_vel < -10.0f) baro_vel = -10.0f;
    }
    s_prev_baro_alt = baro_alt;
    
    // Low-pass filter velocity for smoother output (alpha=0.3 for smoothing)
    const float velocity_alpha = 0.3f;
    s_velocity_filtered = velocity_alpha * baro_vel + (1.0f - velocity_alpha) * s_velocity_filtered;
    
    // 3. Get vertical acceleration from IMU (compensate for gravity and attitude)
    // Transform body-frame Z acceleration to world frame
    float cos_roll = cosf(att->roll);
    float cos_pitch = cosf(att->pitch);
    float az_world = imu->az * cos_roll * cos_pitch;
    
    // Remove gravity (az_world should be ~9.81 when stationary)
    float vertical_accel = az_world - GRAVITATIONAL_ACCEL;
    
    // 4. Integrate accelerometer to get velocity
    float accel_velocity = s_estimated_velocity + vertical_accel * dt;
    
    // 5. Complementary filter: Fuse baro and accel
    // High-pass filter on accelerometer (fast response, but drifts)
    // Low-pass filter on barometer (slow but stable)
    s_estimated_velocity = BARO_WEIGHT * s_velocity_filtered + ACCEL_WEIGHT * accel_velocity;
    s_estimated_altitude = BARO_WEIGHT * baro_alt + ACCEL_WEIGHT * (s_estimated_altitude + s_estimated_velocity * dt);
    
    // 6. Output
    alt_out->altitude = s_estimated_altitude;
    alt_out->velocity = s_estimated_velocity;
}

void altitude_estimator_reset_reference(const baro_sample_t *baro) {
    s_reference_pressure = baro->pressure;
    s_estimated_altitude = 0.0f;
    s_estimated_velocity = 0.0f;
    s_prev_baro_alt = 0.0f;
}
