/**
 * @file altitude_estimator.h
 * @brief Altitude estimation using barometer and accelerometer fusion
 */

#pragma once

#include "fc_types.h"
#include <stdbool.h>

/**
 * @brief Initialize altitude estimator
 * @param initial_pressure Reference pressure at ground level [Pa]
 */
void altitude_estimator_init(float initial_pressure);

/**
 * @brief Start barometer calibration
 * @param duration_ms Duration to collect samples [milliseconds]
 */
void altitude_estimator_start_calibration(uint32_t duration_ms);

/**
 * @brief Add barometer sample to calibration
 * @param baro Raw barometer data
 * @return true if calibration complete, false if still collecting
 */
bool altitude_estimator_calibrate_sample(const baro_sample_t *baro);

/**
 * @brief Check if altitude estimator is calibrated
 */
bool altitude_estimator_is_calibrated(void);

/**
 * @brief Update altitude estimate with new sensor data
 * @param baro Barometer sample
 * @param imu IMU sample (for vertical acceleration)
 * @param dt Time since last update [s]
 * @param alt_out Output altitude estimate
 */
void altitude_estimator_update(
    const baro_sample_t *baro,
    const imu_sample_t *imu,
    const attitude_t *att,
    float dt,
    altitude_est_t *alt_out
);

/**
 * @brief Reset altitude reference to current position
 * @param baro Current barometer reading
 */
void altitude_estimator_reset_reference(const baro_sample_t *baro);
