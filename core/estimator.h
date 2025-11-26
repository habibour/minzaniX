/**
 * @file estimator.h
 * @brief Attitude estimator interface
 */

#pragma once

#include "fc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize/reset estimator
 */
void estimator_reset(void);

/**
 * @brief Start sensor calibration (collect samples for averaging)
 * @param duration_ms Duration to collect samples [milliseconds]
 */
void estimator_start_calibration(uint32_t duration_ms);

/**
 * @brief Add IMU sample to calibration (called during startup)
 * @param imu Raw IMU data
 * @return true if calibration complete, false if still collecting
 */
bool estimator_calibrate_sample(const imu_sample_t *imu);

/**
 * @brief Check if calibration is complete
 * @return true if calibrated and ready for flight
 */
bool estimator_is_calibrated(void);

/**
 * @brief Update estimator with new IMU sample
 * @param imu IMU data
 * @param dt Time step [seconds]
 */
void estimator_update(const imu_sample_t *imu, float dt);

/**
 * @brief Get current attitude estimate
 * @param att Output attitude
 */
void estimator_get_attitude(attitude_t *att);

#ifdef __cplusplus
}
#endif
