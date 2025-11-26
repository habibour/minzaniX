/**
 * @file fc_types.h
 * @brief Core flight controller data types
 * 
 * MCU-independent type definitions for the flight controller.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief IMU sample data
 */
typedef struct {
    float ax, ay, az;  // Accelerometer [m/s²]
    float gx, gy, gz;  // Gyroscope [rad/s]
} imu_sample_t;

/**
 * @brief Barometer sample data
 */
typedef struct {
    float pressure;     // Atmospheric pressure [Pa]
    float temperature;  // Temperature [°C]
} baro_sample_t;

/**
 * @brief Altitude estimate
 */
typedef struct {
    float altitude;      // Altitude above reference [m]
    float velocity;      // Vertical velocity [m/s]
} altitude_est_t;

/**
 * @brief Attitude estimate (Euler angles and rates)
 */
typedef struct {
    float roll, pitch, yaw;           // Angles [rad]
    float roll_rate, pitch_rate, yaw_rate;  // Angular rates [rad/s]
} attitude_t;

/**
 * @brief Attitude setpoint
 */
typedef struct {
    float roll_sp;    // Roll setpoint [rad]
    float pitch_sp;   // Pitch setpoint [rad]
    float yaw_sp;     // Yaw setpoint [rad]
    float thrust_sp;  // Thrust setpoint [0.0 to 1.0]
} attitude_setpoint_t;

/**
 * @brief Motor output commands
 */
typedef struct {
    float motor[4];  // Motor outputs [0.0 to 1.0] for motors 0-3
} motor_output_t;

/**
 * @brief PID parameters
 */
typedef struct {
    float roll_kp, roll_ki, roll_kd;
    float pitch_kp, pitch_ki, pitch_kd;
    float yaw_kp, yaw_ki, yaw_kd;
    float alt_kp, alt_ki, alt_kd;      // Altitude PID gains
    float alt_vel_kp, alt_vel_ki, alt_vel_kd;  // Altitude velocity PID gains
} pid_params_t;

/**
 * @brief Altitude setpoint
 */
typedef struct {
    float altitude_sp;   // Target altitude [m]
    float velocity_sp;   // Target vertical velocity [m/s]
} altitude_setpoint_t;
