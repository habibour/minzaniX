/**
 * @file gazebo_bridge.hpp
 * @brief Optional Gazebo Harmonic transport bridge
 * 
 * This file provides interfaces for connecting to Gazebo Harmonic
 * using gz-transport for IMU and motor communication.
 */

#pragma once

#include <string>

extern "C" {
    #include "core/fc_types.h"
}

namespace gazebo_bridge {

/**
 * @brief Initialize Gazebo transport
 * @return true if successful
 */
bool init();

/**
 * @brief Subscribe to IMU topic
 * @param topic_name Gazebo IMU topic (e.g., "/mini_quad/imu")
 * @return true if successful
 */
bool subscribe_imu(const std::string& topic_name);

/**
 * @brief Publish motor commands
 * @param motors Motor output commands
 * @return true if successful
 */
bool publish_motors(const motor_output_t* motors);

/**
 * @brief Get latest IMU data
 * @param imu Output IMU sample
 * @return true if data available
 */
bool get_imu(imu_sample_t* imu);

/**
 * @brief Subscribe to barometer topic
 * @param topic_name Gazebo air pressure topic
 * @return true if successful
 */
bool subscribe_baro(const std::string& topic_name);

/**
 * @brief Get latest barometer data
 * @param baro Output barometer sample
 * @return true if data available
 */
bool get_baro(baro_sample_t* baro);

/**
 * @brief Reset drone to initial position
 * @param x X position
 * @param y Y position  
 * @param z Z position
 * @param roll Roll angle (radians)
 * @param pitch Pitch angle (radians)
 * @param yaw Yaw angle (radians)
 * @return true if successful
 */
bool reset_drone_pose(double x, double y, double z, double roll, double pitch, double yaw);

} // namespace gazebo_bridge
