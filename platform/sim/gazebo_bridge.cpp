/**
 * @file gazebo_bridge.cpp
 * @brief Gazebo Harmonic transport bridge implementation
 */

#include "gazebo_bridge.hpp"
#include <iostream>
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include <mutex>

namespace gazebo_bridge {

// Global state
static gz::transport::Node g_node;
static imu_sample_t g_latest_imu{};
static bool g_imu_received = false;
static std::mutex g_imu_mutex;

static baro_sample_t g_latest_baro{};
static bool g_baro_received = false;
static std::mutex g_baro_mutex;

// Publisher for motor commands (Actuators message for motor speed)
static gz::transport::Node::Publisher g_motor_pub;

/**
 * @brief IMU callback from Gazebo
 */
void imu_callback(const gz::msgs::IMU &msg) {
    std::lock_guard<std::mutex> lock(g_imu_mutex);
    
    // Convert from Gazebo IMU message to our format
    g_latest_imu.ax = msg.linear_acceleration().x();
    g_latest_imu.ay = msg.linear_acceleration().y();
    g_latest_imu.az = msg.linear_acceleration().z();
    
    g_latest_imu.gx = msg.angular_velocity().x();
    g_latest_imu.gy = msg.angular_velocity().y();
    g_latest_imu.gz = msg.angular_velocity().z();
    
    g_imu_received = true;
}

/**
 * @brief Barometer callback from Gazebo
 */
void baro_callback(const gz::msgs::FluidPressure &msg) {
    std::lock_guard<std::mutex> lock(g_baro_mutex);
    
    // Convert from Gazebo air pressure message to our format
    g_latest_baro.pressure = msg.pressure();
    g_latest_baro.temperature = 15.0f;  // Default temperature if not provided
    
    g_baro_received = true;
}

bool init() {
    std::cout << "[GAZEBO_BRIDGE] Initializing Gazebo transport..." << std::endl;
    
    // Advertise motor speed command topic
    std::string motor_topic = "/model/x500/command/motor_speed";
    
    g_motor_pub = g_node.Advertise<gz::msgs::Actuators>(motor_topic);
    if (!g_motor_pub) {
        std::cerr << "[GAZEBO_BRIDGE] Failed to advertise: " << motor_topic << std::endl;
        return false;
    }
    
    std::cout << "[GAZEBO_BRIDGE] Motor publisher ready: " << motor_topic << std::endl;
    return true;
}

bool subscribe_imu(const std::string& topic_name) {
    std::cout << "[GAZEBO_BRIDGE] Subscribing to IMU: " << topic_name << std::endl;
    
    if (!g_node.Subscribe(topic_name, imu_callback)) {
        std::cerr << "[GAZEBO_BRIDGE] Failed to subscribe to: " << topic_name << std::endl;
        return false;
    }
    
    std::cout << "[GAZEBO_BRIDGE] IMU subscription successful" << std::endl;
    return true;
}

bool publish_motors(const motor_output_t* motors) {
    if (!motors) {
        return false;
    }
    
    // Convert normalized thrust (0-1) to motor angular velocity (rad/s)
    // Motor model equation: thrust = motor_constant * omega^2
    // motor_constant = 8.54858e-06
    // For hover (3.68N per motor): omega = sqrt(3.68 / 8.54858e-06) = 656 rad/s
    // Max velocity = 1100 rad/s
    const double MAX_MOTOR_SPEED = 1100.0;  // rad/s
    
    // Create actuators message
    gz::msgs::Actuators actuators_msg;
    
    for (int i = 0; i < 4; i++) {
        // Convert normalized thrust to motor speed
        // thrust = k * omega^2  =>  omega = sqrt(thrust / k)
        // Normalize: thrust_normalized = motor[i] (0-1)
        // Max thrust ~= k * max_speed^2
        double motor_speed = motors->motor[i] * MAX_MOTOR_SPEED;
        
        // Add actuator command
        actuators_msg.add_velocity(motor_speed);
    }
    
    // Publish motor speeds
    if (!g_motor_pub.Publish(actuators_msg)) {
        return false;
    }
    
    return true;
}

bool get_imu(imu_sample_t* imu) {
    std::lock_guard<std::mutex> lock(g_imu_mutex);
    
    if (!imu || !g_imu_received) {
        return false;
    }
    
    *imu = g_latest_imu;
    return true;
}

bool subscribe_baro(const std::string& topic_name) {
    std::cout << "[GAZEBO_BRIDGE] Subscribing to barometer: " << topic_name << std::endl;
    
    if (!g_node.Subscribe(topic_name, baro_callback)) {
        std::cerr << "[GAZEBO_BRIDGE] Failed to subscribe to: " << topic_name << std::endl;
        return false;
    }
    
    std::cout << "[GAZEBO_BRIDGE] Barometer subscription successful" << std::endl;
    return true;
}

bool get_baro(baro_sample_t* baro) {
    std::lock_guard<std::mutex> lock(g_baro_mutex);
    
    if (!baro || !g_baro_received) {
        return false;
    }
    
    *baro = g_latest_baro;
    return true;
}

bool reset_drone_pose(double x, double y, double z, double roll, double pitch, double yaw) {
    // Request pose reset via Gazebo EntityPose service
    gz::msgs::Pose req;
    req.set_name("x500");
    
    // Set position
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    
    // Convert roll/pitch/yaw to quaternion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;
    
    req.mutable_orientation()->set_w(qw);
    req.mutable_orientation()->set_x(qx);
    req.mutable_orientation()->set_y(qy);
    req.mutable_orientation()->set_z(qz);
    
    gz::msgs::Boolean rep;
    bool result;
    unsigned int timeout = 1000;  // 1 second timeout
    
    bool executed = g_node.Request("/world/minzanix_world/set_pose", req, timeout, rep, result);
    
    if (!executed || !result || !rep.data()) {
        std::cerr << "[GAZEBO_BRIDGE] Failed to reset drone pose" << std::endl;
        return false;
    }
    
    std::cout << "[GAZEBO_BRIDGE] Drone pose reset successfully" << std::endl;
    return true;
}

} // namespace gazebo_bridge
