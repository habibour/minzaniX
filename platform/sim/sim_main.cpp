/**
 * @file sim_main.cpp
 * @brief Simulation main loop (runs FC on PC)
 * 
 * This is the entry point for running the flight controller in simulation.
 * It reads IMU data from Gazebo (or fake data) and writes motor commands.
 */

#include <iostream>
#include <cstdio>
#include <cstring>

extern "C" {
    #include "core/fc_interface.h"
    #include "core/pid_tuning.h"
    #include "core/estimator.h"
    #include "core/altitude_estimator.h"
    #include "platform_api/platform_imu.h"
    #include "platform_api/platform_pwm.h"
    #include "platform_api/platform_time.h"
    #include "platform_api/platform_uart.h"
}

#include "gazebo_bridge.hpp"

// Use Gazebo if USE_GAZEBO is defined
#ifdef USE_GAZEBO
static bool use_gazebo = true;
#else
static bool use_gazebo = false;
#endif

int main(int argc, char** argv) {
    // Check for --gazebo flag
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--gazebo") == 0) {
            use_gazebo = true;
        }
    }
    
    std::cout << "=== minzaniX 1.0 Flight Controller (Simulation) ===" << std::endl;
    
    if (use_gazebo) {
        std::cout << "Mode: Gazebo Harmonic Integration" << std::endl;
    } else {
        std::cout << "Mode: Standalone (fake IMU data)" << std::endl;
    }
    
    // Initialize Gazebo bridge if requested
    if (use_gazebo) {
        if (!gazebo_bridge::init()) {
            std::cerr << "ERROR: Failed to initialize Gazebo bridge" << std::endl;
            return 1;
        }
        
        // Subscribe to PX4 x500 sensors (world name: minzanix_world)
        if (!gazebo_bridge::subscribe_imu("/world/minzanix_world/model/x500/link/base_link/sensor/imu_sensor/imu")) {
            std::cerr << "ERROR: Failed to subscribe to IMU topic" << std::endl;
            return 1;
        }
        
        if (!gazebo_bridge::subscribe_baro("/world/minzanix_world/model/x500/link/base_link/sensor/air_pressure_sensor/air_pressure")) {
            std::cerr << "ERROR: Failed to subscribe to barometer topic" << std::endl;
            return 1;
        }
        
        std::cout << "Waiting for sensor data from Gazebo..." << std::endl;
        // Wait for first IMU message
        imu_sample_t test_imu;
        int wait_count = 0;
        while (!gazebo_bridge::get_imu(&test_imu) && wait_count < 50) {
            platform_sleep_ms(100);
            wait_count++;
        }
        
        if (wait_count >= 50) {
            std::cerr << "ERROR: No IMU data received from Gazebo" << std::endl;
            std::cerr << "Make sure Gazebo is running: gz sim sim_world/world.sdf" << std::endl;
            return 1;
        }
        
        std::cout << "IMU data received from Gazebo!" << std::endl;
    }
    
    // Initialize platform
    if (!platform_imu_init()) {
        std::cerr << "ERROR: Failed to initialize IMU" << std::endl;
        return 1;
    }
    
    if (!platform_pwm_init()) {
        std::cerr << "ERROR: Failed to initialize PWM" << std::endl;
        return 1;
    }
    
    platform_uart_init();
    
    // Initialize flight controller
    fc_init();
    std::cout << "Flight controller initialized" << std::endl;
    
    // ========================================
    // CALIBRATION PHASE (First 3 seconds)
    // ========================================
    std::cout << "\n=== SENSOR CALIBRATION ===" << std::endl;
    std::cout << "Collecting sensor data for 3 seconds..." << std::endl;
    std::cout << "Keep drone stationary!" << std::endl;
    
    const uint32_t CALIBRATION_DURATION_MS = 3000;
    estimator_start_calibration(CALIBRATION_DURATION_MS);
    if (use_gazebo) {
        altitude_estimator_start_calibration(CALIBRATION_DURATION_MS);
    }
    
    uint32_t calib_start = platform_millis();
    bool imu_calibrated = false;
    bool baro_calibrated = !use_gazebo;  // Skip baro if not using gazebo
    
    while (!imu_calibrated || !baro_calibrated) {
        // Read sensors
        imu_sample_t imu;
        if (use_gazebo) {
            if (!gazebo_bridge::get_imu(&imu)) {
                platform_sleep_ms(4);
                continue;
            }
        } else {
            if (!platform_imu_read(&imu)) {
                platform_sleep_ms(4);
                continue;
            }
        }
        
        // Calibrate IMU
        if (!imu_calibrated) {
            imu_calibrated = estimator_calibrate_sample(&imu);
        }
        
        // Calibrate barometer
        if (use_gazebo && !baro_calibrated) {
            baro_sample_t baro;
            if (gazebo_bridge::get_baro(&baro)) {
                baro_calibrated = altitude_estimator_calibrate_sample(&baro);
            }
        }
        
        // Progress indicator
        uint32_t elapsed = platform_millis() - calib_start;
        if (elapsed % 500 == 0) {
            std::cout << "Calibrating... " << (elapsed / 1000.0f) << "s" << std::endl;
        }
        
        platform_sleep_ms(4);
    }
    
    std::cout << "=== CALIBRATION COMPLETE ===" << std::endl;
    std::cout << "Sensors ready for flight!\n" << std::endl;
    
    // PID gains are manually tuned in params.c for simulation stability
    // Uncomment below to use Ziegler-Nichols auto-tuning instead:
    /*
    std::cout << "=== PID TUNING ===" << std::endl;
    std::cout << "Applying conservative Ziegler-Nichols tuning..." << std::endl;
    
    // Tune altitude controller with conservative ZN method
    ziegler_nichols_params_t zn_alt;
    zn_alt.ku = 0.8f;  // Estimated ultimate gain for altitude
    zn_alt.tu = 2.0f;  // Estimated oscillation period
    zn_alt.use_classic = false;  // Use conservative for stability
    pid_tune_ziegler_nichols(PID_ALTITUDE, &zn_alt);
    
    // Tune altitude velocity controller
    ziegler_nichols_params_t zn_vel;
    zn_vel.ku = 1.0f;
    zn_vel.tu = 1.5f;
    zn_vel.use_classic = false;
    pid_tune_ziegler_nichols(PID_ALTITUDE_VELOCITY, &zn_vel);
    
    std::cout << "PID tuning complete!\n" << std::endl;
    */
    std::cout << "Using manually tuned PID gains for stable flight\n" << std::endl;
    
    // Set attitude setpoint (hover with level attitude)
    attitude_setpoint_t sp{};
    sp.roll_sp = 0.0f;
    sp.pitch_sp = 0.0f;
    sp.yaw_sp = 0.0f;
    sp.thrust_sp = 0.80f;  // 80% throttle - x500 is heavy
    fc_set_attitude_setpoint(&sp);
    
    // Enable altitude hold
    if (use_gazebo) {
        altitude_setpoint_t alt_sp{};
        alt_sp.altitude_sp = 2.0f;  // Target altitude: 2.0 meters (reduced from 12m)
        alt_sp.velocity_sp = 0.0f;  // Hover
        fc_set_altitude_setpoint(&alt_sp);
        fc_set_altitude_hold_enabled(true);
        std::cout << "Altitude hold ENABLED - Target: 2.0m" << std::endl;
    } else {
        std::cout << "Using manual thrust - testing lift capability" << std::endl;
    }
    
    // Arm the system
    fc_set_armed(true);
    std::cout << "System ARMED" << std::endl;
    
    // Main loop
    uint32_t last_time = platform_millis();
    uint32_t start_time = platform_millis();
    uint32_t frame_count = 0;
    const uint32_t RESET_INTERVAL = 20000;  // 20 seconds in milliseconds
    
    std::cout << "Starting main loop..." << std::endl;
    std::cout << "Auto-reset enabled: Drone will reset to initial position every 20 seconds" << std::endl;
    
    while (true) {
        // Check if 20 seconds elapsed - reset drone position
        uint32_t elapsed = platform_millis() - start_time;
        if (elapsed >= RESET_INTERVAL) {
            std::cout << "\n=== 20 SECOND RESET ===" << std::endl;
            std::cout << "Resetting drone to initial position (0, 0, 0.5)" << std::endl;
            
            // Reset via Gazebo service
            if (use_gazebo) {
                gazebo_bridge::reset_drone_pose(0, 0, 0.5, 0, 0, 0);
            }
            
            start_time = platform_millis();
            std::cout << "Reset complete. Resuming flight...\n" << std::endl;
        }
        
        // Read IMU (from Gazebo or fake data)
        imu_sample_t imu;
        if (use_gazebo) {
            if (!gazebo_bridge::get_imu(&imu)) {
                std::cerr << "WARNING: No IMU data from Gazebo" << std::endl;
                platform_sleep_ms(4);
                continue;
            }
        } else {
            if (!platform_imu_read(&imu)) {
                std::cerr << "WARNING: Failed to read IMU" << std::endl;
            }
        }
        
        // Read barometer if using Gazebo
        baro_sample_t baro;
        baro_sample_t* baro_ptr = nullptr;
        if (use_gazebo && gazebo_bridge::get_baro(&baro)) {
            baro_ptr = &baro;
            // Debug: print first barometer reading
            if (frame_count == 1) {
                std::cout << "Barometer data received! Pressure: " << baro.pressure << " Pa" << std::endl;
            }
        }
        
        // Calculate dt
        uint32_t current_time = platform_millis();
        float dt = (current_time - last_time) / 1000.0f;
        last_time = current_time;
        
        // Run flight controller
        motor_output_t motors;
        fc_step(&imu, baro_ptr, dt, &motors);
        
        // Write motor outputs (to Gazebo or console)
        if (use_gazebo) {
            gazebo_bridge::publish_motors(&motors);
        } else {
            platform_pwm_write(&motors);
        }
        
        // Print status every 250ms
        frame_count++;
        if (frame_count % 50 == 0) {
            attitude_t att;
            fc_get_attitude(&att);
            
            if (use_gazebo && baro_ptr) {
                altitude_est_t alt;
                fc_get_altitude(&alt);
                printf("Alt: %.2fm Vel:%.2fm/s | Att: R=%.2f P=%.2f Y=%.2f | Motors: %.2f %.2f %.2f %.2f\n",
                       alt.altitude, alt.velocity,
                       att.roll * 57.3f, att.pitch * 57.3f, att.yaw * 57.3f,
                       motors.motor[0], motors.motor[1], 
                       motors.motor[2], motors.motor[3]);
            } else {
                printf("Att: R=%.2f P=%.2f Y=%.2f | Motors: %.2f %.2f %.2f %.2f\n",
                       att.roll * 57.3f, att.pitch * 57.3f, att.yaw * 57.3f,
                       motors.motor[0], motors.motor[1], 
                       motors.motor[2], motors.motor[3]);
            }
        }
        
        // Sleep to maintain ~250Hz loop rate
        platform_sleep_ms(4);
    }
    
    return 0;
}
