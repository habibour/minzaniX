/**
 * @file tasks.c
 * @brief FreeRTOS tasks for flight controller
 * 
 * Main control loop running at 500Hz.
 */

#include "../../core/fc_interface.h"
#include "../../platform_api/platform_imu.h"
#include "../../platform_api/platform_pwm.h"
#include "../../platform_api/platform_time.h"
#include "../../platform_api/platform_uart.h"

// FreeRTOS
// #include "cmsis_os.h"

/**
 * @brief Main control task (500Hz)
 */
void StartControlTask(void const * argument)
{
    motor_output_t motors;
    uint32_t prev_time = platform_millis();
    
    // Initialize flight controller
    fc_init();
    
    // Set default setpoint
    attitude_setpoint_t sp = {
        .roll_sp = 0.0f,
        .pitch_sp = 0.0f,
        .yaw_sp = 0.0f,
        .thrust_sp = 0.0f
    };
    fc_set_attitude_setpoint(&sp);
    
    platform_uart_print("FC Task Started\r\n");
    
    // Main loop
    while (1)
    {
        imu_sample_t imu;
        
        // Read IMU
        if (platform_imu_read(&imu)) {
            // Calculate dt
            uint32_t current_time = platform_millis();
            float dt = (current_time - prev_time) / 1000.0f;
            prev_time = current_time;
            
            // Run flight controller
            fc_step(&imu, dt, &motors);
            
            // Write motor outputs
            platform_pwm_write(&motors);
        }
        
        // Wait for next cycle (2ms = 500Hz)
        // osDelay(2);
        platform_sleep_ms(2);
    }
}
