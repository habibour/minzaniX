/**
 * @file platform_pwm_bp.c
 * @brief STM32 Blue Pill PWM output driver
 * 
 * Uses TIM1 channels 1-4 for motor PWM outputs.
 */

#include "../../platform_api/platform_pwm.h"
// #include "stm32f1xx_hal.h"

// extern TIM_HandleTypeDef htim1;

#define PWM_MIN 1000  // 1ms pulse
#define PWM_MAX 2000  // 2ms pulse

bool platform_pwm_init(void) {
    // Start PWM on all 4 channels
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    return true;
}

bool platform_pwm_write(const motor_output_t *motors) {
    if (!motors) {
        return false;
    }
    
    // Convert [0.0, 1.0] to PWM pulse width [1000, 2000] μs
    for (int i = 0; i < 4; i++) {
        uint16_t pulse = PWM_MIN + (uint16_t)(motors->motor[i] * (PWM_MAX - PWM_MIN));
        
        // Write to timer compare register
        // switch(i) {
        //     case 0: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse); break;
        //     case 1: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse); break;
        //     case 2: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse); break;
        //     case 3: __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse); break;
        // }
    }
    
    return true;
}
