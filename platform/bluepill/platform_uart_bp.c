/**
 * @file platform_uart_bp.c
 * @brief STM32 Blue Pill UART driver
 * 
 * Uses USART1 for telemetry output.
 */

#include "../../platform_api/platform_uart.h"
#include <string.h>
// #include "stm32f1xx_hal.h"

// extern UART_HandleTypeDef huart1;

bool platform_uart_init(void) {
    // UART already initialized in main.c by CubeMX
    return true;
}

uint32_t platform_uart_write(const uint8_t *data, uint32_t len) {
    if (!data || len == 0) {
        return 0;
    }
    
    // HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 100);
    
    return len;
}

uint32_t platform_uart_read(uint8_t *data, uint32_t len) {
    if (!data || len == 0) {
        return 0;
    }
    
    // HAL_UART_Receive(&huart1, data, len, 10);
    
    return 0;
}

void platform_uart_print(const char *str) {
    if (str) {
        uint32_t len = strlen(str);
        platform_uart_write((const uint8_t*)str, len);
    }
}
