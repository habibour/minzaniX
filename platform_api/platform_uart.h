/**
 * @file platform_uart.h
 * @brief Platform-independent UART interface
 * 
 * Abstract interface for serial communication (telemetry, logging).
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize UART
 * @return true if successful
 */
bool platform_uart_init(void);

/**
 * @brief Write data to UART
 * @param data Data buffer
 * @param len Data length
 * @return Number of bytes written
 */
uint32_t platform_uart_write(const uint8_t *data, uint32_t len);

/**
 * @brief Read data from UART
 * @param data Data buffer
 * @param len Maximum bytes to read
 * @return Number of bytes read
 */
uint32_t platform_uart_read(uint8_t *data, uint32_t len);

/**
 * @brief Print string to UART (helper)
 * @param str Null-terminated string
 */
void platform_uart_print(const char *str);

#ifdef __cplusplus
}
#endif
