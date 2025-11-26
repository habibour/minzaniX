/**
 * @file platform_uart_sim.cpp
 * @brief Simulated UART implementation
 * 
 * Prints to console (stdout).
 */

#include <cstdio>
#include <cstring>

extern "C" {
    #include "platform_api/platform_uart.h"
}

extern "C" bool platform_uart_init() {
    printf("[SIM_UART] Initialized\n");
    return true;
}

extern "C" uint32_t platform_uart_write(const uint8_t *data, uint32_t len) {
    if (!data) {
        return 0;
    }
    
    for (uint32_t i = 0; i < len; i++) {
        putchar(data[i]);
    }
    fflush(stdout);
    
    return len;
}

extern "C" uint32_t platform_uart_read(uint8_t *data, uint32_t len) {
    // Not implemented for simulation
    return 0;
}

extern "C" void platform_uart_print(const char *str) {
    if (str) {
        printf("%s", str);
        fflush(stdout);
    }
}
