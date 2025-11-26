/**
 * @file freertos_hooks.c
 * @brief FreeRTOS hook functions
 * 
 * Optional callbacks for FreeRTOS events (idle, stack overflow, malloc fail).
 */

// #include "FreeRTOS.h"
// #include "task.h"

/**
 * @brief Idle task hook
 */
void vApplicationIdleHook(void) {
    // Can be used for low-power mode
}

/**
 * @brief Stack overflow hook
 */
void vApplicationStackOverflowHook(void *xTask, char *pcTaskName) {
    // Error: stack overflow detected
    (void)xTask;
    (void)pcTaskName;
    
    while (1) {
        // Blink LED or log error
    }
}

/**
 * @brief Malloc failed hook
 */
void vApplicationMallocFailedHook(void) {
    // Error: heap allocation failed
    while (1) {
        // Blink LED or log error
    }
}
