/**
 * @file safety.c
 * @brief Safety and arming logic implementation
 */

#include "safety.h"

static bool g_armed = false;

void safety_set_armed(bool armed) {
    g_armed = armed;
}

bool safety_is_armed(void) {
    return g_armed;
}

void safety_apply(motor_output_t *out) {
    if (!g_armed) {
        // Zero all motors when disarmed
        for (int i = 0; i < 4; i++) {
            out->motor[i] = 0.0f;
        }
    }
}
