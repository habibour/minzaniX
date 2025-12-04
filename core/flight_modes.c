/**
 * @file flight_modes.c
 * @brief Autonomous flight mode implementation
 */

#include "flight_modes.h"
#include <stdio.h>
#include <string.h>

// Current state
static flight_mode_t g_current_mode = FLIGHT_MODE_DISARMED;

// Takeoff state
static takeoff_params_t g_takeoff_params;
static float g_takeoff_start_altitude = 0.0f;
static uint32_t g_takeoff_settle_timer = 0;

// Landing state
static landing_params_t g_landing_params;
static uint32_t g_landing_disarm_timer = 0;

// Altitude targets
static float g_target_altitude = 0.0f;
static float g_target_velocity = 0.0f;

void flight_mode_init(void) {
    g_current_mode = FLIGHT_MODE_DISARMED;
    
    // Default takeoff parameters
    g_takeoff_params.target_altitude = 2.0f;
    g_takeoff_params.climb_rate = 0.5f;
    g_takeoff_params.settle_time_ms = 2000;
    
    // Default landing parameters
    g_landing_params.descent_rate = -0.3f;
    g_landing_params.land_altitude = 0.3f;
    g_landing_params.disarm_delay_ms = 2000;
}

flight_mode_t flight_mode_get(void) {
    return g_current_mode;
}

const char* flight_mode_get_name(flight_mode_t mode) {
    switch (mode) {
        case FLIGHT_MODE_DISARMED: return "DISARMED";
        case FLIGHT_MODE_ARMED: return "ARMED";
        case FLIGHT_MODE_TAKEOFF: return "TAKEOFF";
        case FLIGHT_MODE_HOVER: return "HOVER";
        case FLIGHT_MODE_LAND: return "LAND";
        case FLIGHT_MODE_LANDED: return "LANDED";
        default: return "UNKNOWN";
    }
}

bool flight_mode_request_arm(void) {
    if (g_current_mode == FLIGHT_MODE_DISARMED) {
        g_current_mode = FLIGHT_MODE_ARMED;
        printf("[FLIGHT_MODE] Transitioned to ARMED\n");
        return true;
    }
    return false;
}

bool flight_mode_request_disarm(void) {
    // Can disarm from ARMED or LANDED
    if (g_current_mode == FLIGHT_MODE_ARMED || 
        g_current_mode == FLIGHT_MODE_LANDED) {
        g_current_mode = FLIGHT_MODE_DISARMED;
        printf("[FLIGHT_MODE] Transitioned to DISARMED\n");
        return true;
    }
    return false;
}

bool flight_mode_request_takeoff(const takeoff_params_t* params) {
    // Can takeoff from ARMED or LANDED
    if (g_current_mode == FLIGHT_MODE_ARMED || 
        g_current_mode == FLIGHT_MODE_LANDED) {
        
        if (params) {
            g_takeoff_params = *params;
        }
        
        // Validate parameters
        if (g_takeoff_params.target_altitude < 0.5f) {
            g_takeoff_params.target_altitude = 0.5f;
        }
        if (g_takeoff_params.target_altitude > 15.0f) {
            g_takeoff_params.target_altitude = 15.0f;
        }
        if (g_takeoff_params.climb_rate < 0.2f) {
            g_takeoff_params.climb_rate = 0.2f;
        }
        if (g_takeoff_params.climb_rate > 1.0f) {
            g_takeoff_params.climb_rate = 1.0f;
        }
        
        g_current_mode = FLIGHT_MODE_TAKEOFF;
        g_takeoff_settle_timer = 0;
        
        printf("[FLIGHT_MODE] Transitioned to TAKEOFF (target: %.2fm, rate: %.2fm/s)\n",
               g_takeoff_params.target_altitude, g_takeoff_params.climb_rate);
        return true;
    }
    return false;
}

bool flight_mode_request_land(const landing_params_t* params) {
    // Can land from TAKEOFF, HOVER, or in-flight
    if (g_current_mode == FLIGHT_MODE_TAKEOFF || 
        g_current_mode == FLIGHT_MODE_HOVER ||
        g_current_mode == FLIGHT_MODE_ARMED) {
        
        if (params) {
            g_landing_params = *params;
        }
        
        // Validate parameters
        if (g_landing_params.descent_rate > -0.1f) {
            g_landing_params.descent_rate = -0.1f;
        }
        if (g_landing_params.descent_rate < -1.0f) {
            g_landing_params.descent_rate = -1.0f;
        }
        if (g_landing_params.land_altitude < 0.1f) {
            g_landing_params.land_altitude = 0.1f;
        }
        if (g_landing_params.land_altitude > 0.5f) {
            g_landing_params.land_altitude = 0.5f;
        }
        
        g_current_mode = FLIGHT_MODE_LAND;
        g_landing_disarm_timer = 0;
        
        printf("[FLIGHT_MODE] Transitioned to LAND (rate: %.2fm/s)\n",
               g_landing_params.descent_rate);
        return true;
    }
    return false;
}

bool flight_mode_request_hover(void) {
    // Can hover from TAKEOFF or while flying
    if (g_current_mode == FLIGHT_MODE_TAKEOFF ||
        g_current_mode == FLIGHT_MODE_ARMED) {
        g_current_mode = FLIGHT_MODE_HOVER;
        printf("[FLIGHT_MODE] Transitioned to HOVER (altitude: %.2fm)\n", g_target_altitude);
        return true;
    }
    return false;
}

void flight_mode_update(float current_altitude, float current_velocity, float dt) {
    uint32_t dt_ms = (uint32_t)(dt * 1000.0f);
    
    switch (g_current_mode) {
        case FLIGHT_MODE_DISARMED:
        case FLIGHT_MODE_ARMED:
            // No autonomous control
            g_target_altitude = current_altitude;
            g_target_velocity = 0.0f;
            break;
            
        case FLIGHT_MODE_TAKEOFF: {
            // Record start altitude on first entry
            if (g_takeoff_settle_timer == 0 && g_takeoff_start_altitude == 0.0f) {
                g_takeoff_start_altitude = current_altitude;
            }
            
            // Set climb targets
            g_target_altitude = g_takeoff_params.target_altitude;
            g_target_velocity = g_takeoff_params.climb_rate;
            
            // Check if reached target altitude (within 0.2m)
            float altitude_error = g_takeoff_params.target_altitude - current_altitude;
            if (altitude_error < 0.2f && altitude_error > -0.2f) {
                g_takeoff_settle_timer += dt_ms;
                
                // If settled for required time, transition to hover
                if (g_takeoff_settle_timer >= g_takeoff_params.settle_time_ms) {
                    g_current_mode = FLIGHT_MODE_HOVER;
                    g_takeoff_start_altitude = 0.0f;
                    g_takeoff_settle_timer = 0;
                    printf("[FLIGHT_MODE] Takeoff complete! Transitioned to HOVER\n");
                }
            } else {
                // Reset settle timer if moved away from target
                g_takeoff_settle_timer = 0;
            }
            break;
        }
            
        case FLIGHT_MODE_HOVER:
            // Maintain current altitude
            g_target_velocity = 0.0f;
            // Target altitude already set from takeoff or external command
            break;
            
        case FLIGHT_MODE_LAND: {
            // Descend to landing altitude
            g_target_altitude = g_landing_params.land_altitude;
            g_target_velocity = g_landing_params.descent_rate;
            
            // Check if landed (below landing altitude with low velocity)
            if (current_altitude <= g_landing_params.land_altitude + 0.1f &&
                current_velocity > -0.2f && current_velocity < 0.2f) {
                
                g_landing_disarm_timer += dt_ms;
                
                // If on ground for required time, transition to landed
                if (g_landing_disarm_timer >= g_landing_params.disarm_delay_ms) {
                    g_current_mode = FLIGHT_MODE_LANDED;
                    g_landing_disarm_timer = 0;
                    g_target_altitude = 0.0f;
                    g_target_velocity = 0.0f;
                    printf("[FLIGHT_MODE] Landing complete! Transitioned to LANDED\n");
                }
            } else {
                g_landing_disarm_timer = 0;
            }
            break;
        }
            
        case FLIGHT_MODE_LANDED:
            // On ground, motors should be at low throttle
            g_target_altitude = 0.0f;
            g_target_velocity = 0.0f;
            break;
    }
}

bool flight_mode_get_altitude_target(float* target) {
    if (!target) return false;
    
    if (g_current_mode == FLIGHT_MODE_TAKEOFF ||
        g_current_mode == FLIGHT_MODE_HOVER ||
        g_current_mode == FLIGHT_MODE_LAND) {
        *target = g_target_altitude;
        return true;
    }
    
    return false;
}

bool flight_mode_get_velocity_target(float* target) {
    if (!target) return false;
    
    if (g_current_mode == FLIGHT_MODE_TAKEOFF ||
        g_current_mode == FLIGHT_MODE_HOVER ||
        g_current_mode == FLIGHT_MODE_LAND) {
        *target = g_target_velocity;
        return true;
    }
    
    return false;
}

bool flight_mode_is_altitude_hold_active(void) {
    return (g_current_mode == FLIGHT_MODE_TAKEOFF ||
            g_current_mode == FLIGHT_MODE_HOVER ||
            g_current_mode == FLIGHT_MODE_LAND);
}
