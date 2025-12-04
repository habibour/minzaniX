/**
 * @file flight_modes.h
 * @brief Autonomous flight mode management
 * 
 * Provides state machine for:
 * - DISARMED: Motors off
 * - ARMED: Motors armed, ready for flight
 * - TAKEOFF: Autonomous ascent to target altitude
 * - HOVER: Maintain current altitude
 * - LAND: Autonomous descent to ground
 */

#ifndef FLIGHT_MODES_H
#define FLIGHT_MODES_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Flight mode enumeration
 */
typedef enum {
    FLIGHT_MODE_DISARMED,   // Motors off
    FLIGHT_MODE_ARMED,      // Motors armed, manual control
    FLIGHT_MODE_TAKEOFF,    // Autonomous takeoff in progress
    FLIGHT_MODE_HOVER,      // Hovering at altitude
    FLIGHT_MODE_LAND,       // Autonomous landing in progress
    FLIGHT_MODE_LANDED      // Landed, ready to disarm
} flight_mode_t;

/**
 * @brief Takeoff parameters
 */
typedef struct {
    float target_altitude;  // Target altitude in meters
    float climb_rate;       // Climb rate in m/s
    float settle_time_ms;   // Time to settle at altitude
} takeoff_params_t;

/**
 * @brief Landing parameters
 */
typedef struct {
    float descent_rate;     // Descent rate in m/s
    float land_altitude;    // Altitude to consider "landed"
    float disarm_delay_ms;  // Delay before auto-disarm
} landing_params_t;

/**
 * @brief Initialize flight mode system
 */
void flight_mode_init(void);

/**
 * @brief Get current flight mode
 */
flight_mode_t flight_mode_get(void);

/**
 * @brief Request arm (transition to ARMED)
 * @return true if transition allowed
 */
bool flight_mode_request_arm(void);

/**
 * @brief Request disarm (transition to DISARMED)
 * @return true if transition allowed
 */
bool flight_mode_request_disarm(void);

/**
 * @brief Request takeoff
 * @param params Takeoff parameters
 * @return true if transition allowed
 */
bool flight_mode_request_takeoff(const takeoff_params_t* params);

/**
 * @brief Request landing
 * @param params Landing parameters
 * @return true if transition allowed
 */
bool flight_mode_request_land(const landing_params_t* params);

/**
 * @brief Request hover at current altitude
 * @return true if transition allowed
 */
bool flight_mode_request_hover(void);

/**
 * @brief Update flight mode state machine
 * @param current_altitude Current altitude in meters
 * @param current_velocity Current vertical velocity in m/s
 * @param dt Time delta in seconds
 */
void flight_mode_update(float current_altitude, float current_velocity, float dt);

/**
 * @brief Get current target altitude
 * @param target Output: current altitude target
 * @return true if altitude control active
 */
bool flight_mode_get_altitude_target(float* target);

/**
 * @brief Get current target velocity
 * @param target Output: current velocity target
 * @return true if velocity control active
 */
bool flight_mode_get_velocity_target(float* target);

/**
 * @brief Check if altitude hold is active
 */
bool flight_mode_is_altitude_hold_active(void);

/**
 * @brief Get mode name as string
 */
const char* flight_mode_get_name(flight_mode_t mode);

#endif // FLIGHT_MODES_H
