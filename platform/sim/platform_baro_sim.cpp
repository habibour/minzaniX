/**
 * @file platform_baro_sim.cpp
 * @brief Simulation platform barometer implementation
 */

#include "platform_api/platform_baro.h"
#include "gazebo_bridge.hpp"

bool platform_baro_init(void) {
    // Barometer initialization handled by gazebo_bridge
    return true;
}

bool platform_baro_read(baro_sample_t *sample) {
    return gazebo_bridge::get_baro(sample);
}
