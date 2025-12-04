/**
 * @file udp_bridge.hpp
 * @brief Simple UDP bridge header
 */

#ifndef UDP_BRIDGE_HPP
#define UDP_BRIDGE_HPP

extern "C" {
    #include "platform_api/platform_imu.h"
    #include "platform_api/platform_pwm.h"
    #include "core/fc_types.h"
}

namespace udp_bridge {
    bool init();
    bool get_imu(imu_sample_t* imu);
    bool get_baro(baro_sample_t* baro);
    bool publish_motors(const motor_output_t* motors);
}

#endif
