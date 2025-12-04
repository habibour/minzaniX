/**
 * @file udp_bridge.cpp
 * @brief Simple UDP communication for sim testing
 */

#include "udp_bridge.hpp"
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

namespace udp_bridge {

static int g_sock = -1;
static struct sockaddr_in g_fc_addr;
static struct sockaddr_in g_sim_addr;

static imu_sample_t g_latest_imu{};
static baro_sample_t g_latest_baro{};
static bool g_imu_received = false;
static bool g_baro_received = false;

bool init() {
    std::cout << "[UDP_BRIDGE] Initializing UDP connection..." << std::endl;
    
    // Create socket
    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) {
        std::cerr << "[UDP_BRIDGE] Failed to create socket" << std::endl;
        return false;
    }
    
    // Set non-blocking
    int flags = fcntl(g_sock, F_GETFL, 0);
    fcntl(g_sock, F_SETFL, flags | O_NONBLOCK);
    
    // FC address (where we receive)
    memset(&g_fc_addr, 0, sizeof(g_fc_addr));
    g_fc_addr.sin_family = AF_INET;
    g_fc_addr.sin_addr.s_addr = INADDR_ANY;
    g_fc_addr.sin_port = htons(14550);
    
    // Bind to FC port
    if (bind(g_sock, (struct sockaddr*)&g_fc_addr, sizeof(g_fc_addr)) < 0) {
        std::cerr << "[UDP_BRIDGE] Failed to bind to port 14550" << std::endl;
        close(g_sock);
        return false;
    }
    
    // Simulator address (where we send)
    memset(&g_sim_addr, 0, sizeof(g_sim_addr));
    g_sim_addr.sin_family = AF_INET;
    g_sim_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    g_sim_addr.sin_port = htons(14551);
    
    std::cout << "[UDP_BRIDGE] Ready on UDP port 14550" << std::endl;
    return true;
}

bool get_imu(imu_sample_t* imu) {
    if (!imu) return false;
    
    // Drain all available packets
    uint8_t buf[1024];
    for (int i = 0; i < 10; i++) {
        ssize_t len = recvfrom(g_sock, buf, sizeof(buf), 0, nullptr, nullptr);
        if (len < 0) break;
        
        if (len == 24) {  // IMU: 6 floats * 4 bytes
            float* data = (float*)buf;
            g_latest_imu.ax = data[0];
            g_latest_imu.ay = data[1];
            g_latest_imu.az = data[2];
            g_latest_imu.gx = data[3];
            g_latest_imu.gy = data[4];
            g_latest_imu.gz = data[5];
            g_imu_received = true;
        } else if (len == 4) {  // Baro: 1 float
            float* data = (float*)buf;
            g_latest_baro.pressure = data[0];
            g_latest_baro.temperature = 15.0f;
            g_baro_received = true;
        }
    }
    
    if (!g_imu_received) return false;
    
    *imu = g_latest_imu;
    return true;
}

bool get_baro(baro_sample_t* baro) {
    if (!baro || !g_baro_received) return false;
    
    *baro = g_latest_baro;
    return true;
}

bool publish_motors(const motor_output_t* motors) {
    if (!motors) return false;
    
    // Send 4 floats
    float data[4] = {
        motors->motor[0],
        motors->motor[1],
        motors->motor[2],
        motors->motor[3]
    };
    
    ssize_t sent = sendto(g_sock, data, sizeof(data), 0,
                          (struct sockaddr*)&g_sim_addr, sizeof(g_sim_addr));
    
    return sent == sizeof(data);
}

} // namespace udp_bridge
