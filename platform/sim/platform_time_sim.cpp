/**
 * @file platform_time_sim.cpp
 * @brief Simulated time implementation
 * 
 * Uses standard C++ chrono for time functions.
 */

#include <chrono>
#include <thread>

extern "C" {
    #include "platform_api/platform_time.h"
}

static auto g_start_time = std::chrono::steady_clock::now();

extern "C" uint32_t platform_millis() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_start_time);
    return static_cast<uint32_t>(duration.count());
}

extern "C" uint64_t platform_micros() {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - g_start_time);
    return static_cast<uint64_t>(duration.count());
}

extern "C" void platform_sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
