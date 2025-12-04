# minzaniX 1.0

**A Lightweight, Autonomous Flight Controller for Simulation**

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![License](https://img.shields.io/badge/license-MIT-blue)]()

## Overview

minzaniX 1.0 is a clean, minimal flight controller focused on autonomous flight in Gazebo simulation. It features:

- **Core Flight Control**: Attitude estimation, altitude control, PID tuning
- **Autonomous Modes**: Takeoff, hover, and landing sequences
- **Gazebo Integration**: Full sensor simulation (IMU, barometer)
- **Simplified**: All MAVLink and external communication removed
- **Educational**: Clear, documented code for learning flight control

## Architecture

```
minzaniX 1.0/
├── core/              # MCU-independent flight stack
├── platform_api/      # Abstract hardware interfaces
├── platform/
│   ├── sim/          # PC simulation backend
│   └── bluepill/     # STM32 hardware backend
├── sim_world/        # Gazebo models
└── docs/             # Documentation
```

## Features

### Core Flight Stack
- **Estimator**: Complementary filter for attitude estimation
- **Controller**: Cascaded PID for roll/pitch/yaw/throttle
- **Mixer**: X-configuration quadcopter motor mixing
- **Safety**: Arming logic and failsafe mechanisms
- **Parameters**: Runtime-configurable PID gains

### Platform Support
- ✅ **Simulation**: Run on Linux/macOS with Gazebo Harmonic
- ✅ **STM32 Blue Pill**: Real hardware with FreeRTOS
- 🔄 **Extensible**: Easy to add new platforms

## Quick Start

### Prerequisites

```bash
# macOS
brew install cmake gz-harmonic

# Linux
sudo apt install build-essential cmake gz-harmonic
```

### Build

```bash
./build.sh
```

### Run Autonomous 10m Takeoff Test

**Automatic (Single Terminal):**
```bash
./run_test.sh
```

**Manual (Two Terminals) - Recommended for macOS:**

Terminal 1 - Start Gazebo:
```bash
export GZ_SIM_RESOURCE_PATH=$PWD/sim_world:$GZ_SIM_RESOURCE_PATH
gz sim sim_world/simple_world.sdf
```
Click the **PLAY** button ▶️ in Gazebo GUI.

Terminal 2 - Start Flight Controller:
```bash
cd build
./minzanix_sim --gazebo --auto-takeoff
```

### Expected Output

```
=== minzaniX 1.0 Flight Controller (Simulation) ===
Mode: Gazebo Harmonic Integration
Waiting for sensor data from Gazebo...
IMU data received from Gazebo!
Flight controller initialized

=== SENSOR CALIBRATION ===
Calibrating... 1.0s
Calibrating... 2.0s
=== CALIBRATION COMPLETE ===

=== OPEN-LOOP TAKEOFF TEST ===
[AUTO] Arming...
[AUTO] Armed!
Starting main loop...
[OPEN-LOOP] Alt: 0.50m Vel:0.00m/s | Motors: 0.95 0.95 0.95 0.95
[OPEN-LOOP] Alt: 2.34m Vel:1.45m/s | Motors: 0.95 0.95 0.95 0.95
[OPEN-LOOP] Alt: 5.67m Vel:1.89m/s | Motors: 0.95 0.95 0.95 0.95
[OPEN-LOOP] Alt: 9.12m Vel:1.52m/s | Motors: 0.95 0.95 0.95 0.95
[OPEN-LOOP] Alt: 10.01m Vel:0.34m/s | Motors: 0.85 0.85 0.85 0.85
```

The drone climbs to 10m and hovers!

## Configuration

### PID Tuning

Edit default gains in `core/params.c`:

```c
static pid_params_t g_pid_params = {
    .roll_kp = 1.5f, .roll_ki = 0.0f, .roll_kd = 0.3f,
    .pitch_kp = 1.5f, .pitch_ki = 0.0f, .pitch_kd = 0.3f,
    .yaw_kp = 2.0f, .yaw_ki = 0.0f, .yaw_kd = 0.5f
};
```

### Loop Rate

- **Simulation**: 250 Hz (4ms period)
- **STM32**: 500 Hz (2ms period)

## Troubleshooting

### "No IMU data received from Gazebo"
- Make sure Gazebo is running and **playing** (not paused)
- On macOS, use GUI mode instead of server mode (`-s`)
- Check topics are publishing: `gz topic -l | grep imu`

### Build Errors
```bash
rm -rf build/*
./build.sh
```

## Cleaned Up Files

The following unnecessary files were removed:
- All MAVLink-related code and dependencies
- Python test scripts (`autonomous_mission.py`, etc.)
- Shell test scripts (`fly_10m.sh`, etc.)
- External communication tools
- Outdated documentation files

## What the Test Does

1. **Calibration (3s)**: Collects IMU and barometer baseline data
2. **Arm**: Enables motors
3. **Climb**: Applies 0.95 fixed thrust to reach 10m
4. **Hover**: Reduces to 0.85 thrust to maintain 10m altitude

This is an **open-loop control** test - no PID feedback, just fixed thrust based on altitude.

## Project Status

- ✅ Core flight stack implemented
- ✅ Gazebo Harmonic integration working
- ✅ Autonomous takeoff and landing modes
- ✅ Altitude estimation with barometer
- ✅ PID controllers tuned for stable flight
- ✅ Clean codebase (MAVLink removed)

## Documentation

- [Architecture](docs/architecture.md) - System design overview
- [Timing](docs/timing.md) - RTOS task scheduling
- [Requirements](docs/requirements.md) - Feature checklist

## Contributing

This is an educational project. Contributions welcome!

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

MIT License - see LICENSE file for details

## Author

**habibourakash**  
November 2025

Built for learning, research, and portfolio demonstration.

## Acknowledgments

- Inspired by PX4 and ArduPilot architectures
- Complementary filter from Mahony et al.
- STM32 HAL by STMicroelectronics

---

**Keywords**: Flight controller, quadcopter, drone, autopilot, embedded systems, STM32, Gazebo, PID control, attitude estimation, MCU-independent, portable firmware
