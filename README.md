# minzaniX 1.0

**A Lightweight, MCU-Independent PX4-Style Flight Controller**

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![License](https://img.shields.io/badge/license-MIT-blue)]()

## Overview

minzaniX 1.0 is a modular, portable flight controller architecture inspired by PX4 and ArduPilot. It features:

- **MCU-Independent Core**: Flight stack works on any platform
- **Clean Abstraction Layer**: Platform-specific code isolated from control logic
- **Multiple Backends**: Simulation (Gazebo) and STM32 hardware support
- **Lightweight**: Suitable for resource-constrained microcontrollers
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

**For Simulation:**
```bash
# Ubuntu/Debian
sudo apt install build-essential cmake

# macOS
brew install cmake
```

**For Hardware (STM32):**
- STM32CubeIDE
- STM32F103 Blue Pill board
- IMU (MPU6050 or similar)
- 4x brushless motors + ESCs

### Build Simulation

```bash
mkdir build && cd build
cmake ..
make
```

### Run Simulation

```bash
./minzanix_sim
```

Expected output:
```
=== minzaniX 1.0 Flight Controller (Simulation) ===
[SIM_IMU] Initialized
[SIM_PWM] Initialized
Flight controller initialized
System ARMED
Starting main loop...
Att: R=0.00 P=0.00 Y=1.72 | Motors: 0.50 0.50 0.50 0.50
```

### Build for STM32

1. Open project in STM32CubeIDE
2. Import `platform/bluepill/` files
3. Configure peripherals (I2C, PWM, UART)
4. Build and flash to Blue Pill

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

## Hardware Setup

### STM32 Blue Pill Connections

| Peripheral | Pin | Function |
|------------|-----|----------|
| I2C1 SDA   | PB7 | IMU data |
| I2C1 SCL   | PB6 | IMU clock |
| TIM1 CH1   | PA8 | Motor 0 PWM |
| TIM1 CH2   | PA9 | Motor 1 PWM |
| TIM1 CH3   | PA10| Motor 2 PWM |
| TIM1 CH4   | PA11| Motor 3 PWM |
| USART1 TX  | PA9 | Telemetry |
| USART1 RX  | PA10| Commands |

## Gazebo Integration (Optional)

To connect with Gazebo Harmonic:

1. Install gz-transport: `sudo apt install gz-harmonic`
2. Uncomment Gazebo code in `platform/sim/gazebo_bridge.cpp`
3. Rebuild with Gazebo support

Launch world:
```bash
gz sim sim_world/world.sdf
```

## Project Status

- ✅ Core flight stack implemented
- ✅ Simulation backend working
- ✅ STM32 backend skeleton complete
- 🔄 Gazebo integration (optional)
- 🔄 Unit tests
- 🔄 Advanced features (altitude hold, position control)

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
