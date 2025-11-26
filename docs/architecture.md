# minzaniX 1.0 Architecture

## System Overview

minzaniX follows a layered architecture that separates flight control logic from hardware-specific code.

```
┌─────────────────────────────────────────┐
│     External Interface (RC/Autopilot)   │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│      Flight Controller Interface        │
│         (fc_interface.c)                │
└─────────────────┬───────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼───────┐  ┌────────▼────────┐
│   Estimator   │  │   Controller    │
│ (attitude est)│  │  (PID loops)    │
└───────┬───────┘  └────────┬────────┘
        │                   │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │      Mixer        │
        │  (motor mixing)   │
        └─────────┬─────────┘
                  │
        ┌─────────▼─────────┐
        │      Safety       │
        │  (arm/failsafe)   │
        └─────────┬─────────┘
                  │
┌─────────────────▼───────────────────────┐
│       Platform Abstraction Layer        │
│  (platform_imu, platform_pwm, etc.)     │
└─────────────────┬───────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
┌───────▼──────┐   ┌────────▼────────┐
│ Simulation   │   │   STM32 + HW    │
│   Backend    │   │     Backend     │
└──────────────┘   └─────────────────┘
```

## Core Modules

### 1. Estimator (`estimator.c`)

**Purpose**: Fuse IMU sensor data to estimate vehicle attitude

**Algorithm**: Complementary filter
- High-pass: Gyroscope integration (fast, drifts)
- Low-pass: Accelerometer angles (slow, no drift)
- Fusion: `angle = 0.98 * gyro_angle + 0.02 * accel_angle`

**Inputs**:
- Accelerometer [m/s²]
- Gyroscope [rad/s]

**Outputs**:
- Roll, pitch, yaw [rad]
- Angular rates [rad/s]

**Update Rate**: Same as IMU (500-1000 Hz)

### 2. Controller (`controller.c`)

**Purpose**: Generate control commands to achieve desired attitude

**Algorithm**: Cascaded PID
- Rate loop: P + D on angular velocity
- Angle loop: P on angle error (future enhancement)

**Control Law**:
```
roll_cmd = Kp * (roll_sp - roll) - Kd * roll_rate
pitch_cmd = Kp * (pitch_sp - pitch) - Kd * pitch_rate
yaw_cmd = Kp * (yaw_sp - yaw) - Kd * yaw_rate
```

**Inputs**:
- Current attitude (from estimator)
- Setpoint (from RC/autopilot)

**Outputs**:
- Roll/pitch/yaw torque commands [-1, 1]
- Thrust command [0, 1]

### 3. Mixer (`mixer.c`)

**Purpose**: Convert control commands to individual motor speeds

**Algorithm**: X-frame mixing matrix

```
M0 = T + R + P + Y  (Front-Left, CW)
M1 = T - R + P - Y  (Front-Right, CCW)
M2 = T - R - P + Y  (Rear-Right, CW)
M3 = T + R - P - Y  (Rear-Left, CCW)
```

Where:
- T = Thrust
- R = Roll command
- P = Pitch command
- Y = Yaw command

### 4. Safety (`safety.c`)

**Purpose**: Enforce arming conditions and failsafes

**Features**:
- Arm/disarm state machine
- Motor output gating (zero when disarmed)
- Failsafe triggers (future: RC loss, battery low)

### 5. Parameters (`params.c`)

**Purpose**: Store and manage configuration

**Storage**:
- RAM: Runtime parameters
- Persistent: EEPROM/Flash (platform-specific)

## Platform Abstraction Layer

All hardware-specific code is isolated behind abstract interfaces:

### `platform_imu.h`
- `platform_imu_init()`: Initialize IMU sensor
- `platform_imu_read()`: Read accelerometer and gyroscope

### `platform_pwm.h`
- `platform_pwm_init()`: Initialize motor PWM outputs
- `platform_pwm_write()`: Set motor speeds [0.0-1.0]

### `platform_time.h`
- `platform_millis()`: Get milliseconds since boot
- `platform_micros()`: Get microseconds since boot
- `platform_sleep_ms()`: Blocking delay

### `platform_uart.h`
- `platform_uart_init()`: Initialize serial port
- `platform_uart_write()`: Send telemetry data
- `platform_uart_print()`: Print debug messages

## Execution Flow

### Initialization
1. Platform initializes hardware (IMU, PWM, timers)
2. Flight controller initializes (`fc_init()`)
3. Estimator resets to level attitude
4. Controller clears integral terms
5. Safety disarms motors

### Main Loop (250-500 Hz)
1. Read IMU sample → `platform_imu_read()`
2. Update estimator → `estimator_update()`
3. Run controller → `controller_update()`
4. Mix motor commands → `mixer_mix()`
5. Apply safety → `safety_apply()`
6. Write motors → `platform_pwm_write()`
7. Sleep until next cycle

## Key Design Decisions

### Why Complementary Filter?
- **Pros**: Simple, fast, no matrix operations
- **Cons**: Less accurate than EKF
- **Verdict**: Good enough for attitude hold, upgradeable later

### Why PD instead of PID?
- **Reason**: Avoid integral windup during development
- **Future**: Add integral term with anti-windup

### Why X-frame?
- **Reason**: Most common quadcopter configuration
- **Alternative**: + frame, hex, octo (add new mixers)

### Why 500 Hz loop?
- **Reason**: Balance between latency and CPU load
- **Note**: Can run up to 1 kHz on faster MCUs

## Testing Strategy

1. **Unit Tests**: Test each module independently
2. **SITL**: Software-in-the-loop with Gazebo
3. **HITL**: Hardware-in-the-loop with real IMU
4. **Flight Tests**: Tethered → Free flight

## Future Enhancements

- [ ] Extended Kalman Filter (EKF)
- [ ] Altitude hold (barometer + ultrasonic)
- [ ] Position hold (GPS)
- [ ] Acro mode (rate control only)
- [ ] Autonomous waypoint navigation
- [ ] Black-box logging

---

**Last Updated**: November 2025  
**Author**: habibourakash
