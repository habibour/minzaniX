# minzaniX 1.0 - Quick Start Guide

## Prerequisites

- Gazebo Harmonic (gz-sim) installed
- PX4-Autopilot cloned at `~/PX4-Autopilot`
- CMake 3.10+
- C++17 compiler

---

## Step 1: Build minzaniX Flight Controller

```bash
cd /Users/habibourakash/SWE/minzaniX_1.0
./build.sh
```

**Expected Output:**
```
======================================
  minzaniX 1.0 - Build & Run Script
======================================
[*] Configuring CMake...
[*] Building project...
[✓] Build successful!
```

---

## Step 2: Launch Gazebo with PX4 x500 Drone

```bash
# Set the PX4 model path
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH"

# Launch Gazebo server (no GUI, faster)
cd /Users/habibourakash/SWE/minzaniX_1.0
gz sim -s -r sim_world/world.sdf &
```

**Wait 3-5 seconds for Gazebo to initialize**

---

## Step 3: Verify Drone is Loaded

```bash
# Check available topics
gz topic -l | grep x500

# Expected output:
# /world/minzanix_world/model/x500/link/base_link/sensor/imu_sensor/imu
# /x500/command/motor_speed
```

```bash
# Test IMU data (should show sensor readings)
gz topic -e -t /world/minzanix_world/model/x500/link/base_link/sensor/imu_sensor/imu -n 1
```

---

## Step 4: Run minzaniX Flight Controller

```bash
cd /Users/habibourakash/SWE/minzaniX_1.0/build
./minzanix_sim --gazebo
```

**Expected Output:**
```
=== minzaniX 1.0 Flight Controller (Simulation) ===
Mode: Gazebo Harmonic Integration
[GAZEBO_BRIDGE] Initializing Gazebo transport...
[GAZEBO_BRIDGE] Motor publishers ready
[GAZEBO_BRIDGE] Subscribing to IMU: /world/minzanix_world/model/x500/link/base_link/sensor/imu_sensor/imu
[GAZEBO_BRIDGE] IMU subscription successful
Waiting for IMU data from Gazebo...
IMU data received from Gazebo!
[SIM_IMU] Initialized
[SIM_PWM] Initialized
[SIM_UART] Initialized
Flight controller initialized
System ARMED
Starting main loop...
Att: R=-0.00 P=-0.01 Y=-0.00 | Motors: 0.50 0.50 0.50 0.50
Att: R=0.00 P=0.00 Y=0.00 | Motors: 0.50 0.50 0.50 0.50
...
```

---

## Step 5: Observe Values

### Real-time Flight Controller Output

The terminal shows:
- **R (Roll)**: Attitude angle in radians around X-axis
- **P (Pitch)**: Attitude angle in radians around Y-axis  
- **Y (Yaw)**: Attitude angle in radians around Z-axis
- **Motors**: Throttle commands to motors 0-3 (normalized 0.0-1.0)

### Monitor Specific Topics

**Watch IMU data:**
```bash
gz topic -e -t /world/minzanix_world/model/x500/link/base_link/sensor/imu_sensor/imu
```

**Watch Motor commands:**
```bash
gz topic -e -t /x500/command/motor_speed
```

**Monitor all x500 topics:**
```bash
watch -n 0.5 'gz topic -l | grep x500'
```

---

## Optional: Launch with Gazebo GUI

If you want to see the drone visually:

```bash
# Stop server-only instance first
pkill gz
sleep 2

# Launch with GUI
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH"
gz sim sim_world/world.sdf
```

Then in another terminal:
```bash
cd /Users/habibourakash/SWE/minzaniX_1.0/build
./minzanix_sim --gazebo
```

---

## Complete Command Sequence (Copy-Paste Ready)

```bash
# Terminal 1: Build and launch Gazebo
cd /Users/habibourakash/SWE/minzaniX_1.0
./build.sh
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH"
gz sim -s -r sim_world/world.sdf &
sleep 5

# Terminal 2: Run flight controller
cd /Users/habibourakash/SWE/minzaniX_1.0/build
./minzanix_sim --gazebo

# Terminal 3 (optional): Monitor topics
gz topic -l | grep x500
gz topic -e -t /world/minzanix_world/model/x500/link/base_link/sensor/imu_sensor/imu -n 5
```

---

## Troubleshooting

**No IMU data received:**
```bash
# Check if Gazebo is running
ps aux | grep "gz sim"

# Check if IMU topic exists
gz topic -l | grep imu
```

**Model not found:**
```bash
# Verify PX4 path is set
echo $GZ_SIM_RESOURCE_PATH

# Should include: /Users/habibourakash/PX4-Autopilot/Tools/simulation/gz/models
```

**Stop everything:**
```bash
pkill -9 gz
pkill -9 ruby
pkill minzanix_sim
```

---

## What You're Seeing

- **Stable attitude angles** (~0.00-0.02°): Your complementary filter is working
- **Hover throttle** (0.50): PD controller maintaining level flight
- **Low noise**: Sensor fusion filtering out high-frequency noise
- **Real physics**: x500 model includes realistic mass, inertia, drag, thrust

Your minzaniX flight controller is successfully stabilizing a professional PX4 quadcopter! 🚁
