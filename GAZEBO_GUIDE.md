# Running minzaniX with Gazebo Harmonic

## Quick Start Guide

### Step 1: Install Gazebo Transport (if not already installed)

```bash
brew install gz-transport13
```

### Step 2: Set Up Environment

Add this to your `~/.zshrc`:
```bash
export GZ_SIM_RESOURCE_PATH=/Users/habibourakash/SWE/minzaniX_1.0/sim_world:$GZ_SIM_RESOURCE_PATH
```

Then reload:
```bash
source ~/.zshrc
```

### Step 3: Rebuild with Gazebo Support

```bash
cd /Users/habibourakash/SWE/minzaniX_1.0
rm -rf build
mkdir build && cd build
cmake ..
make -j4
```

You should see: `Gazebo Transport found - Gazebo integration enabled`

### Step 4: Launch Gazebo Simulation

**Terminal 1** - Start Gazebo:
```bash
cd /Users/habibourakash/SWE/minzaniX_1.0
gz sim sim_world/world.sdf
```

This will open Gazebo with your quadcopter at height 1.0m.

### Step 5: Run Flight Controller

**Terminal 2** - Run your FC connected to Gazebo:
```bash
cd /Users/habibourakash/SWE/minzaniX_1.0/build
./minzanix_sim --gazebo
```

You should see:
```
=== minzaniX 1.0 Flight Controller (Simulation) ===
Mode: Gazebo Harmonic Integration
[GAZEBO_BRIDGE] Initializing Gazebo transport...
[GAZEBO_BRIDGE] Motor publishers ready
[GAZEBO_BRIDGE] Subscribing to IMU: /mini_quad/imu
[GAZEBO_BRIDGE] IMU subscription successful
Waiting for IMU data from Gazebo...
IMU data received from Gazebo!
Flight controller initialized
System ARMED
Starting main loop...
Att: R=0.00 P=0.00 Y=0.42 | Motors: 0.47 0.53 0.47 0.53
```

## Monitoring & Debugging

### Monitor IMU Data
**Terminal 3**:
```bash
gz topic -e -t /mini_quad/imu
```

### Monitor Motor Commands
**Terminal 4**:
```bash
gz topic -e -t /mini_quad/motor0/cmd_thrust
```

### List All Topics
```bash
gz topic -l
```

### View Topic Info
```bash
gz topic -i -t /mini_quad/imu
```

## Understanding the Setup

### Data Flow

```
┌─────────────────────────────────────────────────┐
│          Gazebo Harmonic Simulator              │
│                                                 │
│  ┌──────────────────────────────────────────┐  │
│  │        mini_quad Model                   │  │
│  │  - Rigid body physics                    │  │
│  │  - IMU sensor plugin                     │  │
│  │  - Gravity, collisions                   │  │
│  └──────────────────────────────────────────┘  │
│                                                 │
│         IMU Data ↓      ↑ Motor Commands       │
│         /mini_quad/imu  /mini_quad/motor*/cmd  │
└─────────────────────────────────────────────────┘
                 ↓                ↑
         gz-transport         gz-transport
                 ↓                ↑
┌─────────────────────────────────────────────────┐
│         minzanix_sim (Your FC Code)             │
│                                                 │
│  gazebo_bridge.cpp                              │
│     ↓ IMU Callback                              │
│  estimator.c → controller.c → mixer.c           │
│     ↑ Motor Commands                            │
│  gazebo_bridge.cpp                              │
└─────────────────────────────────────────────────┘
```

### Topics Used

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/mini_quad/imu` | gz.msgs.IMU | Gazebo → FC | IMU sensor data (accel, gyro) |
| `/mini_quad/motor0/cmd_thrust` | gz.msgs.Double | FC → Gazebo | Motor 0 thrust [0-1] |
| `/mini_quad/motor1/cmd_thrust` | gz.msgs.Double | FC → Gazebo | Motor 1 thrust [0-1] |
| `/mini_quad/motor2/cmd_thrust` | gz.msgs.Double | FC → Gazebo | Motor 2 thrust [0-1] |
| `/mini_quad/motor3/cmd_thrust` | gz.msgs.Double | FC → Gazebo | Motor 3 thrust [0-1] |

## What You'll See

### In Gazebo:
- Quadcopter model hovering at 1m altitude
- Motors spinning (visualized as cylinders)
- Ground plane below

### In Terminal (FC output):
```
Att: R=0.02 P=-0.01 Y=1.23 | Motors: 0.49 0.51 0.50 0.50
```
- **Att**: Attitude angles (degrees)
- **Motors**: Motor commands [0.0 = off, 1.0 = max]

### Expected Behavior:
1. **Roll/Pitch ≈ 0°**: Quadcopter stays level (complementary filter working)
2. **Yaw drifts**: Slowly increases (no magnetometer)
3. **Motors ≈ 0.5**: Hover throttle with small corrections

## Running Without Gazebo

If you want to test without Gazebo (fake IMU):
```bash
./minzanix_sim
# OR explicitly:
./minzanix_sim --no-gazebo
```

## Troubleshooting

### "Failed to initialize Gazebo bridge"
- Make sure `gz-transport13` is installed: `brew list | grep gz-transport`
- Rebuild: `rm -rf build && ./build.sh`

### "No IMU data received from Gazebo"
- Check Gazebo is running: `gz topic -l` should show `/mini_quad/imu`
- Verify model loaded: Look for quadcopter in Gazebo window
- Check resource path: `echo $GZ_SIM_RESOURCE_PATH`

### "Cannot find model://mini_quad"
```bash
export GZ_SIM_RESOURCE_PATH=/Users/habibourakash/SWE/minzaniX_1.0/sim_world:$GZ_SIM_RESOURCE_PATH
```

### Gazebo window is black/empty
- Press play button (▶️) in Gazebo
- Reset view: Right-click → "View" → "Reset"

### Quadcopter falls through ground
- Check physics is enabled in world.sdf
- Make sure ground plane collision is set

## Next Steps

### Test Different Conditions

1. **Tilt the quadcopter** in Gazebo (drag with mouse)
   - Watch FC correct attitude
   - See motor outputs change

2. **Add disturbances** (wind plugin)
   - Test controller robustness

3. **Change PID gains** in `core/params.c`
   - Tune for better performance

4. **Add telemetry plotting**
   - Use `tools/plot_logs.py`

### Advanced Integration

1. **Add motor thrust physics**
   - Implement `motor_plugin.cpp`
   - Apply forces to quadcopter links

2. **Add battery simulation**
   - Voltage drop over time
   - Thrust reduction

3. **Add wind/turbulence**
   - Gazebo wind plugin
   - Test disturbance rejection

## Useful Commands

```bash
# List all Gazebo topics
gz topic -l

# Monitor topic at 10Hz
gz topic -e -t /mini_quad/imu -d 10

# Publish test motor command
gz topic -t /mini_quad/motor0/cmd_thrust -m gz.msgs.Double -p "data: 0.8"

# Get topic statistics
gz topic -i -t /mini_quad/imu

# Pause simulation
gz service -s /world/minzanix_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: true'

# Play simulation  
gz service -s /world/minzanix_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req 'pause: false'
```

## Success Criteria

✅ Gazebo opens and shows quadcopter  
✅ Flight controller connects to Gazebo  
✅ IMU data is received (9.81 m/s² on Z-axis when level)  
✅ Motor commands are published (≈0.5 for hover)  
✅ Attitude estimate is reasonable (±5° when level)  
✅ Controller attempts to stabilize (motors adjust when tilted)

---

**You're now running your flight controller in a physics-based simulation!** 🚁

This is a major milestone - you can now test and tune your FC safely before hardware testing.
