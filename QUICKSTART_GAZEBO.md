# Quick Start: Gazebo Simulation

## Prerequisites ✅
- ✅ Gazebo Harmonic installed on your Mac
- ✅ Code compiled with Gazebo support

## Running the Simulation

### Option 1: Automated (Recommended)

```bash
cd /Users/habibourakash/SWE/minzaniX_1.0
./run_gazebo.sh
```

This script will:
1. Start Gazebo with your quadcopter world
2. Wait for initialization
3. Connect your flight controller
4. Start the control loop

### Option 2: Manual (Two Terminals)

**Terminal 1 - Gazebo:**
```bash
cd /Users/habibourakash/SWE/minzaniX_1.0
export GZ_SIM_RESOURCE_PATH=$PWD/sim_world:$GZ_SIM_RESOURCE_PATH
gz sim -g sim_world/world.sdf
```

**Terminal 2 - Flight Controller:**
```bash
cd /Users/habibourakash/SWE/minzaniX_1.0/build
./minzanix_sim --gazebo
```

## What You Should See

### Gazebo Window:
- 3D view of your quadcopter
- Green ground plane
- Quadcopter hovering at 1m height
- Four colored propeller discs (motors)

### Terminal Output:
```
=== minzaniX 1.0 Flight Controller (Simulation) ===
Mode: Gazebo Harmonic Integration
[GAZEBO_BRIDGE] Initializing Gazebo transport...
[GAZEBO_BRIDGE] Motor publishers ready
[GAZEBO_BRIDGE] IMU subscription successful
IMU data received from Gazebo!
Flight controller initialized
System ARMED
Starting main loop...
Att: R=0.02 P=-0.01 Y=0.85 | Motors: 0.49 0.51 0.50 0.50
```

## Testing the System

### 1. Verify IMU Data Flow
In a new terminal:
```bash
gz topic -e -t /mini_quad/imu
```
You should see IMU messages at ~1000 Hz

### 2. Monitor Motor Commands
```bash
gz topic -e -t /mini_quad/motor0/cmd_thrust
```
You should see motor thrust values around 0.5

### 3. Test Controller Response
In Gazebo GUI:
- Click on the quadcopter
- Drag to tilt it
- Release and watch it stabilize
- Check motor values adjust in terminal

### 4. Check All Topics
```bash
gz topic -l
```
Expected topics:
- `/mini_quad/imu`
- `/mini_quad/motor0/cmd_thrust`
- `/mini_quad/motor1/cmd_thrust`
- `/mini_quad/motor2/cmd_thrust`
- `/mini_quad/motor3/cmd_thrust`

## Common Issues

### "No IMU data received"
**Solution:**
1. Make sure Gazebo window is open
2. Click the Play button (▶️) in Gazebo
3. Check quadcopter model is visible

### "Cannot find model://mini_quad"
**Solution:**
```bash
export GZ_SIM_RESOURCE_PATH=/Users/habibourakash/SWE/minzaniX_1.0/sim_world:$GZ_SIM_RESOURCE_PATH
```
Add to `~/.zshrc` to make permanent

### Quadcopter not visible in Gazebo
**Solution:**
1. Right-click in Gazebo → Reset camera
2. Look for the quadcopter at (0, 0, 1) position
3. Zoom out if needed

### Build errors about gz-transport
**Solution:**
```bash
brew install gz-transport13
cd /Users/habibourakash/SWE/minzaniX_1.0
rm -rf build
./build.sh
```

## Understanding the Output

### Attitude (Att):
- **R** = Roll (degrees) - rotation about forward axis
- **P** = Pitch (degrees) - rotation about side axis  
- **Y** = Yaw (degrees) - rotation about vertical axis

### Motors:
- Values from 0.0 (off) to 1.0 (max thrust)
- Hover ≈ 0.5 for each motor
- Small differences = stabilization corrections

### Expected Behavior:
1. **Level flight**: R ≈ 0°, P ≈ 0° (±2°)
2. **Yaw drift**: Y slowly increases (no magnetometer)
3. **Motor balance**: All around 0.5 ± 0.05
4. **Stability**: Returns to level after disturbance

## Next Steps

### Test Scenarios:

1. **Manual Tilt Test**
   - Drag quadcopter in Gazebo
   - Watch FC correct attitude
   - Observe motor responses

2. **Parameter Tuning**
   - Edit `core/params.c`
   - Adjust PID gains
   - Rebuild and test

3. **Data Logging**
   - Redirect output to file:
     ```bash
     ./minzanix_sim --gazebo | tee flight_log.txt
     ```
   - Plot with `tools/plot_logs.py`

4. **Add Disturbances**
   - Apply forces in Gazebo GUI
   - Test controller robustness

## Keyboard Shortcuts (Gazebo)

- **Space**: Play/Pause simulation
- **R**: Reset view
- **Ctrl+R**: Reset world
- **Mouse drag**: Rotate view
- **Mouse wheel**: Zoom
- **Shift+drag**: Pan view

## Success Checklist

- ✅ Gazebo window opens
- ✅ Quadcopter model visible
- ✅ FC connects to Gazebo
- ✅ IMU data flows (check with `gz topic -e`)
- ✅ Motor commands publish
- ✅ Attitude estimate reasonable
- ✅ Controller stabilizes when tilted

## Stopping the Simulation

1. Press **Ctrl+C** in FC terminal
2. Close Gazebo window
3. Or use: `killall gz`

---

**You're now running a full physics-based simulation!** 🎉

The quadcopter model receives realistic IMU data from Gazebo's physics engine, and your flight controller processes it in real-time. This is exactly how professional autopilot systems are tested before hardware deployment.
