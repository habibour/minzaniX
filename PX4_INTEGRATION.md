# Using PX4 Models with minzaniX Flight Controller

## Overview

Instead of creating custom Gazebo models, you can use proven PX4 drone models with your flight controller. This gives you realistic physics and tested models.

## Option 1: Use Gazebo Fuel Models (Easiest)

Gazebo Fuel has pre-made drone models you can download automatically.

### Update world.sdf to use a Fuel model:

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV</uri>
  <name>quadcopter</name>
  <pose>0 0 0.2 0 0 0</pose>
</include>
```

Popular options:
- `X3 UAV` - Simple quadcopter
- `Quadcopter` - Basic quad
- `Iris` - If available

## Option 2: Use PX4-Autopilot Models (Best for Realism)

### Step 1: Clone PX4-Autopilot

```bash
cd ~/
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
```

### Step 2: Set Gazebo Model Path

Add to `~/.zshrc`:
```bash
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=/Users/habibourakash/SWE/minzaniX_1.0/sim_world:$GZ_SIM_RESOURCE_PATH
```

Then:
```bash
source ~/.zshrc
```

### Step 3: Update Your world.sdf

```xml
<include>
  <uri>model://x500</uri>
  <name>quadcopter</name>
  <pose>0 0 0.2 0 0 0</pose>
</include>
```

Available PX4 models:
- `x500` - Standard quadcopter
- `x500_depth` - With camera
- `rc_cessna` - Fixed wing
- `standard_vtol` - VTOL aircraft

## Option 3: Simple Working Model (No PX4 Needed)

I'll create a minimal working Gazebo model for you right now:

### Create simple_quad_world.sdf:

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="simple_quad_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Simple quadcopter inline -->
    <model name="quadcopter">
      <pose>0 0 0.5 0 0 0</pose>
      
      <link name="base_link">
        <inertial>
          <mass>1.5</mass>
          <inertia>
            <ixx>0.029125</ixx>
            <iyy>0.029125</iyy>
            <izz>0.055225</izz>
          </inertia>
        </inertial>
        
        <visual name="visual">
          <geometry>
            <box>
              <size>0.47 0.47 0.11</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 0.0 1</ambient>
            <diffuse>0.0 0.0 0.0 1</diffuse>
          </material>
        </visual>
        
        <collision name="collision">
          <geometry>
            <box>
              <size>0.47 0.47 0.11</size>
            </box>
          </geometry>
        </collision>
        
        <!-- IMU Sensor -->
        <sensor name="imu_sensor" type="imu">
          <always_on>1</always_on>
          <update_rate>1000</update_rate>
          <visualize>true</visualize>
          <topic>quadcopter/imu</topic>
          <enable_metrics>false</enable_metrics>
        </sensor>
      </link>
      
      <!-- Rotors -->
      <link name="rotor_0">
        <pose>0.13 0.22 0.023 0 0 0</pose>
        <inertial>
          <mass>0.005</mass>
          <inertia>
            <ixx>9.75e-07</ixx>
            <iyy>0.000273104</iyy>
            <izz>0.000274004</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.005</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
      
      <joint name="rotor_0_joint" type="revolute">
        <parent>base_link</parent>
        <child>rotor_0</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      
      <link name="rotor_1">
        <pose>-0.13 0.20 0.023 0 0 0</pose>
        <inertial>
          <mass>0.005</mass>
          <inertia>
            <ixx>9.75e-07</ixx>
            <iyy>0.000273104</iyy>
            <izz>0.000274004</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.005</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
      
      <joint name="rotor_1_joint" type="revolute">
        <parent>base_link</parent>
        <child>rotor_1</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      
      <link name="rotor_2">
        <pose>-0.13 -0.20 0.023 0 0 0</pose>
        <inertial>
          <mass>0.005</mass>
          <inertia>
            <ixx>9.75e-07</ixx>
            <iyy>0.000273104</iyy>
            <izz>0.000274004</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.005</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
      
      <joint name="rotor_2_joint" type="revolute">
        <parent>base_link</parent>
        <child>rotor_2</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      
      <link name="rotor_3">
        <pose>0.13 -0.22 0.023 0 0 0</pose>
        <inertial>
          <mass>0.005</mass>
          <inertia>
            <ixx>9.75e-07</ixx>
            <iyy>0.000273104</iyy>
            <izz>0.000274004</izz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.005</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 1 0 1</ambient>
            <diffuse>1 1 0 1</diffuse>
          </material>
        </visual>
      </link>
      
      <joint name="rotor_3_joint" type="revolute">
        <parent>base_link</parent>
        <child>rotor_3</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      
    </model>
  </world>
</sdf>
```

## Update Your Flight Controller to Use New Topics

### Edit `platform/sim/sim_main.cpp`:

Change the IMU topic subscription:
```cpp
// For simple model:
gazebo_bridge::subscribe_imu("/quadcopter/imu")

// For PX4 x500:
gazebo_bridge::subscribe_imu("/x500/imu")

// For Fuel model:
gazebo_bridge::subscribe_imu("/quadcopter/imu")
```

## Quick Test

### Test with simple inline model:

```bash
cd /Users/habibourakash/SWE/minzaniX_1.0
gz sim sim_world/simple_quad_world.sdf
```

### In another terminal:
```bash
# Check if IMU topic exists
gz topic -l | grep imu

# Monitor IMU
gz topic -e -t /quadcopter/imu
```

### Run your FC:
Update the topic name in sim_main.cpp, rebuild, and run:
```bash
./build.sh
cd build
./minzanix_sim --gazebo
```

## Troubleshooting Transparent Window

The transparent window issue is usually:
1. **Missing render engine**: Add `<render_engine>ogre2</render_engine>` to sensors plugin
2. **Graphics driver issue**: Try `gz sim -g` for GUI only
3. **macOS specific**: May need to run with `--render-engine ogre` or `--render-engine ogre2`

Try:
```bash
gz sim --render-engine ogre2 sim_world/simple_quad_world.sdf
```

Would you like me to create the simple working model file for you now?
