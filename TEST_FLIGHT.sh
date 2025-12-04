#!/bin/bash
# Final working script for autonomous 10m takeoff

echo "=============================================="
echo "  minzaniX - Autonomous 10m Takeoff"
echo "=============================================="
echo ""

PROJECT_DIR="/Users/habibourakash/SWE/minzaniX_1.0"

# Kill any existing processes
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "minzanix_sim" 2>/dev/null
sleep 1

echo "Step 1: Starting Gazebo simulation..."
gz sim -s -r "$PROJECT_DIR/sim_world/motor_world.sdf" > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!

echo "Step 2: Waiting for Gazebo to initialize..."
sleep 5

echo "Step 3: Checking if sensors are publishing..."
/tmp/check_sensors.sh || {
    echo "❌ Sensors not ready. Check /tmp/gazebo.log for errors"
    kill $GAZEBO_PID 2>/dev/null
    exit 1
}

echo ""
echo "Step 4: Starting flight controller..."
echo "=============================================="
cd "$PROJECT_DIR/build"
./minzanix_sim --gazebo --auto-takeoff

# Cleanup on exit
kill $GAZEBO_PID 2>/dev/null
