#!/bin/bash
# Start Gazebo and Flight Controller together

set -e

echo "======================================"
echo "  minzaniX Gazebo Simulation Launcher"
echo "======================================"
echo ""

# Set resource path
export GZ_SIM_RESOURCE_PATH=/Users/habibourakash/SWE/minzaniX_1.0/sim_world:$GZ_SIM_RESOURCE_PATH

# Check if Gazebo is installed
if ! command -v gz &> /dev/null; then
    echo "ERROR: Gazebo not found. Install with: brew install gz-harmonic"
    exit 1
fi

# Check if build exists
if [ ! -f "build/minzanix_sim" ]; then
    echo "ERROR: Flight controller not built. Run: ./build.sh"
    exit 1
fi

echo "Starting Gazebo Harmonic..."
echo "  World: sim_world/world.sdf"
echo ""

# Launch Gazebo GUI in background
gz sim -g sim_world/world.sdf &
GAZEBO_PID=$!

echo "Gazebo started (PID: $GAZEBO_PID)"
echo "Waiting 5 seconds for Gazebo to initialize..."
sleep 5

# Check if Gazebo topics are available
echo "Checking Gazebo topics..."
if gz topic -l | grep -q "/mini_quad/imu"; then
    echo "✓ IMU topic found"
else
    echo "⚠ Warning: IMU topic not found yet, starting FC anyway..."
fi

echo ""
echo "======================================"
echo "  Starting Flight Controller"
echo "======================================"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run flight controller with Gazebo
cd build
./minzanix_sim --gazebo

# Cleanup on exit
echo ""
echo "Stopping Gazebo..."
kill $GAZEBO_PID 2>/dev/null || true
