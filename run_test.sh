#!/bin/bash
# Run autonomous 10m takeoff test

set -e

echo "=============================================="
echo "  minzaniX - Autonomous 10m Takeoff Test"
echo "=============================================="
echo ""

# Set resource path
export GZ_SIM_RESOURCE_PATH=$PWD/sim_world:$GZ_SIM_RESOURCE_PATH

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

echo "Step 1: Starting Gazebo simulation (server mode)..."
gz sim -s sim_world/simple_world.sdf &
GAZEBO_PID=$!

echo "Step 2: Waiting for Gazebo to initialize..."
sleep 8

echo "Step 3: Playing Gazebo simulation..."
gz topic -t /world/minzanix_world/control -m gz.msgs.WorldControl -p 'pause: false'

sleep 3

echo "Step 4: Starting flight controller with auto-takeoff..."
cd build
./minzanix_sim --gazebo --auto-takeoff &
FC_PID=$!
cd ..

echo ""
echo "=============================================="
echo "  Simulation Running"
echo "=============================================="
echo ""
echo "Watch the terminal output above for:"
echo "  • [OPEN-LOOP] status messages"
echo "  • Altitude climbing to 10m"
echo "  • Motor values (should be ~0.95 during climb)"
echo ""
echo "The drone will:"
echo "  1. Calibrate sensors (3 seconds)"
echo "  2. Arm automatically"
echo "  3. Climb to 10m with fixed thrust (0.95)"
echo "  4. Hover at 10m with reduced thrust (0.85)"
echo ""
echo "Press Ctrl+C to stop simulation"
echo "=============================================="

# Wait for user interrupt
trap "echo ''; echo 'Stopping simulation...'; kill $FC_PID 2>/dev/null; kill $GAZEBO_PID 2>/dev/null; exit 0" INT

wait
