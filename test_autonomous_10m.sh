#!/bin/bash
# Complete test script for autonomous takeoff to 10m

set -e

cd /Users/habibourakash/SWE/minzaniX_1.0

echo "=============================================="
echo "  minzaniX Autonomous 10m Takeoff Test"
echo "=============================================="
echo

# Kill any existing processes
echo "[1/5] Cleaning up existing processes..."
pkill -f "gz sim" 2>/dev/null || true
pkill -f "minzanix_sim" 2>/dev/null || true
sleep 2

# Start Gazebo in server mode
echo "[2/5] Starting Gazebo simulation..."
export GZ_SIM_RESOURCE_PATH=$PWD/sim_world:$GZ_SIM_RESOURCE_PATH
gz sim -s sim_world/simple_world.sdf > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
echo "  Gazebo PID: $GAZEBO_PID"
sleep 5

# Start the simulation (unpause it)
echo "  Starting simulation playback..."
gz service -s /world/minzanix_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2000 --req 'pause: false' > /dev/null 2>&1
sleep 2

# Start flight controller
echo "[3/5] Starting flight controller..."
cd build
./minzanix_sim --gazebo > /tmp/fc.log 2>&1 &
FC_PID=$!
cd ..
echo "  Flight controller PID: $FC_PID"
sleep 3

# Check if FC is still running
if ! ps -p $FC_PID > /dev/null; then
    echo "ERROR: Flight controller failed to start"
    echo "Check /tmp/fc.log for details"
    cat /tmp/fc.log
    kill $GAZEBO_PID 2>/dev/null || true
    exit 1
fi

# Run autonomous test
echo "[4/5] Running autonomous takeoff test..."
echo
/Users/habibourakash/SWE/minzaniX_1.0/.venv/bin/python test_10m_takeoff.py

# Wait before cleanup
echo
echo "[5/5] Test complete. Waiting 5 seconds before cleanup..."
sleep 5

# Cleanup
echo "Cleaning up..."
kill $FC_PID 2>/dev/null || true
kill $GAZEBO_PID 2>/dev/null || true

echo
echo "=============================================="
echo "  Test Complete!"
echo "=============================================="
echo
echo "Logs available at:"
echo "  - /tmp/gazebo.log"
echo "  - /tmp/fc.log"
