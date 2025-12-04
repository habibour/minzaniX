#!/bin/bash

echo "=== minzaniX Simple Simulator Test ==="
echo ""

# Kill old processes
pkill -9 simple_sim.py 2>/dev/null
pkill -9 minzanix_sim 2>/dev/null
sleep 1

# Start Python simulator in background
echo "Starting simple physics simulator..."
cd /Users/habibourakash/SWE/minzaniX_1.0
python3 tools/simple_sim.py &
SIM_PID=$!

sleep 2

# Start flight controller with UDP mode
echo ""
echo "Starting flight controller..."
./build/minzanix_sim --udp --auto-takeoff

# Cleanup
kill $SIM_PID 2>/dev/null
