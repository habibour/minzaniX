#!/bin/bash

# Kill any running Gazebo instances
echo "Stopping any running Gazebo instances..."
pkill -9 gz || true
pkill -9 ruby || true
sleep 2

# Start Gazebo server in background
echo "Starting Gazebo server..."
cd /Users/habibourakash/SWE/minzaniX_1.0
gz sim -s sim_world/simple_quad_world.sdf > /tmp/gz_server.log 2>&1 &
GZ_PID=$!
echo "Gazebo server PID: $GZ_PID"

# Wait for Gazebo to start
echo "Waiting for Gazebo to initialize..."
sleep 5

# Check if IMU topic is available
echo ""
echo "Checking available topics:"
gz topic -l | grep quadcopter

echo ""
echo "Checking for IMU messages..."
gz topic -e -t /quadcopter/imu -n 1

echo ""
echo "Gazebo is ready!"
echo "Now run: cd build && ./minzanix_sim --gazebo"
echo ""
echo "To stop Gazebo: kill $GZ_PID"
