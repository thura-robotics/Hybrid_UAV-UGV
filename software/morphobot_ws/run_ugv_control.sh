#!/bin/bash
# Simple script to run UGV Motor Controller after ROS2 Control is running

cd "$(dirname "$0")"

echo "Starting UGV Motor Controller..."
echo "Make sure ROS2 Control is already running!"
echo ""

source /opt/ros/humble/setup.bash
source install/setup.bash

# Run PX4 RC Bridge and UGV Motor Controller
ros2 run control px4_rc_bridge.py &
PX4_PID=$!

sleep 2

ros2 run control ugv_motor_controller &
UGV_PID=$!

echo ""
echo "✓ PX4 RC Bridge running (PID: $PX4_PID)"
echo "✓ UGV Motor Controller running (PID: $UGV_PID)"
echo ""
echo "Press Ctrl+C to stop"

wait
