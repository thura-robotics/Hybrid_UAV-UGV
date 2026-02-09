#!/bin/bash
# Run PX4 RC Bridge
# Bridges MAVROS RC input to simple ROS topics

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting PX4 RC Bridge..."
echo "Subscribes to: /mavros/rc/in"
echo "Publishes to: /rc/channels, /robot/mode"
echo ""

ros2 run control px4_rc_bridge.py
