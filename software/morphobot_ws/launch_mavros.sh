#!/bin/bash
# Launch MAVROS connected to PX4
# Adjust fcu_url based on your connection type

source /opt/ros/humble/setup.bash

echo "Launching MAVROS..."
echo "Connection: /dev/ttyACM0 @ 921600 baud"
echo "Press Ctrl+C to stop"
echo ""

ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:921600

# Alternative connection options (uncomment as needed):
# USB: fcu_url:=/dev/ttyACM0:921600
# Telemetry: fcu_url:=/dev/ttyUSB0:57600
# UDP (SITL): fcu_url:=udp://:14540@127.0.0.1:14557
