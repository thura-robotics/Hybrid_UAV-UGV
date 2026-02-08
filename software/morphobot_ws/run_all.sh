#!/bin/bash
# Run EVERYTHING in a single terminal
# This launches both the service node AND ROS2 Control together

cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 launch ugv_motor_driver robot_control.launch.py
