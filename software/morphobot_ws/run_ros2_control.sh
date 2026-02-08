#!/bin/bash
# Run ROS2 Control System (without service node)
# NOTE: Run ./run_service_node.sh in a separate terminal FIRST!

cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 launch ugv_motor_driver robot_control_only.launch.py
