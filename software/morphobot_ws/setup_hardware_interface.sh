#!/bin/bash
# Setup script to configure environment for hybrid robot hardware interface

# Add st3215 library to Python path
export PYTHONPATH=$PYTHONPATH:/home/eisan/Hybrid_UAV-UGV/software/st3215_driver

# Source ROS 2 workspace
source /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws/install/setup.bash

echo "Environment configured!"
echo "PYTHONPATH includes: /home/eisan/Hybrid_UAV-UGV/software/st3215_driver"
echo ""
echo "You can now run:"
echo "  ros2 launch hybrid_robot_hardware hybrid_robot_control.launch.py"
