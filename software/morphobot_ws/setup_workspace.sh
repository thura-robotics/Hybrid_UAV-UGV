#!/bin/bash
# Source ROS 2 and workspace setup
# Usage: source setup_workspace.sh

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✓ ROS 2 Humble sourced"
echo "✓ Workspace sourced"
echo ""
echo "Available nodes:"
ros2 pkg executables control
