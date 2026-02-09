#!/bin/bash
# Complete UGV System Launcher
# Launches MAVROS, ROS2 Control, PX4 RC Bridge, and UGV Motor Controller

cd "$(dirname "$0")"

echo "========================================="
echo "  Starting Complete UGV Control System  "
echo "========================================="
echo ""

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "✓ ROS 2 and workspace sourced"
echo ""

# Check if devices exist
if [ ! -e "/dev/ttyACM0" ]; then
    echo "❌ ERROR: /dev/ttyACM0 (PX4/Radio) not found!"
    exit 1
fi

if [ ! -e "/dev/ttyUSB0" ]; then
    echo "❌ ERROR: /dev/ttyUSB0 (Servo shield) not found!"
    exit 1
fi

echo "✓ Found /dev/ttyACM0 (PX4/Radio)"
echo "✓ Found /dev/ttyUSB0 (Servo shield)"
echo ""

# Launch everything using ros2 launch
echo "Starting all nodes..."
echo ""

# Create a temporary launch file
cat > /tmp/ugv_complete.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # MAVROS
        ExecuteProcess(
            cmd=['ros2', 'launch', 'mavros', 'px4.launch', 'fcu_url:=/dev/ttyACM0:921600'],
            output='screen',
            name='mavros'
        ),
        
        # ST3215 Service Node
        Node(
            package='ugv_motor_driver',
            executable='st3215_service_node.py',
            name='st3215_service_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baudrate': 1000000,
                'servo_ids': [4, 10]
            }]
        ),
        
        # ROS2 Control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.expanduser('~/Hybrid_UAV-UGV/software/morphobot_ws/src/ugv_motor_driver/config/st3215_controllers.yaml')],
            output='screen'
        ),
        
        # PX4 RC Bridge
        Node(
            package='control',
            executable='px4_rc_bridge.py',
            name='px4_rc_bridge',
            output='screen'
        ),
        
        # UGV Motor Controller
        Node(
            package='control',
            executable='ugv_motor_controller.py',
            name='ugv_motor_controller',
            output='screen'
        ),
    ])
EOF

# Launch the system
ros2 launch /tmp/ugv_complete.launch.py
