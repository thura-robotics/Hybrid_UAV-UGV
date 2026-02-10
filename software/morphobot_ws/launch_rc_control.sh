#!/bin/bash
# Launch RC control for UGV with new servo configuration (servos 3 and 12)

cd "$(dirname "$0")"

echo "=========================================="
echo "  UGV RC Control Launcher"
echo "=========================================="
echo ""
echo "Prerequisites:"
echo "  1. ros2_control_node must be running"
echo "  2. MAVROS must be running (./launch_mavros.sh)"
echo ""
echo "This will start:"
echo "  - PX4 RC Bridge (reads RC from MAVROS)"
echo "  - UGV Motor Controller (controls servos 3 & 12)"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash
source install/setup.bash

# Check if required topics exist
echo "Checking prerequisites..."

if ! ros2 topic list | grep -q "/mavros/rc/in"; then
    echo "❌ ERROR: MAVROS not running!"
    echo "   Please run: ./launch_mavros.sh"
    exit 1
fi

if ! ros2 topic list | grep -q "/velocity_controller/commands"; then
    echo "❌ ERROR: ROS2 Control not running!"
    echo "   Please run: ros2 launch ugv_motor_driver robot_control.launch.py"
    exit 1
fi

echo "✓ MAVROS detected"
echo "✓ ROS2 Control detected"
echo ""

# Start PX4 RC Bridge
echo "Starting PX4 RC Bridge..."
ros2 run control px4_rc_bridge.py &
PX4_PID=$!
sleep 2

# Start UGV Motor Controller  
echo "Starting UGV Motor Controller..."
ros2 run control ugv_motor_controller.py &
UGV_PID=$!
sleep 1

echo ""
echo "=========================================="
echo "✓ RC Control System Running"
echo "=========================================="
echo "  PX4 RC Bridge:        PID $PX4_PID"
echo "  UGV Motor Controller: PID $UGV_PID"
echo ""
echo "Monitoring topics:"
echo "  /mavros/rc/in          → RC input from PX4"
echo "  /ugv/motor_commands    → Motor commands"
echo "  /velocity_controller/commands → Servo commands"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

# Wait for both processes
wait
