#!/bin/bash
# ─────────────────────────────────────────────────
# Morphobot Full System Bringup
# Launches: MAVROS, PX4 RC Bridge, ROS2 Control,
#           UGV Control, and Morphing Control
# ─────────────────────────────────────────────────

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash

# ─────────────────────────────────────────────────
# Port Configuration
# Change these values if your devices are on different ports
# ─────────────────────────────────────────────────
FCU_URL="/dev/ttyACM0:921600"
SERIAL_PORT="/dev/ttyUSB0"

echo "╔══════════════════════════════════════════╗"
echo "║     MORPHOBOT FULL SYSTEM BRINGUP       ║"
echo "╠══════════════════════════════════════════╣"
echo "║  1. MAVROS         ($FCU_URL)       ║"
echo "║  2. PX4 RC Bridge                       ║"
echo "║  3. ROS2 Control   ($SERIAL_PORT)       ║"
echo "║  4. UGV Control Node                    ║"
echo "║  5. Morphing Control Node               ║"
echo "╚══════════════════════════════════════════╝"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

ros2 launch control morphobot_bringup.launch.py fcu_url:=$FCU_URL serial_port:=$SERIAL_PORT
