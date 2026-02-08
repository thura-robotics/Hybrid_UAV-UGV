#!/bin/bash
# Run all control nodes

cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash

# Run all control nodes in background
ros2 run control mode_manager_node &
MODE_PID=$!

ros2 run control morphing_control_node &
MORPH_PID=$!

ros2 run control ugv_control_node &
UGV_PID=$!

echo "Control nodes started:"
echo "  Mode Manager: PID $MODE_PID"
echo "  Morphing Control: PID $MORPH_PID"
echo "  UGV Control: PID $UGV_PID"
echo ""
echo "Press Ctrl+C to stop all nodes"

# Wait for Ctrl+C
trap "kill $MODE_PID $MORPH_PID $UGV_PID; exit" INT
wait
