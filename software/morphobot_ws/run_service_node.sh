#!/bin/bash
# Run ST3215 Service Node

cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 run ugv_motor_driver st3215_service_node.py \
  --ros-args --params-file src/ugv_motor_driver/config/st3215_params.yaml
