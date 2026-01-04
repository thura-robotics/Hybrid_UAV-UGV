# Quick Test Instructions

## Test the system:

```bash
cd ~/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/eisan/Hybrid_UAV-UGV/software/st3215_driver

# Launch the system
ros2 launch hybrid_robot_hardware robot_control.launch.py
```

## What should happen:
1. Python service node starts and connects to servos
2. C++ hardware interface connects to Python services  
3. Controllers spawn successfully

## Check it's working:
```bash
# In another terminal
source install/setup.bash
ros2 service list | grep st3215
ros2 control list_controllers
ros2 topic echo /joint_states
```

## Send test command:
```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.57, 0.0, 1.57]"
```
