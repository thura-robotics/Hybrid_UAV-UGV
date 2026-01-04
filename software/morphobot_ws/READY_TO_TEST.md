# ✅ System Ready - Test Instructions

## The C++ hardware interface with Python service node is now built and ready!

### Launch the system:

```bash
cd ~/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/eisan/Hybrid_UAV-UGV/software/st3215_driver
ros2 launch hybrid_robot_hardware robot_control.launch.py
```

### In another terminal, verify it's working:

```bash
cd ~/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash

# Check services are available
ros2 service list | grep st3215

# Check controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states
```

### Send test commands:

```bash
# Position command (radians)
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.57, 0.0, 1.57]"
```

## Architecture

**C++ Hardware Interface** → ROS 2 Services → **Python ST3215 Node** → st3215 library → Servos

All typesupport libraries are correctly built!
