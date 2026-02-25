# Hybrid Robot Workspace - Quick Start

## Setup 

Build the workspace:
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source /opt/ros/humble/setup.bash
colcon build
```



### Terminal 1: Launch MAVROS
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
./launch_mavros.sh
```

### Terminal 2: Run PX4 RC Bridge
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
./run_px4_bridge.sh
```

### Terminal 3: Run Your Control Nodes
```bash

# Msource /opt/ros/humble/setup.bash
source ~/Hybrid_UAV-UGV/software/morphobot_ws/install/setup.bash
ros2 run control ugv_control_node


# UGV control
ros2 run control ugv_control_node

# Morphing control
ros2 run control morphing_control_node
```

## Monitoring

### Check PX4 RC Input (Raw PWM)
```bash
source setup_workspace.sh
ros2 topic echo /mavros/rc/in
```

### Check Normalized RC Channels
```bash
source setup_workspace.sh
ros2 topic echo /rc/channels
```

### Check Robot Mode
```bash
source setup_workspace.sh
ros2 topic echo /robot/mode
```

### Check Available Topics
```bash
source setup_workspace.sh
ros2 topic list
```

## Connection Settings

Edit `launch_mavros.sh` to change PX4 connection:
- **USB**: `fcu_url:=/dev/ttyACM0:921600`
- **Telemetry**: `fcu_url:=/dev/ttyUSB1:57600`
- **Simulation**: `fcu_url:=udp://:14540@127.0.0.1:14557`

## PX4 RC Bridge

The `px4_rc_bridge` node converts MAVROS RC input to simple ROS topics:

**Input:**
- `/mavros/rc/in` - Raw PWM values (1000-2000)

**Output:**
- `/rc/channels` - Normalized values (-1.0 to 1.0)
- `/robot/mode` - Current mode (UAV/UGV/MORPH) based on channel 5

**Mode Detection:**
- Channel 5 < 1300 → UAV mode
- Channel 5 > 1700 → MORPH mode
- 1300 ≤ Channel 5 ≤ 1700 → UGV mode

ls /dev/ttyUSB*

# List controllers (not 'ros2 controller')
ros2 control list_controllers

# List hardware interfaces
ros2 control list_hardware_interfaces


ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [3.11, 3.23, 3.23, 3.32, 3.23, 3.23, 3.04, 3.39]"

ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [3.03, 3.14, 3.14, 3.23, 3.14, 3.14, 2.95, 3.39]"


ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [3.19, 3.37, 3.40, 3.49, 3.40, 3.39, 3.53, 3.39]"






ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [3.19, 1.57, 3.40, 5, 3.40, 5, 2, 3.39]"

ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray   "data: [3.19, 1.57, 3.40,4.71, 3.40, 5, 1.57, 3.39]"


ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [2.95, 2.95, 2.95, 2.95, 2.95, 2.95]"

1200	−1.30 rad
1800	−0.38 rad
2048	0 rad (center)
2100	+0.08 rad
2700	+1.00 rad

ros2 launch ugv_motor_driver robot_control.launch.py