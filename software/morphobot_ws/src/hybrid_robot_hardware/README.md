# hybrid_robot_hardware

ROS 2 hardware interface package for controlling ST3215 serial servos in a hybrid UAV-UGV robot using ros2_control.

## Features

- ✅ **Mixed Control Modes**: Simultaneous velocity and position control
  - Servo 1: Velocity control (continuous rotation)
  - Servos 3 & 4: Position control (precise positioning)
- ✅ **Full ROS 2 Control Integration**: Standard ros2_control framework
- ✅ **Service-Based Architecture**: Clean separation between C++ hardware interface and Python servo driver
- ✅ **Real-Time Feedback**: Position and velocity state publishing
- ✅ **Robust Communication**: Error handling and timeout management

## Package Structure

```
hybrid_robot_hardware/
├── config/              # Controller configurations
├── doc/                 # Documentation
├── include/             # C++ headers
├── launch/              # Launch files
├── nodes/               # Python ROS 2 nodes
├── scripts/             # Test and utility scripts
├── src/                 # C++ source files
├── srv/                 # Service definitions
└── urdf/                # Robot description files
```

## Dependencies

### System Dependencies
- ROS 2 Humble
- Python 3.10+
- ST3215 servo driver library

### ROS 2 Dependencies
- `ros2_control`
- `controller_manager`
- `forward_command_controller`
- `joint_state_broadcaster`

## Installation

1. **Install ST3215 driver library**:
   ```bash
   cd /path/to/st3215_driver
   pip3 install -e .
   ```

2. **Build the package**:
   ```bash
   cd ~/Hybrid_UAV-UGV/software/morphobot_ws
   colcon build --packages-select hybrid_robot_hardware --symlink-install
   source install/setup.bash
   ```

## Quick Start

### Launch Robot Control

```bash
ros2 launch hybrid_robot_hardware robot_control.launch.py
```

This will:
- Start the ST3215 service node
- Initialize the hardware interface
- Activate joint state broadcaster
- Activate position and velocity controllers

### Control Servos

**Velocity Control (Servo 1)**:
```bash
# Start spinning at 500 ticks/s
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [500]"

# Stop
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0]"
```

**Position Control (Servos 3 & 4)**:
```bash
# Move to positions (in radians)
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.8, 1.5]"
```

### Monitor Joint States

```bash
# Continuous monitoring
ros2 topic echo /joint_states

# Single snapshot
ros2 topic echo /joint_states --once
```

### Check Controller Status

```bash
ros2 control list_controllers
```

## Configuration

### Servo Configuration
Edit `config/controllers.yaml` to modify:
- Controller update rates
- Joint assignments
- Interface types

### Hardware Parameters
Edit `urdf/hybrid_robot.urdf.xacro` to modify:
- Servo IDs
- Serial port
- Joint limits
- Command interfaces

## Architecture

### Components

1. **ST3215 Service Node** (`nodes/st3215_service_node.py`)
   - Python node wrapping ST3215 driver
   - Provides ROS 2 services for servo communication
   - Configures servos on startup

2. **Hardware Interface** (`src/st3215_hardware_interface.cpp`)
   - C++ ros2_control SystemInterface
   - Exports command and state interfaces
   - Calls ST3215 services for read/write operations

3. **Controllers**
   - `joint_state_broadcaster`: Publishes joint states
   - `position_controller`: Forward command controller for position
   - `velocity_controller`: Forward command controller for velocity

### Communication Flow

```
User Command
    ↓
Controller (position/velocity)
    ↓
Hardware Interface (C++)
    ↓
ST3215 Service (Python)
    ↓
ST3215 Driver Library
    ↓
Serial Communication → Servos
```

## Services

- `/st3215/read_positions` - Read servo positions
- `/st3215/write_positions` - Write servo positions
- `/st3215/read_velocities` - Read servo velocities
- `/st3215/write_velocities` - Write servo velocities

## Topics

### Published
- `/joint_states` - Current joint positions, velocities, efforts
- `/tf` - Transform tree
- `/robot_description` - URDF description

### Subscribed
- `/position_controller/commands` - Position commands for servos 3 & 4
- `/velocity_controller/commands` - Velocity commands for servo 1

## Testing

### Test Servo Detection
```bash
ros2 run hybrid_robot_hardware test_servo_detection.py
```

### Test Position Commands
```bash
ros2 run hybrid_robot_hardware test_position_commands.py
```

## Troubleshooting

### Servos Not Detected
- Check USB connection: `ls /dev/ttyUSB*`
- Verify servo IDs match configuration
- Check power supply to servos

### Controllers Not Activating
- Verify ST3215 service is running
- Check service availability: `ros2 service list`
- Review logs for error messages

### Communication Timeouts
- Reduce control loop rate in `controllers.yaml`
- Check serial cable quality
- Verify baud rate (1000000 for ST3215)

## Documentation

- [Technical Walkthrough](doc/TECHNICAL_WALKTHROUGH.md) - Detailed implementation guide
- [Implementation Plan](doc/implementation_plan.md) - Development roadmap

## Acknowledgments

- ST3215 servo driver library
- ROS 2 Control framework

## Build 
colcon build --packages-select hybrid_robot_hardware --symlink-install

source install/setup.bash
ros2 launch hybrid_robot_hardware robot_control.launch.py


## Quick reference for common angles:

0.0 rad = 0°
π/2 rad ≈ 1.5708 rad = 90°
π rad ≈ 3.1416 rad = 180°
2π rad ≈ 6.2832 rad = 360°

## Normal 
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "data: [3.1416, 3.1416]"
## Move servo2 to 90° 
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "data: [3.1416, 1.5708]"
## Move servo1 to 90° 
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.5708, 1.5708]"
## Move servo 2 to 180°
ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.5708, 3.1416]"

## launch the robot control system first 
cd ~/Hybrid_UAV-UGV/software/morphobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hybrid_robot_hardware robot_control.launch.py

## run the servo sequence
cd ~/Hybrid_UAV-UGV/software/morphobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hybrid_robot_hardware servo_sequence.py


cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash

# Read all servo positions
ros2 service call /st3215/read_positions hybrid_robot_hardware/srv/ReadPositions "{servo_ids: [1, 2, 3, 9, 10]}"

# Read single servo position (e.g., servo 1)
ros2 service call /st3215/read_positions hybrid_robot_hardware/srv/ReadPositions "{servo_ids: [1]}"

# Read specific servos (e.g., servos 1 and 2)
ros2 service call /st3215/read_positions hybrid_robot_hardware/srv/ReadPositions "{servo_ids: [1, 2]}"

# Move multiple servos at once (servos 1 and 2)
ros2 service call /st3215/write_positions hybrid_robot_hardware/srv/WritePositions "{servo_ids: [1, 9], positions: [2100, 2100]}"