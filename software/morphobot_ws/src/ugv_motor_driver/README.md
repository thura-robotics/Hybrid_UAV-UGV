# ugv_motor_driver

ROS 2 hardware interface package for controlling ST3215 serial servos in a hybrid UAV-UGV robot. Provides low-level motor control for ground operations (wheels and morphing actuators).

## Features

- ✅ **ST3215 Servo Control**: Direct communication with ST3215 servos via serial
- ✅ **Mixed Control Modes**: Simultaneous velocity and position control
- ✅ **Service-Based Architecture**: Clean ROS 2 service interface
- ✅ **Real-Time Feedback**: Position and velocity state reading
- ✅ **Robust Communication**: Error handling and timeout management

## Package Structure

```
ugv_motor_driver/
├── config/              # Servo and controller configurations
├── include/             # C++ headers (ROS2 Control hardware interface)
├── launch/              # Launch files
├── nodes/               # Python ROS 2 nodes
│   ├── st3215_service_node.py    # Low-level servo driver
│   └── motor_driver_node.py      # High-level motor coordinator
├── src/                 # C++ source files
├── srv/                 # Service definitions
└── urdf/                # Robot description files
```

## Quick Start

### 1. Run ST3215 Service Node

```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 run ugv_motor_driver st3215_service_node.py --ros-args --params-file src/ugv_motor_driver/config/st3215_params.yaml
```

### 2. Read Servo Positions

```bash
# Read all servos
ros2 service call /st3215/read_positions ugv_motor_driver/srv/ReadPositions "{servo_ids: [1, 2, 3, 9, 10]}"

# Read single servo
ros2 service call /st3215/read_positions ugv_motor_driver/srv/ReadPositions "{servo_ids: [1]}"

# Read specific servos
ros2 service call /st3215/read_positions ugv_motor_driver/srv/ReadPositions "{servo_ids: [1, 2]}"
```

### 3. Write Servo Positions

```bash
# Move servos 1 and 9 to position 2100
ros2 service call /st3215/write_positions ugv_motor_driver/srv/WritePositions "{servo_ids: [1, 9], positions: [2100, 2100]}"

# Move to center position (2048)
ros2 service call /st3215/write_positions ugv_motor_driver/srv/WritePositions "{servo_ids: [1, 2, 9, 10], positions: [2048, 2048, 2048, 2048]}"

# Move multiple servos to different positions
ros2 service call /st3215/write_positions ugv_motor_driver/srv/WritePositions "{servo_ids: [1, 9], positions: [1800, 2200]}"
```

### 4. Write Servo Velocities

```bash
# Spin servo 3 forward at speed 100
ros2 service call /st3215/write_velocities ugv_motor_driver/srv/WriteVelocities "{servo_ids: [3], velocities: [100]}"

# Spin servo 3 backward
ros2 service call /st3215/write_velocities ugv_motor_driver/srv/WriteVelocities "{servo_ids: [3], velocities: [-100]}"

# Stop servo 3
ros2 service call /st3215/write_velocities ugv_motor_driver/srv/WriteVelocities "{servo_ids: [3], velocities: [0]}"
```

### 5. Read Servo Velocities

```bash
# Read all servo velocities
ros2 service call /st3215/read_velocities ugv_motor_driver/srv/ReadVelocities "{servo_ids: [1, 2, 3, 9, 10]}"

# Read velocity of servo 3 (velocity mode)
ros2 service call /st3215/read_velocities ugv_motor_driver/srv/ReadVelocities "{servo_ids: [3]}"
```

## Servo Configuration

Current configuration (from `config/st3215_params.yaml`):

```yaml
st3215_service_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    servo_ids: [1, 2, 3, 9, 10]
    position_servo_ids: [1, 2, 9, 10]  # Morphing actuators
    velocity_servo_ids: [3]             # Ground wheels
```

**Servo Modes:**
- **Position Mode** (servos 1, 2, 9, 10): For morphing actuators
  - Range: 0 - 4095 (2048 = center)
  - Speed: 400 (configured)
  
- **Velocity Mode** (servo 3): For ground wheels
  - Range: -1000 to 1000 (0 = stop)

## Position Reference

**ST3215 Position Values:**
- Minimum: 0
- Center: 2048
- Maximum: 4095

**Common Positions:**
- 1600 = ~70° left
- 2048 = Center (0°)
- 2400 = ~70° right

## Services

The ST3215 service node provides four ROS 2 services:

| Service | Type | Description |
|---------|------|-------------|
| `/st3215/read_positions` | `ReadPositions` | Read current servo positions |
| `/st3215/write_positions` | `WritePositions` | Write target positions to servos |
| `/st3215/read_velocities` | `ReadVelocities` | Read current servo velocities |
| `/st3215/write_velocities` | `WriteVelocities` | Write target velocities to servos |

## Repeat Movement Example

To repeat a movement sequence 4 times with delays:

```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash

for i in {1..4}; do
  echo "Movement $i of 4..."
  ros2 service call /st3215/write_positions ugv_motor_driver/srv/WritePositions "{servo_ids: [1, 9], positions: [1600, 2400]}"
  sleep 2
  ros2 service call /st3215/write_positions ugv_motor_driver/srv/WritePositions "{servo_ids: [1, 9], positions: [2100, 2100]}"
  sleep 2
done

echo "Done! Completed 4 repetitions."
```

## Build Instructions

```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
colcon build --packages-select ugv_motor_driver
source install/setup.bash
```

## Dependencies

### System Dependencies
- ROS 2 Humble
- Python 3.10+
- ST3215 servo driver library

### ROS 2 Dependencies
- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `hardware_interface` (for ROS2 Control)
- `controller_manager` (for ROS2 Control)

## Troubleshooting

### Servos Not Detected
- Check USB connection: `ls /dev/ttyUSB*`
- Verify servo IDs match configuration
- Check power supply to servos
- Ensure only one instance of the node is running

### Service Call Fails
- Make sure workspace is sourced: `source install/setup.bash`
- Verify node is running: `ros2 node list`
- Check service availability: `ros2 service list`

### Multiple Access Error
- Only run one instance of `st3215_service_node.py`
- Kill duplicate instances: `pkill -f st3215_service_node`

### Serial Port Permission Denied
```bash
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## Architecture

This package is part of a larger hybrid robot control system:

```
ugv_motor_driver/           ← This package (low-level hardware)
├── st3215_service_node.py  ← Direct servo communication
└── motor_driver_node.py    ← Routes commands to servos

ugv_control/                ← Ground movement logic (separate package)
morphing_control/           ← Morphing sequences (separate package)
```

## Related Packages

- `ugv_control` - High-level ground movement control
- `morphing_control` - Morphing state machine and sequences
- `px4_ros_com` - Flight control interface (for UAV mode)

## License

Apache 2.0