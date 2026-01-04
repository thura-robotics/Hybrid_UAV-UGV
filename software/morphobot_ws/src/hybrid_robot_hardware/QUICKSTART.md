# Quick Start Guide - C++ Hardware Interface

## Setup (One-time)

```bash
cd ~/Hybrid_UAV-UGV/software/morphobot_ws
source setup_hardware_interface.sh
```

This configures the Python path for your st3215 library.

## Launch the System

```bash
# Make sure servos are connected to /dev/ttyUSB0
ros2 launch hybrid_robot_hardware hybrid_robot_control.launch.py
```

## Test Commands

### Check Controllers
```bash
ros2 control list_controllers
```

### Monitor Joint States
```bash
ros2 topic echo /joint_states
```

### Send Position Commands
```bash
# Method 1: Use test script
ros2 run hybrid_robot_hardware test_position_commands.py

# Method 2: Manual command (positions in radians)
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.57, 0.0, 1.57]"
```

## Integration with Your Python Code

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        
        # Publisher for position commands
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        # Subscriber for joint states
        self.state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
    
    def send_positions(self, positions):
        msg = Float64MultiArray()
        msg.data = positions  # [joint1, joint2, joint3] in radians
        self.cmd_pub.publish(msg)
    
    def joint_state_callback(self, msg):
        # Receive current positions
        for name, pos in zip(msg.name, msg.position):
            print(f"{name}: {pos:.3f} rad")
```

## Troubleshooting

**Python import error:**
```bash
export PYTHONPATH=$PYTHONPATH:/home/eisan/Hybrid_UAV-UGV/software/st3215_driver
```

**Serial port permission:**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Different serial port:**
```bash
ros2 launch hybrid_robot_hardware hybrid_robot_control.launch.py serial_port:=/dev/ttyUSB1
```
