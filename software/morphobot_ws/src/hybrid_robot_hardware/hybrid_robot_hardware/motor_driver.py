#!/usr/bin/env python3
"""
Motor driver node for ST3215 servos with ROS 2 command topics.
Publishes joint states and accepts position/velocity commands.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64MultiArray
import time

try:
    from st3215 import ST3215
except ImportError:
    print("ERROR: st3215 library not found. Install with: pip3 install -e /path/to/st3215_driver")
    raise


class ST3215DriverNode(Node):
    """ROS 2 node for ST3215 servo control with command topics."""
    
    def __init__(self):
        super().__init__('st3215_driver_node')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('servo_ids', [1, 3, 4])  # Default servo IDs
        
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.servo_ids = self.get_parameter('servo_ids').get_parameter_value().integer_array_value
        
        # Command storage
        self.position_commands = {}  # {servo_id: position_in_ticks}
        self.velocity_commands = {}  # {servo_id: velocity}
        
        # Initialize ST3215 driver
        try:
            self.servo = ST3215(port)
            self.get_logger().info(f'Connected to ST3215 servos on {port}')
            
            # Detect servos
            detected = self.servo.ListServos()
            self.get_logger().info(f'Detected servos: {detected}')
            
            # Filter to only use detected servos
            self.servo_ids = [sid for sid in self.servo_ids if sid in detected]
            if not self.servo_ids:
                self.get_logger().error('No configured servos detected!')
                raise RuntimeError('No servos found')
                
            self.get_logger().info(f'Using servos: {self.servo_ids}')
            
            # Configure servos for position mode by default
            for sid in self.servo_ids:
                self.servo.SetMode(sid, 0)  # Position mode
                self.servo.SetSpeed(sid, 2400)  # Fast speed
                self.servo.SetAcceleration(sid, 50)
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribers for commands
        self.position_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'servo_position_commands',
            self.position_command_callback,
            10
        )
        
        self.velocity_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'servo_velocity_commands',
            self.velocity_command_callback,
            10
        )
        
        # Timer for read/write loop
        self.timer = self.create_timer(1.0 / rate, self.control_loop)
        
        self.get_logger().info(f'ST3215 driver started at {rate} Hz')
        self.get_logger().info('Listening for commands on:')
        self.get_logger().info('  - /servo_position_commands (Float64MultiArray)')
        self.get_logger().info('  - /servo_velocity_commands (Float64MultiArray)')

    def position_command_callback(self, msg):
        """Handle position commands: [servo_id, position_ticks, servo_id, position_ticks, ...]"""
        if len(msg.data) % 2 != 0:
            self.get_logger().error('Position command must have pairs: [servo_id, position]')
            return
        
        for i in range(0, len(msg.data), 2):
            servo_id = int(msg.data[i])
            position = int(msg.data[i + 1])
            
            if servo_id in self.servo_ids:
                self.position_commands[servo_id] = position
                self.get_logger().info(f'Position command: Servo {servo_id} -> {position}')
            else:
                self.get_logger().warn(f'Servo {servo_id} not in configured servos')

    def velocity_command_callback(self, msg):
        """Handle velocity commands: [servo_id, velocity, servo_id, velocity, ...]"""
        if len(msg.data) % 2 != 0:
            self.get_logger().error('Velocity command must have pairs: [servo_id, velocity]')
            return
        
        for i in range(0, len(msg.data), 2):
            servo_id = int(msg.data[i])
            velocity = int(msg.data[i + 1])
            
            if servo_id in self.servo_ids:
                self.velocity_commands[servo_id] = velocity
                self.get_logger().info(f'Velocity command: Servo {servo_id} -> {velocity}')
            else:
                self.get_logger().warn(f'Servo {servo_id} not in configured servos')

    def control_loop(self):
        """Main control loop: write commands, read sensors, publish states."""
        
        # Write position commands
        for servo_id, position in self.position_commands.items():
            try:
                self.servo.WritePosition(servo_id, position)
            except Exception as e:
                self.get_logger().error(f'Failed to write position to servo {servo_id}: {e}')
        
        # Write velocity commands
        for servo_id, velocity in self.velocity_commands.items():
            try:
                self.servo.Rotate(servo_id, velocity)
            except Exception as e:
                self.get_logger().error(f'Failed to write velocity to servo {servo_id}: {e}')
        
        # Read from hardware and publish states
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for sid in self.servo_ids:
            # Read position
            pos = self.servo.ReadPosition(sid)
            if pos is not None:
                # Convert to radians (0-4095 -> 0-2π)
                pos_rad = (pos / 4095.0) * 2.0 * 3.14159265359
                
                msg.name.append(f'servo_joint_{sid}')
                msg.position.append(pos_rad)
                msg.velocity.append(0.0)  # Velocity not available
                msg.effort.append(0.0)    # Effort not available
        
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ST3215DriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
