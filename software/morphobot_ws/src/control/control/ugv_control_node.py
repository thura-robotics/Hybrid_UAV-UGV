#!/usr/bin/env python3
"""
UGV Control Node - Ground movement control using ROS2 Control.

Subscribes to:
- /robot_mode: Current robot mode
- /cmd_vel: Velocity commands (Twist)

Publishes to:
- /velocity_controller/commands: Commands to ROS2 Control velocity controller
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class UGVControlNode(Node):
    """UGV control - converts RC commands to velocity controller commands."""
    
    def __init__(self):
        super().__init__('ugv_control_node')
        
        # Parameters
        self.declare_parameter('max_ticks', 3400)       # Max servo velocity in ticks
        self.declare_parameter('invert_right', True)    # Right motor is physically reversed
        self.declare_parameter('deadzone', 0.05)
        
        self.max_ticks = self.get_parameter('max_ticks').value
        self.invert_right = self.get_parameter('invert_right').value
        self.deadzone = self.get_parameter('deadzone').value
        
        # State
        self.ugv_active = False  # Only active when mode == 2 (UGV)
        
        # Subscribe to mode from px4_rc_bridge  (/robot/mode → [0=UAV, 1=MORPH, 2=UGV])
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/robot/mode',
            self.mode_callback,
            10
        )
        
        # Subscribe to RC motor commands from px4_rc_bridge ([throttle, steering])
        self.motor_cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/ugv/motor_commands',
            self.motor_cmd_callback,
            10
        )
        
        # Publisher to diff_drive_controller
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/diff_drive_controller/cmd_vel_unstamped',
            10
        )
        
        self.get_logger().info('UGV Control Node started')
        self.get_logger().info('Subscribing: /robot/mode + /ugv/motor_commands')
        self.get_logger().info('Publishing:  /diff_drive_controller/cmd_vel_unstamped')
        self.get_logger().info(f'Max ticks: ±{self.max_ticks}, Invert right: {self.invert_right}')
    
    def mode_callback(self, msg: Float32MultiArray):
        """Activate only in UGV mode (mode == 2.0)."""
        if len(msg.data) < 1:
            return
        mode = int(msg.data[0])
        if mode == 2 and not self.ugv_active:
            self.get_logger().info('UGV mode active — motors enabled')
            self.ugv_active = True
        elif mode != 2 and self.ugv_active:
            self.get_logger().info('Left UGV mode — stopping motors')
            self.ugv_active = False
            self.stop_wheels()
    
    def motor_cmd_callback(self, msg: Float32MultiArray):
        """
        Convert RC [throttle, steering] into geometry_msgs/Twist for diff_drive_controller.
        Only processes commands when in UGV mode.
        """
        if not self.ugv_active or len(msg.data) < 2:
            return
        
        throttle = float(msg.data[0])
        steering = float(msg.data[1])
        
        # Apply deadzone
        if abs(throttle) < self.deadzone:
            throttle = 0.0
        if abs(steering) < self.deadzone:
            steering = 0.0
        
        # Max velocity conversion approximations
        # max_ticks: integer ticks mapped to max physical speed (m/s and rad/s)
        # diff_drive_controller takes linear.x (m/s) and angular.z (rad/s)
        
        # Scaling joystick (+-1.0 limits) to theoretical limits
        max_linear_speed = 1.0  # m/s target at full throttle
        max_angular_speed = 3.0 # rad/s target at full steering
        
        twist = Twist()
        twist.linear.x = throttle * max_linear_speed
        
        # If invert_right is true, we might need to adjust the angular direction.
        # But diff_drive_controller handles wheel inversion internally based on URDF joint definitions.
        twist.angular.z = steering * max_angular_speed
        
        self.cmd_vel_pub.publish(twist)
        
        self.get_logger().info(
            f'T:{throttle:.2f} S:{steering:.2f} → Vx:{twist.linear.x:.2f} Wz:{twist.angular.z:.2f}',
            throttle_duration_sec=1.0
        )
    
    def stop_wheels(self):
        """Stop diff_drive_controller."""
        twist = Twist() # defaults to 0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Wheels stopped (Twist = 0)')


def main(args=None):
    rclpy.init(args=args)
    node = UGVControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
