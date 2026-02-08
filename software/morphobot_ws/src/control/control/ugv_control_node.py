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
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist


class UGVControlNode(Node):
    """UGV control - publishes to velocity controller."""
    
    def __init__(self):
        super().__init__('ugv_control_node')
        
        # Parameters
        self.declare_parameter('max_velocity', 1000.0)  # Max servo velocity
        self.declare_parameter('velocity_scale', 1000.0)  # Scale factor for cmd_vel
        
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.velocity_scale = self.get_parameter('velocity_scale').get_parameter_value().double_value
        
        # State
        self.active = False  # Only active in CRAWL mode
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher to velocity controller (ROS2 Control)
        self.vel_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )
        
        self.get_logger().info('UGV Control initialized (ROS2 Control mode)')
        self.get_logger().info(f'Max velocity: {self.max_velocity}')
        self.get_logger().info(f'Velocity scale: {self.velocity_scale}')
    
    def mode_callback(self, msg):
        """Handle mode changes."""
        if msg.data == "CRAWL":
            if not self.active:
                self.get_logger().info('UGV Control activated (CRAWL mode)')
                self.active = True
        else:
            if self.active:
                self.get_logger().info(f'UGV Control deactivated (mode: {msg.data})')
                self.active = False
                self.stop_wheels()
    
    def cmd_vel_callback(self, msg):
        """
        Handle velocity commands.
        
        TODO: Implement proper kinematics
        - Convert Twist to wheel velocities
        - Handle differential drive
        - Apply velocity limits
        """
        if not self.active:
            return
        
        self.current_linear_vel = msg.linear.x
        self.current_angular_vel = msg.angular.z
        
        # Simple implementation: just use linear velocity for now
        # TODO: Implement proper differential drive kinematics
        velocity = self.current_linear_vel * self.velocity_scale
        
        # Clamp velocity
        velocity = max(-self.max_velocity, min(self.max_velocity, velocity))
        
        self.send_wheel_command(velocity)
    
    def send_wheel_command(self, velocity):
        """Send velocity command to ROS2 Control velocity controller."""
        msg = Float64MultiArray()
        msg.data = [float(velocity)]  # One value per joint in velocity controller
        self.vel_cmd_pub.publish(msg)
    
    def stop_wheels(self):
        """Stop all wheel servos."""
        self.send_wheel_command(0.0)
        self.get_logger().info('Wheels stopped')


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
