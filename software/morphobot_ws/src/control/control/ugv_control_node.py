#!/usr/bin/env python3
"""
UGV Control Node - Ground movement control for hybrid robot.

Subscribes to:
- /robot_mode: Current robot mode
- /cmd_vel: Velocity commands (Twist)

Calls services:
- /st3215/write_velocities: Control wheel servos
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from ugv_motor_driver.srv import WriteVelocities


class UGVControlNode(Node):
    """UGV control - converts velocity commands to wheel servo commands."""
    
    def __init__(self):
        super().__init__('ugv_control_node')
        
        # Parameters
        self.declare_parameter('wheel_servos', [3])  # Servo IDs for wheels
        self.wheel_servos = list(self.get_parameter('wheel_servos').get_parameter_value().integer_array_value)
        
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
        
        # Service client
        self.write_vel_client = self.create_client(WriteVelocities, '/st3215/write_velocities')
        
        # Wait for service
        while not self.write_vel_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /st3215/write_velocities service...')
        
        self.get_logger().info('UGV Control initialized')
        self.get_logger().info(f'Wheel servos: {self.wheel_servos}')
    
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
        velocity = int(self.current_linear_vel * 1000)  # Scale to servo units
        
        # Clamp velocity
        velocity = max(-1000, min(1000, velocity))
        
        self.send_wheel_command(velocity)
    
    def send_wheel_command(self, velocity):
        """Send velocity command to wheel servos."""
        request = WriteVelocities.Request()
        request.servo_ids = self.wheel_servos
        request.velocities = [velocity] * len(self.wheel_servos)
        
        future = self.write_vel_client.call_async(request)
        future.add_done_callback(self.velocity_response_callback)
    
    def stop_wheels(self):
        """Stop all wheel servos."""
        self.send_wheel_command(0)
        self.get_logger().info('Wheels stopped')
    
    def velocity_response_callback(self, future):
        """Handle service response."""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Velocity write failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


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
