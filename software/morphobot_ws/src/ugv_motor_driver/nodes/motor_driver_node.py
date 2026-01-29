#!/usr/bin/env python3
"""
Motor Driver Node - Low-level interface to ST3215 servos.
Receives high-level commands and translates them to servo commands.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String
from ugv_motor_driver.srv import WritePositions, WriteVelocities, ReadPositions, ReadVelocities


class MotorDriverNode(Node):
    """Motor driver node for hybrid robot."""
    
    def __init__(self):
        super().__init__('motor_driver_node')
        
        # Parameters - servo mapping
        self.declare_parameter('wheel_servos', [3])  # Servos for wheels (velocity mode)
        self.declare_parameter('morphing_servos', [1, 2, 9, 10])  # Servos for morphing (position mode)
        
        self.wheel_servos = self.get_parameter('wheel_servos').get_parameter_value().integer_array_value
        self.morphing_servos = self.get_parameter('morphing_servos').get_parameter_value().integer_array_value
        
        # Current mode
        self.current_mode = "GROUND"  # GROUND, FLIGHT, MORPHING
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )
        
        self.wheel_cmd_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_commands',
            self.wheel_command_callback,
            10
        )
        
        self.morphing_cmd_sub = self.create_subscription(
            Int32MultiArray,
            '/morphing_commands',
            self.morphing_command_callback,
            10
        )
        
        # Service clients to ST3215 service node
        self.write_vel_client = self.create_client(WriteVelocities, '/st3215/write_velocities')
        self.write_pos_client = self.create_client(WritePositions, '/st3215/write_positions')
        self.read_pos_client = self.create_client(ReadPositions, '/st3215/read_positions')
        self.read_vel_client = self.create_client(ReadVelocities, '/st3215/read_velocities')
        
        # Wait for services
        self.get_logger().info('Waiting for ST3215 services...')
        self.write_vel_client.wait_for_service(timeout_sec=5.0)
        self.write_pos_client.wait_for_service(timeout_sec=5.0)
        
        self.get_logger().info('Motor driver node ready!')
        self.get_logger().info(f'Wheel servos: {list(self.wheel_servos)}')
        self.get_logger().info(f'Morphing servos: {list(self.morphing_servos)}')
    
    def mode_callback(self, msg):
        """Update current robot mode."""
        self.current_mode = msg.data
        self.get_logger().info(f'Mode changed to: {self.current_mode}')
        
        # Stop wheels when not in GROUND mode
        if self.current_mode != "GROUND":
            self.stop_wheels()
    
    def wheel_command_callback(self, msg):
        """Handle wheel velocity commands."""
        if self.current_mode != "GROUND":
            self.get_logger().warn('Ignoring wheel command - not in GROUND mode')
            return
        
        # msg.data format: [servo_id1, velocity1, servo_id2, velocity2, ...]
        if len(msg.data) % 2 != 0:
            self.get_logger().error('Invalid wheel command format')
            return
        
        servo_ids = []
        velocities = []
        for i in range(0, len(msg.data), 2):
            servo_ids.append(msg.data[i])
            velocities.append(msg.data[i + 1])
        
        # Send to ST3215 service
        request = WriteVelocities.Request()
        request.servo_ids = servo_ids
        request.velocities = velocities
        
        future = self.write_vel_client.call_async(request)
        future.add_done_callback(self.velocity_response_callback)
    
    def morphing_command_callback(self, msg):
        """Handle morphing position commands."""
        if self.current_mode != "MORPHING":
            self.get_logger().warn('Ignoring morphing command - not in MORPHING mode')
            return
        
        # msg.data format: [servo_id1, position1, servo_id2, position2, ...]
        if len(msg.data) % 2 != 0:
            self.get_logger().error('Invalid morphing command format')
            return
        
        servo_ids = []
        positions = []
        for i in range(0, len(msg.data), 2):
            servo_ids.append(msg.data[i])
            positions.append(msg.data[i + 1])
        
        # Send to ST3215 service
        request = WritePositions.Request()
        request.servo_ids = servo_ids
        request.positions = positions
        
        future = self.write_pos_client.call_async(request)
        future.add_done_callback(self.position_response_callback)
    
    def stop_wheels(self):
        """Stop all wheel servos."""
        request = WriteVelocities.Request()
        request.servo_ids = list(self.wheel_servos)
        request.velocities = [0] * len(self.wheel_servos)
        
        future = self.write_vel_client.call_async(request)
        self.get_logger().info('Stopping all wheels')
    
    def velocity_response_callback(self, future):
        """Handle velocity write response."""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Velocity write failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def position_response_callback(self, future):
        """Handle position write response."""
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f'Position write failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
