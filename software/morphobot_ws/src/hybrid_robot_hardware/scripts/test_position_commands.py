#!/usr/bin/env python3
"""
Test script to send position commands to the hardware interface via ROS 2 Control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class PositionCommandPublisher(Node):
    def __init__(self):
        super().__init__('position_command_publisher')
        
        # Publisher for position commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        self.get_logger().info('Position command publisher started')
        self.get_logger().info('Publishing to /position_controller/commands')
    
    def send_position(self, positions):
        """
        Send position commands to all joints
        Args:
            positions: List of positions in radians [joint1, joint2, joint3]
        """
        msg = Float64MultiArray()
        msg.data = positions
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent positions: {positions}')


def main(args=None):
    rclpy.init(args=args)
    
    node = PositionCommandPublisher()
    
    try:
        # Test sequence
        positions_sequence = [
            [0.0, 0.0, 0.0],      # Center position
            [1.57, 0.0, 0.0],     # Move joint 1
            [1.57, 1.57, 0.0],    # Move joint 2
            [1.57, 1.57, 1.57],   # Move joint 3
            [0.0, 0.0, 0.0],      # Return to center
        ]
        
        node.get_logger().info('Starting position command sequence...')
        
        for i, positions in enumerate(positions_sequence):
            node.get_logger().info(f'\nStep {i+1}/{len(positions_sequence)}')
            node.send_position(positions)
            time.sleep(3)  # Wait for movement
        
        node.get_logger().info('\nSequence complete!')
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
