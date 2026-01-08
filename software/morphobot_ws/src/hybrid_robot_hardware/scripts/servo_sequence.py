#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class ServoSequenceController(Node):
    def __init__(self):
        super().__init__('servo_sequence_controller')
        
        # Create publisher for position commands
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        # Wait for publisher to be ready
        time.sleep(1)
        
        self.get_logger().info('Servo Sequence Controller initialized')
    
    def publish_position(self, positions, description=""):
        """Publish position command to servos"""
        msg = Float64MultiArray()
        msg.data = positions
        self.position_pub.publish(msg)
        if description:
            self.get_logger().info(f'{description}: {positions}')
    
    def run_sequence(self, delay=2.0):
        """Execute the predefined servo movement sequence"""
        self.get_logger().info('Starting servo sequence...')
        
        # Step 1: Normal - Both servos to 180°
        self.publish_position([3.1416, 3.1416], "Step 1 - Normal (180°, 180°)")
        time.sleep(delay)
        
        # Step 2: Move servo2 to 90°
        self.publish_position([3.1416, 1.5708], "Step 2 - Servo2 to 90° (180°, 90°)")
        time.sleep(delay)
        
        # Step 3: Move servo1 to 90°
        self.publish_position([1.5708, 1.5708], "Step 3 - Servo1 to 90° (90°, 90°)")
        time.sleep(delay)
        
        # Step 4: Move servo2 to 180°
        self.publish_position([1.5708, 3.1416], "Step 4 - Servo2 to 180° (90°, 180°)")
        time.sleep(delay)
        
        self.get_logger().info('Sequence completed!')


def main(args=None):
    rclpy.init(args=args)
    
    controller = ServoSequenceController()
    
    try:
        # Run the sequence with 3 second delays between movements
        controller.run_sequence(delay=3.0)
        
        # Keep node alive for a bit to ensure last message is sent
        time.sleep(1)
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
