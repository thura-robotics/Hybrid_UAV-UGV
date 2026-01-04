#!/usr/bin/env python3
"""
Quick test to verify the Python service node works independently
"""

import rclpy
from rclpy.node import Node
import sys
import os

# Add st3215 to path
sys.path.insert(0, '/home/eisan/Hybrid_UAV-UGV/software/st3215_driver')

from hybrid_robot_hardware.srv import ReadPositions, WritePositions

class ServiceTester(Node):
    def __init__(self):
        super().__init__('service_tester')
        
        # Create clients
        self.read_client = self.create_client(ReadPositions, 'st3215/read_positions')
        self.write_client = self.create_client(WritePositions, 'st3215/write_positions')
        
        self.get_logger().info('Waiting for services...')
        
        if not self.read_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Read service not available!')
            return
        
        if not self.write_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Write service not available!')
            return
        
        self.get_logger().info('Services available! Testing...')
        self.test_read()
    
    def test_read(self):
        request = ReadPositions.Request()
        request.servo_ids = [1, 3, 4]
        
        future = self.read_client.call_async(request)
        future.add_done_callback(self.read_callback)
    
    def read_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Read success: {response.success}')
            self.get_logger().info(f'Positions: {response.positions}')
            self.get_logger().info(f'Message: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main():
    rclpy.init()
    node = ServiceTester()
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
