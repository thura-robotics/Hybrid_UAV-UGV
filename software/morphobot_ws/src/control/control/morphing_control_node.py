#!/usr/bin/env python3
"""
Morphing Control Node - Manages morphing sequences for hybrid robot.

Subscribes to:
- /robot_mode: Current robot mode

Calls services:
- /st3215/write_positions: Control morphing servos

Publishes:
- /morphing_state: Current morphing state and progress
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ugv_motor_driver.srv import WritePositions
import time


class MorphingControlNode(Node):
    """Morphing control - executes morphing sequences based on mode."""
    
    # Morphing states
    STATE_IDLE = "IDLE"
    STATE_MORPHING = "MORPHING"
    STATE_COMPLETE = "COMPLETE"
    
    def __init__(self):
        super().__init__('morphing_control_node')
        
        # Parameters
        self.declare_parameter('morphing_servos', [1, 2, 9, 10])
        self.morphing_servos = list(self.get_parameter('morphing_servos').get_parameter_value().integer_array_value)
        
        # Morphing sequences (positions for each configuration)
        # TODO: Load from config file
        self.sequences = {
            "MORPH_START": [2048, 2048, 2048, 2048],      # Start position
            "MORPH_UPRIGHT": [2048, 1500, 2048, 2500],    # Upright configuration
            "MORPH_QUADROTOR": [1500, 1500, 2500, 2500],  # Quadrotor configuration
        }
        
        # State
        self.morphing_state = self.STATE_IDLE
        self.current_mode = None
        
        # Subscribers
        self.mode_sub = self.create_subscription(
            String,
            '/robot_mode',
            self.mode_callback,
            10
        )
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/morphing_state', 10)
        
        # Service client
        self.write_pos_client = self.create_client(WritePositions, '/st3215/write_positions')
        
        # Wait for service
        while not self.write_pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /st3215/write_positions service...')
        
        self.get_logger().info('Morphing Control initialized')
        self.get_logger().info(f'Morphing servos: {self.morphing_servos}')
    
    def mode_callback(self, msg):
        """Handle mode changes and trigger morphing sequences."""
        mode = msg.data
        
        # Check if this is a morphing mode
        if mode in self.sequences:
            if self.current_mode != mode:
                self.get_logger().info(f'Executing morphing sequence: {mode}')
                self.current_mode = mode
                self.execute_morphing(mode)
        else:
            self.current_mode = mode
    
    def execute_morphing(self, mode):
        """
        Execute morphing sequence for given mode.
        
        TODO: Add smooth interpolation between positions
        TODO: Add progress tracking
        TODO: Add error handling
        """
        if mode not in self.sequences:
            self.get_logger().warn(f'No sequence defined for mode: {mode}')
            return
        
        self.morphing_state = self.STATE_MORPHING
        self.publish_state()
        
        positions = self.sequences[mode]
        
        # Send position command
        request = WritePositions.Request()
        request.servo_ids = self.morphing_servos
        request.positions = positions
        
        future = self.write_pos_client.call_async(request)
        future.add_done_callback(self.position_response_callback)
    
    def position_response_callback(self, future):
        """Handle service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Morphing sequence complete')
                self.morphing_state = self.STATE_COMPLETE
            else:
                self.get_logger().error(f'Morphing failed: {response.message}')
                self.morphing_state = self.STATE_IDLE
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.morphing_state = self.STATE_IDLE
        
        self.publish_state()
    
    def publish_state(self):
        """Publish current morphing state."""
        msg = String()
        msg.data = self.morphing_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MorphingControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
