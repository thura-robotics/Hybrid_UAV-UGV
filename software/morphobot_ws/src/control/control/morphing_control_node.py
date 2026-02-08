#!/usr/bin/env python3
"""
Morphing Control Node - Manages morphing sequences using ROS2 Control.

Subscribes to:
- /robot_mode: Current robot mode

Publishes to:
- /position_controller/commands: Commands to ROS2 Control position controller
- /morphing_state: Current morphing state and progress
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import math


class MorphingControlNode(Node):
    """Morphing control - publishes to position controller."""
    
    # Morphing states
    STATE_IDLE = "IDLE"
    STATE_MORPHING = "MORPHING"
    STATE_COMPLETE = "COMPLETE"
    
    def __init__(self):
        super().__init__('morphing_control_node')
        
        # Morphing sequences (positions in ST3215 ticks: 0-4095, center=2048)
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
        
        # Publisher to position controller (ROS2 Control)
        self.pos_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        self.get_logger().info('Morphing Control initialized (ROS2 Control mode)')
        self.get_logger().info(f'Available sequences: {list(self.sequences.keys())}')
    
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
        
        positions_ticks = self.sequences[mode]
        
        # Convert ST3215 ticks to radians for ROS2 Control
        msg = Float64MultiArray()
        msg.data = [self.ticks_to_radians(p) for p in positions_ticks]
        
        # Publish to position controller
        self.pos_cmd_pub.publish(msg)
        
        self.get_logger().info(f'Morphing command sent: {positions_ticks} ticks')
        self.get_logger().info(f'Converted to radians: {msg.data}')
        
        # Mark as complete (in reality, you'd wait for feedback)
        # TODO: Add feedback from joint_states to verify completion
        self.morphing_state = self.STATE_COMPLETE
        self.publish_state()
    
    def ticks_to_radians(self, ticks):
        """
        Convert ST3215 position ticks (0-4095) to radians.
        
        ST3215 range: 0-4095 ticks
        Center: 2048 ticks = 0 radians
        Full range: 4096 ticks ≈ 2π radians (360°)
        """
        # Normalize around center (2048)
        normalized = ticks - 2048
        # Convert to radians
        radians = normalized * (2 * math.pi / 4096)
        return radians
    
    def radians_to_ticks(self, radians):
        """Convert radians back to ST3215 ticks."""
        normalized = radians * (4096 / (2 * math.pi))
        ticks = int(normalized + 2048)
        return max(0, min(4095, ticks))  # Clamp to valid range
    
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
