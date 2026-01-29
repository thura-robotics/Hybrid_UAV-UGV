#!/usr/bin/env python3
"""
Mode Manager Node - Top-level state machine for hybrid robot.

States:
- CRAWL: Ground movement
- MORPH_START: Begin morphing sequence
- MORPH_UPRIGHT: Morphing to upright position
- MORPH_QUADROTOR: Morphing to quadrotor configuration
- TAKEOFF: Transition to flight
- FLY: Aerial operation
- LAND: Landing sequence
- STOP: Emergency stop
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManagerNode(Node):
    """Mode manager - orchestrates robot state transitions."""
    
    # Define robot states
    CRAWL = "CRAWL"
    MORPH_START = "MORPH_START"
    MORPH_UPRIGHT = "MORPH_UPRIGHT"
    MORPH_QUADROTOR = "MORPH_QUADROTOR"
    TAKEOFF = "TAKEOFF"
    FLY = "FLY"
    LAND = "LAND"
    STOP = "STOP"
    
    def __init__(self):
        super().__init__('mode_manager_node')
        
        # Current state
        self.current_mode = self.STOP
        
        # Publisher for current mode
        self.mode_pub = self.create_publisher(String, '/robot_mode', 10)
        
        # Timer to publish mode periodically
        self.timer = self.create_timer(0.1, self.publish_mode)  # 10 Hz
        
        # TODO: Add service or topic to receive mode change requests
        # self.mode_request_sub = self.create_subscription(...)
        
        self.get_logger().info('Mode Manager initialized')
        self.get_logger().info(f'Current mode: {self.current_mode}')
    
    def publish_mode(self):
        """Publish current robot mode."""
        msg = String()
        msg.data = self.current_mode
        self.mode_pub.publish(msg)
    
    def change_mode(self, new_mode):
        """
        Change robot mode with validation.
        
        TODO: Add state transition validation
        - Check if transition is valid
        - Ensure prerequisites are met
        - Handle transition logic
        """
        if self.is_valid_transition(self.current_mode, new_mode):
            self.get_logger().info(f'Mode transition: {self.current_mode} -> {new_mode}')
            self.current_mode = new_mode
        else:
            self.get_logger().warn(f'Invalid transition: {self.current_mode} -> {new_mode}')
    
    def is_valid_transition(self, from_mode, to_mode):
        """
        Validate state transitions.
        
        TODO: Implement proper state machine logic
        Example valid transitions:
        - CRAWL -> MORPH_START
        - MORPH_QUADROTOR -> TAKEOFF
        - FLY -> LAND
        - Any -> STOP (emergency)
        """
        # For now, allow all transitions
        # You should implement proper validation logic here
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
