#!/usr/bin/env python3
"""
Morphing Control Node - Manages servo position sequences for robot transformation.

Subscribes to:
- /robot/mode: Current robot mode (0=UAV, 1=MORPH, 2=UGV)

Publishes to:
- /position_controller/commands: Commands to ROS2 Control position controller (servos 1, 2, 5, 6)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
import math

class MorphingControlNode(Node):
    """Morphing control - moves servos to specific positions based on mode."""
    
    def __init__(self):
        super().__init__('morphing_control_node')
        
        # Constants
        self.IDLE_TICKS = 2095
        self.MORPH_SERVO_1_TICKS = 3000
        self.MORPH_SERVO_5_TICKS = 1000
        
        # Current state
        self.current_mode = -1  # Unknown
        
        # Subscribe to mode from px4_rc_bridge (/robot/mode → [0=UAV, 1=MORPH, 2=UGV])
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/robot/mode',
            self.mode_callback,
            10
        )
        
        # Publisher to position controller (ROS2 Control) — [servo_1, servo_2, servo_5, servo_6]
        self.pos_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )
        
        self.get_logger().info('Morphing Control Node started')
        self.get_logger().info('Waiting for /robot/mode messages...')

    def ticks_to_radians(self, ticks):
        """Convert ST3215 ticks (0-4095) to radians (0-2π)."""
        # Based on ST3215 documentation/hardware interface: 4096 ticks = 2π radians
        return (float(ticks) / 4096.0) * 2.0 * math.pi

    def mode_callback(self, msg: Float32MultiArray):
        """Process mode changes and publish servo positions."""
        if len(msg.data) < 1:
            return
            
        new_mode = int(msg.data[0])
        
        # Only act if mode has changed
        if new_mode == self.current_mode:
            return
            
        self.current_mode = new_mode
        mode_names = {0: "UAV", 1: "MORPH", 2: "UGV"}
        mode_name = mode_names.get(new_mode, "UNKNOWN")
        
        self.get_logger().info(f'Detected Mode Change: {mode_name} ({new_mode})')
        
        # Determine target positions (in radians)
        targets_radians = []
        
        if new_mode == 1:  # MORPH
            # Servo 1 -> 3000, Servo 5 -> 1500, Others -> 2095
            targets_radians.append(self.ticks_to_radians(self.MORPH_SERVO_1_TICKS)) # servo_1
            targets_radians.append(self.ticks_to_radians(self.IDLE_TICKS))          # servo_2
            targets_radians.append(self.ticks_to_radians(self.MORPH_SERVO_5_TICKS)) # servo_5
            targets_radians.append(self.ticks_to_radians(self.IDLE_TICKS))          # servo_6
            self.get_logger().info(f'Transitioning to MORPH positions: S1={self.MORPH_SERVO_1_TICKS}, S5={self.MORPH_SERVO_5_TICKS}')
        else:
            # All to IDLE (2095)
            targets_radians = [self.ticks_to_radians(self.IDLE_TICKS)] * 4
            self.get_logger().info(f'Transitioning to IDLE position: {self.IDLE_TICKS} ticks')
            
        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = targets_radians
        self.pos_cmd_pub.publish(cmd_msg)
        
        # Debug output in ticks for user reference
        self.get_logger().info(f'Published positions to /position_controller/commands')


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
