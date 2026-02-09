#!/usr/bin/env python3
"""
UGV Motor Controller Node
Subscribes to /ugv/motor_commands and controls ST3215 servos 4 and 10 for UGV movement
Uses velocity commands: Forward/Back at 1000, Turn at 400
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray


class UgvMotorController(Node):
    """Controls ST3215 servos 4 and 10 based on UGV motor commands"""
    
    def __init__(self):
        super().__init__('ugv_motor_controller')
        
        # Parameters for velocity (ST3215 range: 0-3400)
        self.declare_parameter('forward_velocity', 3400.0)  # Maximum speed
        self.declare_parameter('turn_velocity', 2000.0)     # Slower for turning
        
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.turn_vel = self.get_parameter('turn_velocity').value
        
        # Subscribe to motor commands
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/ugv/motor_commands',
            self.motor_cmd_callback,
            10
        )
        
        # Publisher for velocity controller (servos 4 and 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10
        )
        
        self.get_logger().info('UGV Motor Controller started (Servos 4 and 10)')
        self.get_logger().info(f'Forward velocity: {self.forward_vel}, Turn velocity: {self.turn_vel}')
    
    def motor_cmd_callback(self, msg: Float32MultiArray):
        """
        Process motor commands
        msg.data[0] = throttle (-1.0 to 1.0)
        msg.data[1] = steering (-1.0 to 1.0)
        """
        if len(msg.data) < 2:
            self.get_logger().warn('Invalid motor command: need [throttle, steering]')
            return
        
        throttle = msg.data[0]  # -1.0 to 1.0
        steering = msg.data[1]  # -1.0 to 1.0
        
        # Calculate velocity for both servos
        # If throttle is dominant, use forward_velocity
        # If steering is dominant, use turn_velocity
        if abs(throttle) > abs(steering):
            # Forward/backward mode - both servos same speed
            velocity = throttle * self.forward_vel
            servo4_vel = velocity
            servo10_vel = velocity
        else:
            # Turning mode - both servos same speed
            velocity = steering * self.turn_vel
            servo4_vel = velocity
            servo10_vel = velocity
        
        # Publish velocity commands for both servos [servo_4, servo_10]
        velocity_cmd = Float64MultiArray()
        velocity_cmd.data = [float(servo4_vel), float(servo10_vel)]
        self.velocity_pub.publish(velocity_cmd)
        
        mode = "FORWARD" if abs(throttle) > abs(steering) else "TURN"
        self.get_logger().info(
            f'🚗 [{mode}] Throttle={throttle:.2f}, Steering={steering:.2f} → '
            f'Servo4={servo4_vel:.0f}, Servo10={servo10_vel:.0f}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = UgvMotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
