#!/usr/bin/env python3
"""
PX4 RC Bridge Node
Subscribes to MAVROS RC input and republishes as simple normalized values
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
from std_msgs.msg import Float32MultiArray


class Px4RcBridge(Node):
    """Bridge between PX4/MAVROS RC input and simple ROS topics"""
    
    def __init__(self):
        super().__init__('px4_rc_bridge')
        
        # Parameters
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)
        self.declare_parameter('pwm_center', 1500)
        self.declare_parameter('mode_channel', 4)  # Channel 5 (0-indexed)
        self.declare_parameter('mode_uav_threshold', 1300)
        self.declare_parameter('mode_morph_threshold', 1700)
        
        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.pwm_center = self.get_parameter('pwm_center').value
        self.mode_channel = self.get_parameter('mode_channel').value
        self.mode_uav_threshold = self.get_parameter('mode_uav_threshold').value
        self.mode_morph_threshold = self.get_parameter('mode_morph_threshold').value
        
        # Current mode (0=UAV, 1=MORPH, 2=UGV)
        self.current_mode = 1  # Start with MORPH
        
        # Subscriber to MAVROS RC input
        self.rc_sub = self.create_subscription(
            RCIn,
            '/mavros/rc/in',
            self.rc_callback,
            10
        )
        
        # Publishers
        self.rc_channels_pub = self.create_publisher(
            Float32MultiArray,
            '/rc/channels',
            10
        )
        
        self.mode_pub = self.create_publisher(
            Float32MultiArray,  # Will publish [mode] as single element array
            '/robot/mode',
            10
        )
        
        # Publisher for UGV motor commands (only used in mode 2)
        self.ugv_motor_pub = self.create_publisher(
            Float32MultiArray,
            '/ugv/motor_commands',
            10
        )
        
        self.get_logger().info('PX4 RC Bridge started')
        self.get_logger().info(f'Mode channel: {self.mode_channel + 1} (1-indexed)')
        self.get_logger().info(f'Thresholds: UAV<{self.mode_uav_threshold}, MORPH>{self.mode_morph_threshold}')
    
    def normalize_pwm(self, pwm_value):
        """Normalize PWM value from [pwm_min, pwm_max] to [-1.0, 1.0]"""
        pwm_value = max(self.pwm_min, min(self.pwm_max, pwm_value))
        
        if pwm_value >= self.pwm_center:
            normalized = (pwm_value - self.pwm_center) / (self.pwm_max - self.pwm_center)
        else:
            normalized = (pwm_value - self.pwm_center) / (self.pwm_center - self.pwm_min)
        
        return normalized
    
    def detect_mode(self, channels):
        """Detect robot mode from RC channel
        Returns: 0 = UAV, 1 = MORPH, 2 = UGV
        """
        if len(channels) <= self.mode_channel:
            return self.current_mode
        
        ch_pwm = channels[self.mode_channel]
        
        if ch_pwm < self.mode_uav_threshold:
            return 0  # UAV
        elif ch_pwm > self.mode_morph_threshold:
            return 2  # UGV (high position)
        else:
            return 1  # MORPH (middle position)
    
    def rc_callback(self, msg: RCIn):
        """Process RC input and publish normalized data"""
        
        # Print what we SUBSCRIBE to (raw PWM from MAVROS)
        channels_str = ', '.join([str(ch) for ch in msg.channels[:8]])  # Show first 8 channels
        self.get_logger().info(
            f'📥 SUBSCRIBED: /mavros/rc/in → PWM[{channels_str}]',
            throttle_duration_sec=2.0
        )
        
        # Detect mode
        new_mode = self.detect_mode(msg.channels)
        
        mode_names = {0: "UAV", 1: "MORPH", 2: "UGV"}
        
        if new_mode != self.current_mode:
            self.get_logger().info(
                f'Mode: {mode_names[self.current_mode]} -> {mode_names[new_mode]}'
            )
            self.current_mode = new_mode
        
        # Publish mode as Float32MultiArray with single element
        mode_msg = Float32MultiArray()
        mode_msg.data = [float(self.current_mode)]
        self.mode_pub.publish(mode_msg)
        
        # Print what we PUBLISH to /robot/mode
        self.get_logger().info(
            f'📤 PUBLISHED: /robot/mode → {self.current_mode} ({mode_names[self.current_mode]})',
            throttle_duration_sec=2.0
        )
        
        # Publish raw PWM channels (no normalization)
        channels_msg = Float32MultiArray()
        channels_msg.data = [float(ch) for ch in msg.channels]  # Raw PWM values
        self.rc_channels_pub.publish(channels_msg)
        
        # Print what we PUBLISH to /rc/channels
        if len(channels_msg.data) >= 8:
            pwm_str = ', '.join([f'{int(ch)}' for ch in channels_msg.data[:8]])  # Show first 8
            self.get_logger().info(
                f'📤 PUBLISHED: /rc/channels → PWM[{pwm_str}]',
                throttle_duration_sec=2.0
            )
        
        # If in UGV mode (mode 2), publish motor commands
        if self.current_mode == 2 and len(msg.channels) >= 2:
            ch1_pwm = msg.channels[0]  # Steering (1050-1950)
            ch2_pwm = msg.channels[1]  # Throttle (1050-1950)
            
            # Normalize to -1.0 to 1.0
            steering = self.normalize_pwm(ch1_pwm)  # Right = positive
            throttle = self.normalize_pwm(ch2_pwm)  # Forward = positive
            
            # Publish motor commands [throttle, steering]
            motor_cmd = Float32MultiArray()
            motor_cmd.data = [throttle, steering]
            self.ugv_motor_pub.publish(motor_cmd)
            
            self.get_logger().info(
                f'📤 PUBLISHED: /ugv/motor_commands → Throttle:{throttle:.2f}, Steering:{steering:.2f}',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = Px4RcBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
