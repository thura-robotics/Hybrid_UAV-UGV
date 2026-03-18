#!/usr/bin/env python3
"""
PX4 RC Bridge Node
Subscribes to MAVROS RC input and republishes as simple normalized values.

RC Channel Map:
  CH5 (idx 4) → Robot mode:  UAV / MORPH / UGV
  CH6 (idx 5) → Flight mode: Manual / Position
  CH7 (idx 6) → Arm switch:  Disarmed / Armed
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn
from std_msgs.msg import Float32MultiArray


class Px4RcBridge(Node):
    """Bridge between PX4/MAVROS RC input and simple ROS topics"""

    def __init__(self):
        super().__init__('px4_rc_bridge')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('pwm_min', 1000)
        self.declare_parameter('pwm_max', 2000)
        self.declare_parameter('pwm_center', 1500)

        # Channel indices (0-indexed)
        self.declare_parameter('mode_channel', 4)          # CH5
        self.declare_parameter('flight_mode_channel', 5)   # CH6
        self.declare_parameter('arm_channel', 6)           # CH7

        # Thresholds
        self.declare_parameter('mode_uav_threshold', 1300)
        self.declare_parameter('mode_morph_threshold', 1700)
        self.declare_parameter('switch_threshold', 1500)   # CH6/CH7 high/low boundary

        self.pwm_min = self.get_parameter('pwm_min').value
        self.pwm_max = self.get_parameter('pwm_max').value
        self.pwm_center = self.get_parameter('pwm_center').value
        self.mode_channel = self.get_parameter('mode_channel').value
        self.flight_mode_channel = self.get_parameter('flight_mode_channel').value
        self.arm_channel = self.get_parameter('arm_channel').value
        self.mode_uav_threshold = self.get_parameter('mode_uav_threshold').value
        self.mode_morph_threshold = self.get_parameter('mode_morph_threshold').value
        self.switch_threshold = self.get_parameter('switch_threshold').value

        # ── State ─────────────────────────────────────────────────────
        self.current_mode = 1          # 0=UAV, 1=MORPH, 2=UGV
        self.current_flight_mode = 0   # 0=Manual, 1=Position
        self.current_arm = 0           # 0=Disarmed, 1=Armed

        # ── Subscriber ────────────────────────────────────────────────
        self.rc_sub = self.create_subscription(
            RCIn, '/mavros/rc/in', self.rc_callback, 10)

        # ── Publishers ────────────────────────────────────────────────
        self.rc_channels_pub = self.create_publisher(
            Float32MultiArray, '/rc/channels', 10)

        self.mode_pub = self.create_publisher(
            Float32MultiArray, '/robot/mode', 10)

        self.flight_mode_pub = self.create_publisher(
            Float32MultiArray, '/robot/flight_mode', 10)

        self.arm_pub = self.create_publisher(
            Float32MultiArray, '/robot/arm_command', 10)

        self.ugv_motor_pub = self.create_publisher(
            Float32MultiArray, '/ugv/motor_commands', 10)

        self.uav_rc_pub = self.create_publisher(
            Float32MultiArray, '/uav/rc_commands', 10)

        # ── Startup log ──────────────────────────────────────────────
        self.get_logger().info('PX4 RC Bridge started')
        self.get_logger().info(
            f'CH5(mode)={self.mode_channel+1}  '
            f'CH6(flight)={self.flight_mode_channel+1}  '
            f'CH7(arm)={self.arm_channel+1}')

    # ── Helpers ───────────────────────────────────────────────────────

    def normalize_pwm(self, pwm_value):
        """Normalize PWM value from [pwm_min, pwm_max] to [-1.0, 1.0]"""
        pwm_value = max(self.pwm_min, min(self.pwm_max, pwm_value))
        if pwm_value >= self.pwm_center:
            return (pwm_value - self.pwm_center) / (self.pwm_max - self.pwm_center)
        else:
            return (pwm_value - self.pwm_center) / (self.pwm_center - self.pwm_min)

    def detect_mode(self, channels):
        """Detect robot mode from CH5.  Returns: 0=UAV, 1=MORPH, 2=UGV"""
        if len(channels) <= self.mode_channel:
            return self.current_mode
        ch = channels[self.mode_channel]
        if ch < self.mode_uav_threshold:
            return 0   # UAV
        elif ch > self.mode_morph_threshold:
            return 2   # UGV
        else:
            return 1   # MORPH

    def detect_switch(self, channels, channel_idx):
        """Read a 2-position switch.  Returns: 0 (low) or 1 (high)"""
        if len(channels) <= channel_idx:
            return 0
        return 1 if channels[channel_idx] >= self.switch_threshold else 0

    # ── Main callback ─────────────────────────────────────────────────

    def rc_callback(self, msg: RCIn):
        """Process RC input and publish parsed data"""

        mode_names = {0: "UAV", 1: "MORPH", 2: "UGV"}
        flight_names = {0: "MANUAL", 1: "POSITION"}
        arm_names = {0: "DISARMED", 1: "ARMED"}

        # --- CH5: Robot mode ---
        new_mode = self.detect_mode(msg.channels)
        if new_mode != self.current_mode:
            self.get_logger().info(
                f'Mode: {mode_names[self.current_mode]} → {mode_names[new_mode]}')
            self.current_mode = new_mode

        mode_msg = Float32MultiArray()
        mode_msg.data = [float(self.current_mode)]
        self.mode_pub.publish(mode_msg)

        # --- CH6: Flight mode ---
        new_flight = self.detect_switch(msg.channels, self.flight_mode_channel)
        if new_flight != self.current_flight_mode:
            self.get_logger().info(
                f'Flight mode: {flight_names[self.current_flight_mode]} → {flight_names[new_flight]}')
            self.current_flight_mode = new_flight

        fm_msg = Float32MultiArray()
        fm_msg.data = [float(self.current_flight_mode)]
        self.flight_mode_pub.publish(fm_msg)

        # --- CH7: Arm switch ---
        new_arm = self.detect_switch(msg.channels, self.arm_channel)
        if new_arm != self.current_arm:
            self.get_logger().info(
                f'Arm: {arm_names[self.current_arm]} → {arm_names[new_arm]}')
            self.current_arm = new_arm

        arm_msg = Float32MultiArray()
        arm_msg.data = [float(self.current_arm)]
        self.arm_pub.publish(arm_msg)

        # --- Status summary (throttled) ---
        self.get_logger().info(
            f'{mode_names[self.current_mode]} | '
            f'{flight_names[self.current_flight_mode]} | '
            f'{arm_names[self.current_arm]}',
            throttle_duration_sec=2.0)

        # --- Raw PWM channels ---
        channels_msg = Float32MultiArray()
        channels_msg.data = [float(ch) for ch in msg.channels]
        self.rc_channels_pub.publish(channels_msg)

        # --- Mode-specific commands ---

        # UGV mode (mode 2) → publish motor commands
        if self.current_mode == 2 and len(msg.channels) >= 2:
            steering = self.normalize_pwm(msg.channels[0])
            throttle = self.normalize_pwm(msg.channels[1])

            motor_cmd = Float32MultiArray()
            motor_cmd.data = [throttle, steering]
            self.ugv_motor_pub.publish(motor_cmd)

            self.get_logger().info(
                f'UGV: T:{throttle:.2f} S:{steering:.2f}',
                throttle_duration_sec=2.0)

        # UAV mode (mode 0) → publish stick commands (for monitoring / future use)
        elif self.current_mode == 0 and len(msg.channels) >= 4:
            roll = self.normalize_pwm(msg.channels[0])
            pitch = self.normalize_pwm(msg.channels[1])
            throttle = (msg.channels[2] - self.pwm_min) / (self.pwm_max - self.pwm_min)
            throttle = max(0.0, min(1.0, throttle))
            yaw = self.normalize_pwm(msg.channels[3])

            uav_msg = Float32MultiArray()
            uav_msg.data = [roll, pitch, throttle, yaw]
            self.uav_rc_pub.publish(uav_msg)

            self.get_logger().info(
                f'UAV sticks: R:{roll:.2f} P:{pitch:.2f} T:{throttle:.2f} Y:{yaw:.2f}',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = Px4RcBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
