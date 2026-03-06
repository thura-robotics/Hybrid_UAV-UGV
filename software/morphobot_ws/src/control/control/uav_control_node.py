#!/usr/bin/env python3
"""
UAV Control Node — Lightweight MAVROS Monitor (Option A)

PX4 handles all flight control directly via the RC receiver.
This node only monitors the UAV state for logging and coordination
with the rest of the Morphobot system (UGV, morphing).

Subscribes to:
  /robot/mode          → Activate monitoring when mode == 0 (UAV)
  /robot/flight_mode   → CH6: Manual (0) / Position (1)
  /robot/arm_command   → CH7: Disarmed (0) / Armed (1)
  /mavros/state        → FC connected, armed, current mode
  /uav/rc_commands     → [roll, pitch, throttle, yaw] for logging

Publishes to:
  (nothing — PX4 flies directly)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import State


class UAVControlNode(Node):
    """UAV monitor — tracks flight state when robot is in UAV mode."""

    def __init__(self):
        super().__init__('uav_control_node')

        # ── State ─────────────────────────────────────────────────────
        self.uav_active = False        # True when robot mode == UAV
        self.flight_mode = 0           # 0=Manual, 1=Position (from CH6)
        self.arm_command = 0           # 0=Disarm, 1=Arm (from CH7)
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = ''

        # ── Subscriptions ─────────────────────────────────────────────

        # Robot mode from px4_rc_bridge (CH5)
        self.create_subscription(
            Float32MultiArray, '/robot/mode',
            self.mode_callback, 10)

        # Flight mode from px4_rc_bridge (CH6)
        self.create_subscription(
            Float32MultiArray, '/robot/flight_mode',
            self.flight_mode_callback, 10)

        # Arm command from px4_rc_bridge (CH7)
        self.create_subscription(
            Float32MultiArray, '/robot/arm_command',
            self.arm_callback, 10)

        # MAVROS FC state
        self.create_subscription(
            State, '/mavros/state',
            self.state_callback, 10)

        # RC stick commands (for logging only)
        self.create_subscription(
            Float32MultiArray, '/uav/rc_commands',
            self.rc_cmd_callback, 10)

        # ── Status timer ──────────────────────────────────────────────
        self.timer = self.create_timer(2.0, self.status_timer)

        self.get_logger().info('UAV Control Node started (Monitor — PX4 flies directly)')

    # ── Callbacks ─────────────────────────────────────────────────────

    def mode_callback(self, msg: Float32MultiArray):
        """Activate monitoring when mode == 0 (UAV)."""
        if not msg.data:
            return
        mode = int(msg.data[0])

        if mode == 0 and not self.uav_active:
            self.uav_active = True
            self.get_logger().info('═══ UAV MODE ACTIVE — monitoring flight state ═══')
        elif mode != 0 and self.uav_active:
            self.uav_active = False
            self.get_logger().info('═══ Exited UAV mode ═══')

    def flight_mode_callback(self, msg: Float32MultiArray):
        """Track CH6 flight mode switch."""
        if not msg.data:
            return
        new_fm = int(msg.data[0])
        if new_fm != self.flight_mode:
            names = {0: 'MANUAL', 1: 'POSITION'}
            self.get_logger().info(f'Flight mode: {names.get(self.flight_mode)} → {names.get(new_fm)}')
            self.flight_mode = new_fm

    def arm_callback(self, msg: Float32MultiArray):
        """Track CH7 arm switch."""
        if not msg.data:
            return
        new_arm = int(msg.data[0])
        if new_arm != self.arm_command:
            names = {0: 'DISARMED', 1: 'ARMED'}
            self.get_logger().info(f'Arm switch: {names.get(self.arm_command)} → {names.get(new_arm)}')
            self.arm_command = new_arm

    def state_callback(self, msg: State):
        """Track MAVROS/FC state."""
        changed = (
            msg.connected != self.fc_connected or
            msg.armed != self.fc_armed or
            msg.mode != self.fc_mode
        )
        self.fc_connected = msg.connected
        self.fc_armed = msg.armed
        self.fc_mode = msg.mode

        if changed and self.uav_active:
            self.get_logger().info(
                f'FC: connected={self.fc_connected} armed={self.fc_armed} mode={self.fc_mode}')

    def rc_cmd_callback(self, msg: Float32MultiArray):
        """Log RC stick values (monitor only, no action taken)."""
        if not self.uav_active or len(msg.data) < 4:
            return
        # Logging handled by px4_rc_bridge already

    # ── Periodic status ───────────────────────────────────────────────

    def status_timer(self):
        """Print periodic status when in UAV mode."""
        if not self.uav_active:
            return

        fm_name = 'POSITION' if self.flight_mode else 'MANUAL'
        arm_name = 'ARMED' if self.arm_command else 'DISARMED'
        fc_status = f'FC:{self.fc_mode}' if self.fc_connected else 'FC:DISCONNECTED'

        self.get_logger().info(
            f'UAV | {fm_name} | {arm_name} | {fc_status} | fc_armed={self.fc_armed}')


def main(args=None):
    rclpy.init(args=args)
    node = UAVControlNode()

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
