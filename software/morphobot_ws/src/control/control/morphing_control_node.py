#!/usr/bin/env python3
"""
Morphing Control Node

Joint order for /position_controller/commands: S1, S2, S4, S5, S7, S8
                                    index:      0   1   2   3   4   5

HOME:  S1=2048 S2=2760 S4=2048 S5=1200 S7=2048 S8=1200

UAV sequence (after going home):
  Step 1 → S2=2048, S5=2048, S8=2048
  Step 2 → S1=900,  S4=2900, S7=2900
  Step 3 → S2=2900, S5=1000, S8=1000

MORPH / UGV / default: go to HOME
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray

STEP_DELAY = 2.0   # seconds between UAV sequence steps

#              S1    S2    S4    S5    S7    S8
HOME      = [2048, 2760, 2048, 1200, 2048, 1200]
UAV_STEP1 = [2048, 2048, 2048, 2048, 2048, 2048]  # S2, S5, S8 → 2048
UAV_STEP2 = [ 900, 2048, 2900, 2048, 2900, 2048]  # S1→900, S4→2900, S7→2900
UAV_STEP3 = [ 900, 2900, 2900, 1000, 2900, 1000]  # S2→2900, S5→1000, S8→1000

UAV_STEPS = [UAV_STEP1, UAV_STEP2, UAV_STEP3]


def t2r(ticks):
    return ticks / 4096.0 * 2.0 * math.pi


class MorphingControlNode(Node):

    def __init__(self):
        super().__init__('morphing_control_node')
        self.current_mode = -1
        self._positions = list(HOME)
        self._seq_step = 0
        self._timer = None

        self.create_subscription(
            Float32MultiArray, '/robot/mode', self.mode_callback, 10)
        self.pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        self.get_logger().info('Morphing Control Node ready')
        self.get_logger().info(
            'HOME: S1=2048 S2=2760 S4=2048 S5=1200 S7=2048 S8=1200')

    def _publish(self, positions):
        msg = Float64MultiArray()
        msg.data = [t2r(t) for t in positions]
        self.pub.publish(msg)
        self.get_logger().info(f'Cmd: {positions}')

    def _cancel_timer(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None

    def mode_callback(self, msg: Float32MultiArray):
        if not msg.data:
            return
        new_mode = int(msg.data[0])
        if new_mode == self.current_mode:
            return
        self.current_mode = new_mode
        self._cancel_timer()

        names = {0: 'UAV', 1: 'MORPH', 2: 'UGV'}
        self.get_logger().info(f'Mode → {names.get(new_mode, str(new_mode))}')

        # Always go home first
        self._positions = list(HOME)
        self._publish(self._positions)
        self.get_logger().info('→ HOME')

        if new_mode == 0:   # UAV: run sequence after home
            self._seq_step = 0
            self._timer = self.create_timer(STEP_DELAY, self._uav_next_step)

    def _uav_next_step(self):
        self._cancel_timer()
        if self._seq_step >= len(UAV_STEPS):
            self.get_logger().info('UAV sequence complete ✓')
            return
        step = UAV_STEPS[self._seq_step]
        self.get_logger().info(
            f'UAV step {self._seq_step + 1}/{len(UAV_STEPS)}: {step}')
        self._publish(step)
        self._seq_step += 1
        if self._seq_step < len(UAV_STEPS):
            self._timer = self.create_timer(STEP_DELAY, self._uav_next_step)
        else:
            self.get_logger().info('UAV sequence complete ✓')


def main(args=None):
    rclpy.init(args=args)
    node = MorphingControlNode()
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
