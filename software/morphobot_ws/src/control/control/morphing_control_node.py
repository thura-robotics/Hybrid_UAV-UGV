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
from sensor_msgs.msg import JointState

STEP_DELAY = 1.5   # seconds between sequence steps
MATCH_THRESHOLD = 0.057  # ~10 ticks (10/4096 * 2pi)

# Joint Names mapped to indices in UAV/UGV arrays
JOINTS = ['servo_joint_1', 'servo_joint_2', 'servo_joint_4', 'servo_joint_5', 'servo_joint_7', 'servo_joint_8']

UAV_STEP1 = [2048, 2048, 2048, 2048, 2048, 2048]  
UAV_STEP2 = [ 900, 2048, 2900, 2048, 2900, 2048]  
UAV_HOME  = [ 900, 2900, 2900, 1000, 2900, 1000]  
UAV_STEPS = [UAV_STEP1, UAV_STEP2, UAV_HOME]

UGV_STEP1 = [ 900, 2048, 2900, 2048, 2900, 2048]  
UGV_STEP2 = [ 2048, 2048, 2048, 2048, 2048, 2048]  
UGV_HOME  = [ 2048, 2900, 2082, 1157, 2023, 1011]  
UGV_STEPS = [UGV_STEP1, UGV_STEP2, UGV_HOME]


def t2r(ticks):
    return ticks / 4096.0 * 2.0 * math.pi


class MorphingControlNode(Node):

    def __init__(self):
        super().__init__('morphing_control_node')
        self.current_mode = -1
        
        self._current_joint_positions = {}
        self._active_sequence = []
        self._seq_step = 0
        self._timer = None

        self.create_subscription(
            Float32MultiArray, '/robot/mode', self.mode_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)
            
        self.pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        self.get_logger().info('Morphing Control Node ready (State-Aware)')

    def _joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._current_joint_positions[name] = pos

    def _is_at_position(self, target_ticks):
        if not self._current_joint_positions:
            return False
            
        for i, name in enumerate(JOINTS):
            if name not in self._current_joint_positions:
                return False
            curr = self._current_joint_positions[name]
            target = t2r(target_ticks[i])
            if abs(curr - target) > MATCH_THRESHOLD:
                return False
        return True

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
            
        old_mode = self.current_mode
        self.current_mode = new_mode
        self._cancel_timer()

        names = {0: 'UAV', 1: 'MORPH', 2: 'UGV'}
        self.get_logger().info(f'Mode Changed: {names.get(old_mode, "INIT")} → {names.get(new_mode, str(new_mode))}')

        # Handle Transitions to MORPH
        if new_mode == 1: # MORPH
            # Check if we were at UGV_HOME or UAV_HOME to decide which way to morph
            if self._is_at_position(UGV_HOME):
                self.get_logger().info('Detected UGV_HOME state. Triggering UAV sequence...')
                self._start_sequence(UAV_STEPS)
            elif self._is_at_position(UAV_HOME):
                self.get_logger().info('Detected UAV_HOME state. Triggering UGV sequence...')
                self._start_sequence(UGV_STEPS)
            else:
                self.get_logger().warn('MORPH mode set but robot is not in a known HOME position. No sequence triggered.')

    def _start_sequence(self, steps):
        self._active_sequence = steps
        self._seq_step = 0
        self._next_step()

    def _next_step(self):
        self._cancel_timer()
        if self._seq_step >= len(self._active_sequence):
            self.get_logger().info('Sequence complete ✓')
            return
            
        step = self._active_sequence[self._seq_step]
        self.get_logger().info(f'Step {self._seq_step + 1}/{len(self._active_sequence)}: {step}')
        self._publish(step)
        
        self._seq_step += 1
        if self._seq_step < len(self._active_sequence):
            self._timer = self.create_timer(STEP_DELAY, self._next_step)
        else:
            self.get_logger().info('Sequence complete ✓')


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
