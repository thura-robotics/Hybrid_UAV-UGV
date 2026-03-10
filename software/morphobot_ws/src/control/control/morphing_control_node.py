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

STEP_DELAY = 3.0   # seconds between sequence steps
MATCH_THRESHOLD = 0.077   # ~50 ticks (50/4096 * 2π)

# Joint Names mapped to indices in UAV/UGV arrays
JOINTS = ['hip_FL', 'ankle_FL', 'hip_BL', 'ankle_BL', 'hip_FR', 'ankle_FR', 'hip_BR', 'ankle_BR']
HOME      = [2048, 2875, 2048, 1217, 2048, 1327, 2048, 2865] 

UAV_STEP1 = [2048, 2048, 2048, 2048, 2048, 2048,2048, 2048]  
UAV_STEP2 = [ 1684, 2032, 2378, 2134, 2347, 2174, 1740, 2062]  
UAV_STEP3 = [ 1684, 2971, 2378, 997, 2347, 1210, 1740, 2997] 
UAV_HOME= [ 1092, 2973, 3016, 1122, 2982, 1226,1096, 2996]  
UAV_STEPS = [UAV_STEP1,UAV_STEP2,UAV_STEP3,UAV_HOME]


UGV_STEP1 = [ 1684, 2971, 2378, 997, 2347, 1210, 1740, 2997]  
UGV_STEP2 = [ 1684, 2032, 2378, 2134, 2347, 2174, 1740, 2062]  
UGV_STEP3 = [ 2048, 2048, 2048, 2048, 2048, 2048,2048, 2048] 
UGV_HOME = [2048, 2875, 2048, 1217, 2048, 1327, 2048, 2865] 


UGV_STEPS = [ UGV_STEP1,UGV_STEP2,UGV_STEP3,UGV_HOME]


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

    def _log_position_comparison(self, target_ticks, label):
        """Log current vs target positions for debugging."""
        if not self._current_joint_positions:
            self.get_logger().warn(f'  {label}: No joint positions received yet!')
            return
        for i, name in enumerate(JOINTS):
            if name not in self._current_joint_positions:
                self.get_logger().warn(f'  {label}: {name} — NO DATA')
                continue
            curr_rad = self._current_joint_positions[name]
            curr_ticks = int(curr_rad / (2.0 * math.pi) * 4096)
            target = target_ticks[i]
            delta = abs(curr_ticks - target)
            mark = '✗' if delta > (MATCH_THRESHOLD / (2.0 * math.pi) * 4096) else '✓'
            self.get_logger().info(f'  {label} {name}: curr={curr_ticks} target={target} Δ={delta} {mark}')

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
                self.get_logger().warn('--- Position comparison vs UGV_HOME ---')
                self._log_position_comparison(UGV_HOME, 'UGV_HOME')
                self.get_logger().warn('--- Position comparison vs UAV_HOME ---')
                self._log_position_comparison(UAV_HOME, 'UAV_HOME')

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
