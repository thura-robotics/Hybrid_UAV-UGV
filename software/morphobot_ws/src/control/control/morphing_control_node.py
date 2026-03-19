#!/usr/bin/env python3
"""

Listens to robot mode, Checks current joint positions, Runs a morphing sequence, Publishes servo positions, Keeps holding the final position


Subscribe: robot mode --- Float64MultiArray
Subscribe: joint states --- JointState
Publish : /position_controller/commands --- Float64MultiArray

Joint order for /position_controller/commands: S1, S2, S4, S5, S7, S8
                                    index:      0   1   2   3   4   5

"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray
from sensor_msgs.msg import JointState

STEP_DELAY = 3.0            # seconds between sequence steps
MATCH_THRESHOLD = 0.090     # ~90 ticks (90/4096 * 2π)
HOLD_PUBLISH_RATE = 0.5     # seconds — re-publish interval to keep servos from going limp

# Joint Names mapped to indices in UAV/UGV arrays
JOINTS = ['hip_FL', 'ankle_FL', 'hip_BL', 'ankle_BL', 'hip_FR', 'ankle_FR', 'hip_BR', 'ankle_BR']
HOME      = [2048, 2875, 2048, 1217, 2048, 1327, 2048, 2865]

UAV_STEP1 = [2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048]
# UAV_HOME= [ 1046, 3050, 3050, 1046, 3050, 1046,1046, 3050] 
UAV_HOME= [ 990, 3050, 3080, 990, 3050, 1046,990, 3050]  
UAV_STEPS = [UAV_STEP1,UAV_HOME]

UGV_STEP1 = [ 2048, 2048, 2048, 2048, 2048, 2048,2048, 2048] 
UGV_HOME = [2048, 3050, 2048, 1046, 2048, 1046, 2048, 3050] 

UGV_STEPS = [UGV_STEP1,UGV_HOME]


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

        # Track last commanded position so the hold timer can re-publish it.
        # This keeps servos stiff (in position-hold mode) when no sequence is running.
        self._last_published_positions = None

        self.create_subscription(
            Float32MultiArray, '/robot/mode', self.mode_callback, 10)
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10)

        self.pub = self.create_publisher(
            Float64MultiArray, '/position_controller/commands', 10)

        # Periodic hold publisher — keeps servos from going limp between commands
        self._hold_timer = self.create_timer(HOLD_PUBLISH_RATE, self._hold_position_callback)

        self.get_logger().info('Morphing Control Node ready (State-Aware)')

    # ------------------------------------------------------------------ #

    def _joint_state_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._current_joint_positions[name] = pos

    # ------------------------------------------------------------------ #

    def _hold_position_callback(self):
        """
        Re-publish the last commanded position at a fixed rate.
        This prevents Dynamixel servos from releasing torque when they stop
        receiving commands (e.g. once a morphing sequence completes in UAV mode).
        """
        if self._last_published_positions is not None:
            msg = Float64MultiArray()
            msg.data = [t2r(t) for t in self._last_published_positions]
            self.pub.publish(msg)

    # ------------------------------------------------------------------ #

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
            mark = 'x' if delta > (MATCH_THRESHOLD / (2.0 * math.pi) * 4096) else '--'
            self.get_logger().info(
                f'  {label} {name}: curr={curr_ticks} target={target} Δ={delta} {mark}')

    # ------------------------------------------------------------------ #

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

    # ------------------------------------------------------------------ #

    def _publish(self, positions):
        """Publish a position command and remember it for the hold timer."""
        self._last_published_positions = positions   # keep for hold-timer re-publish
        msg = Float64MultiArray()
        msg.data = [t2r(t) for t in positions]
        self.pub.publish(msg)
        self.get_logger().info(f'Cmd: {positions}')

    # ------------------------------------------------------------------ #

    def _cancel_timer(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None

    # ------------------------------------------------------------------ #

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
        self.get_logger().info(
            f'Mode Changed: {names.get(old_mode, "INIT")} → {names.get(new_mode, str(new_mode))}')

        # Handle transitions to MORPH
        if new_mode == 1:  # MORPH
            if old_mode == 2:  # UGV
                self.get_logger().info('Previous mode was UGV. Triggering UAV sequence...')
                self._start_sequence(UAV_STEPS)
            elif old_mode == 0:  # UAV
                self.get_logger().info('Previous mode was UAV. Triggering UGV sequence...')
                self._start_sequence(UGV_STEPS)
            else:
                self.get_logger().warn(
                    f'MORPH mode set but old_mode is {old_mode}. No sequence triggered.')
                self.get_logger().warn('--- Position comparison vs UGV_HOME ---')
                self._log_position_comparison(UGV_HOME, 'UGV_HOME')
                self.get_logger().warn('--- Position comparison vs UAV_HOME ---')
                self._log_position_comparison(UAV_HOME, 'UAV_HOME')

    # ------------------------------------------------------------------ #

    def _start_sequence(self, steps):
        self._active_sequence = steps
        self._seq_step = 0
        self._next_step()

    # ------------------------------------------------------------------ #

    def _next_step(self):
        self._cancel_timer()
        if self._seq_step >= len(self._active_sequence):
            self.get_logger().info('Sequence complete ')
            # _hold_timer will now keep re-publishing _last_published_positions
            # so servos remain stiff at the final position.
            return

        step = self._active_sequence[self._seq_step]
        self.get_logger().info(f'Step {self._seq_step + 1}/{len(self._active_sequence)}: {step}')
        self._publish(step)

        self._seq_step += 1
        if self._seq_step < len(self._active_sequence):
            self._timer = self.create_timer(STEP_DELAY, self._next_step)
        else:
            self.get_logger().info('Sequence complete ')
            # _hold_timer will now keep re-publishing _last_published_positions


# ---------------------------------------------------------------------- #

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