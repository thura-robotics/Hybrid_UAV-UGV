
#!/usr/bin/env python3
"""
Morphobot System Monitor — Terminal Dashboard

Displays a real-time overview of robot status:
  • Battery voltage & percentage
  • Vehicle state (connected, armed, flight mode)
  • Robot mode (UAV / UGV / MORPH)
  • RC signal status
  • ROS 2 controller status
  • GPS / position info

Run:  ros2 run control system_monitor
"""

import os
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State, RCIn, VfrHud
from diagnostic_msgs.msg import DiagnosticArray


class SystemMonitor(Node):
    """Terminal dashboard for Morphobot system monitoring."""

    def __init__(self):
        super().__init__('system_monitor')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('refresh_rate', 1.0)  # Hz
        refresh = self.get_parameter('refresh_rate').value

        # ── Cached state ──────────────────────────────────────────────
        # Battery
        self.batt_voltage = 0.0
        self.batt_current = 0.0
        self.batt_percentage = -1.0   # -1 = no data
        self.batt_cells = 0

        # FC / Vehicle
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = '---'
        self.fc_guided = False

        # Robot mode (from px4_rc_bridge)
        self.robot_mode = -1         # 0=UAV, 1=MORPH, 2=UGV
        self.flight_mode = -1        # 0=Manual, 1=Position
        self.arm_switch = -1         # 0=Disarm, 1=Arm

        # RC
        self.rc_channels = []
        self.rc_last_time = None

        # VFR HUD (airspeed, groundspeed, altitude, heading)
        self.airspeed = 0.0
        self.groundspeed = 0.0
        self.altitude = 0.0
        self.heading = 0

        # GPS
        self.gps_fix = -1            # -1 = no data
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        self.gps_alt = 0.0
        self.gps_satellites = -1

        # Controller status (queried periodically)
        self.controller_status = {}

        # ── QoS for MAVROS topics ─────────────────────────────────────
        mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            BatteryState, '/mavros/battery',
            self.battery_cb, mavros_qos)

        self.create_subscription(
            State, '/mavros/state',
            self.state_cb, mavros_qos)

        self.create_subscription(
            RCIn, '/mavros/rc/in',
            self.rc_cb, mavros_qos)

        self.create_subscription(
            VfrHud, '/mavros/vfr_hud',
            self.vfr_hud_cb, mavros_qos)

        self.create_subscription(
            NavSatFix, '/mavros/global_position/global',
            self.gps_cb, mavros_qos)

        # Robot-level topics (from px4_rc_bridge)
        self.create_subscription(
            Float32MultiArray, '/robot/mode',
            self.robot_mode_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/robot/flight_mode',
            self.flight_mode_cb, 10)

        self.create_subscription(
            Float32MultiArray, '/robot/arm_command',
            self.arm_cb, 10)

        # ── Timers ────────────────────────────────────────────────────
        self.create_timer(1.0 / refresh, self.display_dashboard)
        self.create_timer(5.0, self.query_controllers)   # query every 5s

        self.get_logger().info('System Monitor started')

    # ── Subscription callbacks ────────────────────────────────────────

    def battery_cb(self, msg: BatteryState):
        self.batt_voltage = msg.voltage
        self.batt_current = msg.current if msg.current == msg.current else 0.0  # NaN check
        self.batt_percentage = msg.percentage if msg.percentage == msg.percentage else -1.0
        if msg.cell_voltage:
            self.batt_cells = len(msg.cell_voltage)

    def state_cb(self, msg: State):
        self.fc_connected = msg.connected
        self.fc_armed = msg.armed
        self.fc_mode = msg.mode if msg.mode else '---'
        self.fc_guided = msg.guided

    def rc_cb(self, msg: RCIn):
        self.rc_channels = list(msg.channels)
        self.rc_last_time = self.get_clock().now()

    def vfr_hud_cb(self, msg: VfrHud):
        self.airspeed = msg.airspeed
        self.groundspeed = msg.groundspeed
        self.altitude = msg.altitude
        self.heading = msg.heading

    def gps_cb(self, msg: NavSatFix):
        self.gps_fix = msg.status.status
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_alt = msg.altitude

    def robot_mode_cb(self, msg: Float32MultiArray):
        if msg.data:
            self.robot_mode = int(msg.data[0])

    def flight_mode_cb(self, msg: Float32MultiArray):
        if msg.data:
            self.flight_mode = int(msg.data[0])

    def arm_cb(self, msg: Float32MultiArray):
        if msg.data:
            self.arm_switch = int(msg.data[0])

    # ── Controller query ──────────────────────────────────────────────

    def query_controllers(self):
        """Query ros2 control controller_manager for active controllers."""
        try:
            result = subprocess.run(
                ['ros2', 'control', 'list_controllers'],
                capture_output=True, text=True, timeout=3
            )
            self.controller_status = {}
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    line = line.strip()
                    if not line or line.startswith('['):
                        continue
                    # Format: "controller_name  [type]  active/inactive"
                    parts = line.split()
                    if len(parts) >= 1:
                        name = parts[0]
                        state = ''
                        for p in parts:
                            if p in ('active', 'inactive', 'unconfigured', 'finalized'):
                                state = p
                                break
                        self.controller_status[name] = state
        except Exception:
            pass  # silently skip if unavailable

    # ── Dashboard rendering ───────────────────────────────────────────

    def _bar(self, pct, width=15):
        """Create a text progress bar."""
        if pct < 0:
            return '░' * width + ' N/A'
        filled = int(pct / 100.0 * width)
        filled = max(0, min(width, filled))
        bar = '█' * filled + '░' * (width - filled)
        return bar

    def _color(self, text, code):
        """ANSI color wrapper."""
        return f'\033[{code}m{text}\033[0m'

    def display_dashboard(self):
        """Clear screen and print the dashboard."""
        # Clear screen
        print('\033[2J\033[H', end='')

        W = 52  # box width
        HL = '═' * (W - 2)

        # ── Header ──
        print(f'╔{HL}╗')
        title = '  MORPHOBOT SYSTEM MONITOR  '
        print(f'║{title:^{W-2}}║')
        print(f'╠{HL}╣')

        # ── Robot Mode ──
        mode_names = {0: 'UAV ✈', 1: 'MORPH ⚙', 2: 'UGV 🚗', -1: '---'}
        fm_names = {0: 'MANUAL', 1: 'POSITION', -1: '---'}
        arm_names = {0: 'DISARMED', 1: 'ARMED', -1: '---'}

        mode_str = mode_names.get(self.robot_mode, '???')
        fm_str = fm_names.get(self.flight_mode, '???')
        arm_str = arm_names.get(self.arm_switch, '???')
        line = f' Mode: {mode_str} | {fm_str} | {arm_str}'
        print(f'║{line:<{W-2}}║')
        print(f'╠{HL}╣')

        # ── Battery ──
        if self.batt_percentage >= 0:
            pct = self.batt_percentage * 100 if self.batt_percentage <= 1.0 else self.batt_percentage
        else:
            pct = -1
        bar = self._bar(pct)
        if pct >= 0:
            pct_str = f'{pct:.0f}%'
        else:
            pct_str = 'N/A'
        line = f' Battery: {self.batt_voltage:.1f}V  {pct_str}  {bar}'
        print(f'║{line:<{W-2}}║')

        if self.batt_current > 0:
            line = f'          {self.batt_current:.1f}A  {self.batt_cells}S'
            print(f'║{line:<{W-2}}║')

        print(f'╠{HL}╣')

        # ── FC State ──
        conn = '● CONNECTED' if self.fc_connected else '○ DISCONNECTED'
        armed = 'ARMED' if self.fc_armed else 'DISARMED'
        line = f' FC: {conn}'
        print(f'║{line:<{W-2}}║')
        line = f'     {self.fc_mode} | {armed}'
        print(f'║{line:<{W-2}}║')

        # ── HUD ──
        line = f'     Alt:{self.altitude:.1f}m  GS:{self.groundspeed:.1f}m/s  Hdg:{self.heading}°'
        print(f'║{line:<{W-2}}║')
        print(f'╠{HL}╣')

        # ── GPS ──
        fix_names = {-1: 'NO FIX', 0: 'FIX', 1: 'SBAS', 2: 'DGPS'}
        fix_str = fix_names.get(self.gps_fix, f'TYPE:{self.gps_fix}')
        line = f' GPS: {fix_str}'
        print(f'║{line:<{W-2}}║')
        if self.gps_fix >= 0:
            line = f'      {self.gps_lat:.6f}, {self.gps_lon:.6f}'
            print(f'║{line:<{W-2}}║')
            line = f'      Alt: {self.gps_alt:.1f}m'
            print(f'║{line:<{W-2}}║')
        print(f'╠{HL}╣')

        # ── RC ──
        if self.rc_channels:
            n_ch = len(self.rc_channels)
            # Check staleness
            if self.rc_last_time:
                age = (self.get_clock().now() - self.rc_last_time).nanoseconds / 1e9
                if age > 2.0:
                    rc_status = f'STALE ({age:.0f}s ago)'
                else:
                    rc_status = f'OK ({n_ch}ch)'
            else:
                rc_status = f'{n_ch}ch'
            line = f' RC: {rc_status}'
            print(f'║{line:<{W-2}}║')
            # Show first 8 channels compactly
            ch_str = ' '.join([f'{int(c):4d}' for c in self.rc_channels[:8]])
            line = f'     {ch_str}'
            print(f'║{line:<{W-2}}║')
        else:
            line = ' RC: NO DATA'
            print(f'║{line:<{W-2}}║')
        print(f'╠{HL}╣')

        # ── Controllers ──
        if self.controller_status:
            line = ' Controllers:'
            print(f'║{line:<{W-2}}║')
            for name, state in self.controller_status.items():
                icon = '●' if state == 'active' else '○'
                line = f'   {icon} {name} [{state}]'
                print(f'║{line:<{W-2}}║')
        else:
            line = ' Controllers: (querying...)'
            print(f'║{line:<{W-2}}║')

        # ── Footer ──
        print(f'╚{HL}╝')
        print(' Press Ctrl+C to exit')


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()

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
