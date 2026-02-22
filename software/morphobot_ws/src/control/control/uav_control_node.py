#!/usr/bin/env python3
"""
UAV Control Node - Air movement control using PX4 Offboard mode.

Subscribes to:
- /robot/mode: Current robot mode
- /uav/rc_commands: Normalized RC commands [roll, pitch, throttle, yaw]

Publishes to:
- /fmu/in/offboard_control_mode: PX4 offboard control mode
- /fmu/in/trajectory_setpoint: PX4 trajectory setpoints
- /fmu/in/vehicle_command: PX4 vehicle commands (arm, mode set)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)
import math
import time


class UAVControlNode(Node):
    """UAV control - handles communication and state during UAV flight using PX4 Offboard."""
    
    def __init__(self):
        super().__init__('uav_control_node')
        
        # QoS Profile for PX4 communication (standard for Micro-XRCE-DDS bridge)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Parameters
        self.declare_parameter('vel_max_xy', 2.0)    # Max horizontal velocity (m/s)
        self.declare_parameter('vel_max_z', 1.0)     # Max vertical velocity (m/s)
        self.declare_parameter('yaw_rate_max', 1.5)  # Max yaw rate (rad/s)
        
        self.vel_max_xy = self.get_parameter('vel_max_xy').value
        self.vel_max_z = self.get_parameter('vel_max_z').value
        self.yaw_rate_max = self.get_parameter('yaw_rate_max').value
        
        # State
        self.uav_active = False
        self.last_rc_cmds = [0.0, 0.0, 0.5, 0.0]  # [roll, pitch, throttle, yaw] - throttle neutral at 0.5
        
        # Publishers to PX4
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscriptions
        self.mode_sub = self.create_subscription(
            Float32MultiArray,
            '/robot/mode',
            self.mode_callback,
            10
        )
        
        self.uav_cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/uav/rc_commands',
            self.uav_cmd_callback,
            10
        )
        
        # Offboard Loop (needs to be > 2Hz for PX4 heartbeat)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        self.get_logger().info('UAV Control Node (Offboard) initialized')
    
    def mode_callback(self, msg: Float32MultiArray):
        """Activate only in UAV mode (mode == 0)."""
        if len(msg.data) < 1:
            return
        mode = int(msg.data[0])
        
        if mode == 0 and not self.uav_active:
            self.get_logger().info('UAV mode active — flight controls enabled')
            self.uav_active = True
        elif mode != 0 and self.uav_active:
            self.get_logger().info('Exited UAV mode — flight controls disabled')
            self.uav_active = False
    
    def uav_cmd_callback(self, msg: Float32MultiArray):
        """Store normalized UAV commands."""
        if len(msg.data) < 4:
            return
        self.last_rc_cmds = msg.data
        
    def timer_callback(self):
        """Main control loop - runs at 20Hz."""
        if not self.uav_active:
            return
            
        # 1. Always publish offboard mode heartbeat
        self.publish_offboard_control_mode()
        
        # 2. Map RC commands to trajectories
        roll, pitch, throttle, yaw = self.last_rc_cmds
        
        # Transform RC inputs to velocities
        # roll/pitch mapped to vx/vy (standard drone mapping)
        vx = pitch * self.vel_max_xy
        vy = roll * self.vel_max_xy
        # throttle 0.0-1.0 mapped to -vel_max_z to +vel_max_z
        vz = (throttle - 0.5) * 2.0 * self.vel_max_z
        # yaw mapped to yaw rate
        yaw_rate = yaw * self.yaw_rate_max
        
        self.publish_trajectory_setpoint(vx, vy, vz, yaw_rate)
        
        self.get_logger().info(
            f'Offboard setpoint: vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} yaw={yaw_rate:.2f}',
            throttle_duration_sec=2.0
        )

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, vx, vy, vz, yaw_rate):
        """Publish a trajectory setpoint (velocity control)."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        # We are using velocity control, so position and acceleration are NaN
        msg.position = [float('nan')] * 3
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = yaw_rate
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UAVControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
