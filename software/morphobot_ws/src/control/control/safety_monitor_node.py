#!/usr/bin/env python3
"""
Safety Monitor Node - Monitors system state and triggers failsafes if necessary.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Empty, Bool
from mavros_msgs.msg import State

class SafetyMonitorNode(Node):
    """Monitors system parameters and ensures safe operation."""
    
    def __init__(self):
        super().__init__('safety_monitor_node')
        
        # Parameters
        self.declare_parameter('watchdog_period', 0.2) # seconds
        self.declare_parameter('cmd_timeout', 1.0) # seconds
        self.declare_parameter('heartbeat_timeout', 2.0) # seconds
        self.declare_parameter('mavros_timeout', 2.0) # seconds
        
        self.watchdog_period = self.get_parameter('watchdog_period').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        self.mavros_timeout = self.get_parameter('mavros_timeout').value
        
        # State variables
        
        self.px4_connected = False
        self.estop_active = False
        
        # Timestamps for timeouts
        now = self.get_clock().now()
        self.last_heartbeat_time = now
        self.last_mavros_state_time = now
        
        # Subscriptions
        self.create_subscription(State, '/mavros/state', self.mavros_state_cb, 10)
        self.create_subscription(Empty, '/robot/heartbeat', self.heartbeat_cb, 10)
        
        # Publisher
        self.estop_pub = self.create_publisher(Bool, '/robot/emergency_stop', 10)
        
        # Timer for periodic safety checks (watchdog)
        self.timer = self.create_timer(self.watchdog_period, self.watchdog_check)
        
        self.get_logger().info('Safety Monitor Node started.')
        self.get_logger().info(f'Watchdog timer running every {self.watchdog_period}s.')

    def mavros_state_cb(self, msg: State):
        self.px4_connected = msg.connected
        self.last_mavros_state_time = self.get_clock().now()

    def heartbeat_cb(self, msg: Empty):
        self.last_heartbeat_time = self.get_clock().now()

    def manual_estop_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().error("MANUAL ESTOP TRIGGERED!")
            self.trigger_estop()


    def trigger_estop(self):
        if not self.estop_active:
            self.estop_active = True
            msg = Bool()
            msg.data = True
            self.estop_pub.publish(msg)
            self.get_logger().error("Emergency Stop Activated!")

    def watchdog_check(self):
        """Perform periodic safety evaluations."""
        now = self.get_clock().now()
        
        # 1. Check PX4 Connection
        if not self.px4_connected:
            self.get_logger().error("Safety violation: PX4 not connected.")
            self.trigger_estop()
            
        mavros_dt = (now - self.last_mavros_state_time).nanoseconds / 1e9
        if mavros_dt > self.mavros_timeout:
            self.get_logger().error("Safety violation: MAVROS state timeout.")
            self.trigger_estop()

        # 2. Check Heartbeat timeout
        hb_dt = (now - self.last_heartbeat_time).nanoseconds / 1e9
        if hb_dt > self.heartbeat_timeout:
            self.get_logger().error("Safety violation: Heartbeat timeout.")
            self.trigger_estop()

        # 3. Check Command timeouts based on mode
        if self.current_mode == 0.0: # UAV mode
            uav_dt = (now - self.last_uav_cmd_time).nanoseconds / 1e9
            if uav_dt > self.cmd_timeout:
                self.get_logger().error("Safety violation: UAV command timeout.")
                self.trigger_estop()
                
        elif self.current_mode == 2.0: # UGV mode
            ugv_dt = (now - self.last_ugv_cmd_time).nanoseconds / 1e9
            if ugv_dt > self.cmd_timeout:
                self.get_logger().error("Safety violation: UGV command timeout.")
                self.trigger_estop()
                
        # Publish estop state repeatedly if active
        if self.estop_active:
            msg = Bool()
            msg.data = True
            self.estop_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
