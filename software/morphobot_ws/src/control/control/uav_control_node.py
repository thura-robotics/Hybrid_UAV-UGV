#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import StatusText

class UAVControlNode(Node):

    def __init__(self):
        super().__init__('uav_control_node')

        # robot states
        self.robot_mode = 1
        self.flight_mode = 0
        self.arm_command = 0

        # PX4 state
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = ""

        # MAVROS services
        self.arm_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming'
        )
       

        self.mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode'
        )

        # subscriptions
        self.create_subscription(
            Float32MultiArray,
            '/robot/mode',
            self.mode_callback,
            10
        )
        self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self.status_text_callback,
            10
        )
        self.create_subscription(
            Float32MultiArray,
            '/robot/flight_mode',
            self.flight_mode_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/robot/arm_command',
            self.arm_callback,
            10
        )

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/uav/rc_commands',
            self.rc_cmd_callback,
            10
        )

        self.get_logger().info("UAV Control Node Started")

    # ------------------------------------------------

    def mode_callback(self, msg):

        if not msg.data:
            return

        new_mode = int(msg.data[0])

        if new_mode == self.robot_mode:
            return

        self.robot_mode = new_mode

        if self.robot_mode == 0:
            self.get_logger().info("UAV MODE ACTIVE")
        else:
            # Force disarm when leaving UAV mode
            self.get_logger().info(f"Non-UAV mode ({self.robot_mode}) — disarming PX4")
            self.arm_command = 0
            self.disarm_px4()

    # ------------------------------------------------
    def status_text_callback(self, msg):

        # Print PX4 warning messages
        if msg.severity <= 4:  # warning or error
            self.get_logger().error(f"PX4: {msg.text}")
        else:
            self.get_logger().info(f"PX4: {msg.text}")



    
    def flight_mode_callback(self, msg):

        if not msg.data:
            return

        new_mode = int(msg.data[0])

        if new_mode != self.flight_mode:

            names = {0: "MANUAL", 1: "POSITION"}

            self.get_logger().info(
                f"Flight mode: {names.get(self.flight_mode)} -> {names.get(new_mode)}"
            )

            self.flight_mode = new_mode

            if self.robot_mode == 0 and self.flight_mode == 1:
                self.set_position_mode()

    # ------------------------------------------------

    def arm_callback(self, msg):

        if not msg.data:
            return

        new_arm = int(msg.data[0])

        if new_arm != self.arm_command:

            names = {0: "DISARMED", 1: "ARMED"}

            self.get_logger().info(
                f"Arm switch: {names.get(self.arm_command)} -> {names.get(new_arm)}"
            )

            self.arm_command = new_arm

            if self.robot_mode == 0:

                if self.arm_command == 1:
                    self.arm_px4()
                else:
                    self.disarm_px4()

    # ------------------------------------------------

    def arm_px4(self):

        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("PX4 arm service not available")
            return

        req = CommandBool.Request()
        req.value = True

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response)

    # ------------------------------------------------

    def disarm_px4(self):

        req = CommandBool.Request()
        req.value = False

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response)

    # ------------------------------------------------

    def set_position_mode(self):

        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("PX4 set_mode service not available")
            return

        req = SetMode.Request()
        req.custom_mode = "POSCTL"

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_response)

    # ------------------------------------------------

    def arm_response(self, future):

        try:
            result = future.result()

            if result.success:
                self.get_logger().info("PX4 Arm command accepted")
            else:
                self.get_logger().warn("PX4 Arm rejected — check PX4 statustext")

        except Exception as e:
            self.get_logger().error(str(e))

    # ------------------------------------------------

    def mode_response(self, future):

        try:
            result = future.result()

            if result.mode_sent:
                self.get_logger().info("PX4 switched to POSITION mode")
            else:
                self.get_logger().warn("PX4 mode change rejected")

        except Exception as e:
            self.get_logger().error(str(e))

    # ------------------------------------------------

    def state_callback(self, msg):

        self.fc_connected = msg.connected
        self.fc_armed = msg.armed
        self.fc_mode = msg.mode

    # ------------------------------------------------

    def rc_cmd_callback(self, msg):

        if len(msg.data) < 4:
            return

        roll = msg.data[0]
        pitch = msg.data[1]
        throttle = msg.data[2]
        yaw = msg.data[3]

        # PX4 handles RC flight control directly
        # This is only for monitoring/logging

    # ------------------------------------------------


def main(args=None):

    rclpy.init(args=args)

    node = UAVControlNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()