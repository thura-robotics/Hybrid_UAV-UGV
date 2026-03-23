#!/usr/bin/env python3

"""

Subscribe: 
/robot/mode --- std_msgs/Float32MultiArray
/robot/flight_mode --- std_msgs/Float32MultiArray
/robot/arm_command --- std_msgs/Float32MultiArray
/mavros/state --- mavros_msgs/State
/mavros/statustext/recv --- mavros_msgs/StatusText
/uav/rc_commands --- std_msgs/Float32MultiArray --- [roll, pitch, throttle, yaw]

Services:
/mavros/cmd/arming
/mavros/set_mode


"""


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

        # emergency stop flag
        self.estop_active = False

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
        # self.create_subscription(
        #     Float32MultiArray,
        #     '/robot/emergency_stop',
        #     self.estop_callback,
        #     10
        # )
        
        self.status_pub = self.create_publisher(Float32MultiArray, '/uav/status', 10)
        self.create_timer(0.1, self.publish_status)

        self.get_logger().info("UAV Control Node Started")



    def publish_status(self):
            msg = Float32MultiArray()
            msg.data = [
                float(self.fc_armed),
                float(self.fc_connected),
                float(self.flight_mode),
                float(self.robot_mode)
            ]
            self.status_pub.publish(msg)
    # -------------Robot mode 

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

    # ----------PX4 status text
    def status_text_callback(self, msg):

        # Print PX4 warning messages
        if msg.severity <= 4:  # warning or error
            self.get_logger().error(f"PX4: {msg.text}")
        else:
            self.get_logger().info(f"PX4: {msg.text}")


 #flight mode
    
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

            if self.robot_mode == 0:
                if self.flight_mode == 1:
                    self.set_position_mode()
                elif self.flight_mode == 0:
                    self.set_manual_mode()

    # ---------mode switch--------------------
    def set_manual_mode(self):

        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("PX4 set_mode service not available")
            return

        req = SetMode.Request()
        req.custom_mode = "MANUAL"

        future = self.mode_client.call_async(req)

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
            if self.robot_mode != 0:
                return

            if self.arm_command == 0:
                self.disarm_px4()
                return

            if self.arm_command == 1:
                if not self.fc_connected:
                    self.get_logger().warn("Cannot arm: FCU not connected")
                    return

               
                if self.fc_mode != "POSCTL":
                    self.get_logger().warn(f"Cannot arm: must be in POSITION mode first (current: {self.fc_mode})")
                    return

                # 3. Avoid duplicate arm
                if self.fc_armed:
                    self.get_logger().info("Already armed")
                    return

                # 4. Try arming
                self.get_logger().info("Sending ARM command")
                self.arm_px4()


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
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Arm service unavailable — cannot disarm!")
            return
        req = CommandBool.Request()
        req.value = False

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response)

    
    # Emergency Stop
    
    # def estop_callback(self, msg):

    #     if not msg.data:
    #         return

    #     estop = int(msg.data[0])

    #     if estop == 1:

    #         self.get_logger().error("EMERGENCY STOP TRIGGERED")

    #         self.estop_active = True

    #         self.disarm_px4()

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

    # ------PX4 state----------------

    def state_callback(self, msg):
        # if self.fc_armed and not msg.armed:
        #     self.get_logger().warn("PX4 DISARMED")
        self.fc_connected = msg.connected
        self.fc_armed = msg.armed
        self.fc_mode = msg.mode

    # --------RC commands monitor------------------

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