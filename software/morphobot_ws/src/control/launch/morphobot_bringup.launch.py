#!/usr/bin/env python3
"""
Morphobot Full System Bringup Launch File
Launches all subsystems in one go:
  1. MAVROS (connected to PX4 flight controller)
  2. PX4 RC Bridge (RC input → ROS topics)
  3. ROS2 Control (servo hardware interface + controllers)
  4. UGV Control Node (differential drive from RC)
  5. Morphing Control Node (servo sequences for mode transitions)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Declare arguments ──────────────────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(
            "fcu_url",
            default_value="/dev/ttyACM0:921600",
            description="FCU connection URL for MAVROS",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB1",
            description="Serial port for ST3215 servos",
        ),
    ]

    fcu_url = LaunchConfiguration("fcu_url")
    serial_port = LaunchConfiguration("serial_port")

    # ══════════════════════════════════════════════════════════════════
    # 1. MAVROS
    # ══════════════════════════════════════════════════════════════════
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mavros"),
                "launch",
                "px4.launch",
            ])
        ]),
        launch_arguments={"fcu_url": fcu_url}.items(),
    )

    # ══════════════════════════════════════════════════════════════════
    # 2. PX4 RC Bridge (delay slightly for MAVROS to start)
    # ══════════════════════════════════════════════════════════════════
    px4_rc_bridge_node = Node(
        package="control",
        executable="px4_rc_bridge.py",
        name="px4_rc_bridge",
        output="both",
    )

    delay_rc_bridge = TimerAction(
        period=3.0,
        actions=[px4_rc_bridge_node],
    )

    # ══════════════════════════════════════════════════════════════════
    # 2.5 ST3215 Service Node (Must start before ROS2 Control)
    # ══════════════════════════════════════════════════════════════════
    st3215_service_node = Node(
        package="ugv_motor_driver",
        executable="st3215_service_node.py",
        name="st3215_service_node",
        output="both",
        parameters=[{"serial_port": serial_port}]
    )

    # ══════════════════════════════════════════════════════════════════
    # 3. ROS2 Control (include the existing robot_control launch)
    # ══════════════════════════════════════════════════════════════════
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ugv_motor_driver"),
                "launch",
                "robot_control.launch.py",
            ])
        ]),
        launch_arguments={"serial_port": serial_port}.items(),
    )
    
    # Delay ROS2 Control slightly to ensure ST3215 Service Node is ready
    delay_ros2_control = TimerAction(
        period=3.0,
        actions=[ros2_control_launch],
    )

    # ══════════════════════════════════════════════════════════════════
    # 4. UGV Control Node (delay for controllers to be ready)
    # ══════════════════════════════════════════════════════════════════
    ugv_control_node = Node(
        package="control",
        executable="ugv_control_node.py",
        name="ugv_control_node",
        output="both",
    )

    delay_ugv_control = TimerAction(
        period=15.0,  # Wait for ROS2 Control + controllers to initialize
        actions=[ugv_control_node],
    )

    # ══════════════════════════════════════════════════════════════════
    # 5. Morphing Control Node (delay for controllers to be ready)
    # ══════════════════════════════════════════════════════════════════
    morphing_control_node = Node(
        package="control",
        executable="morphing_control_node.py",
        name="morphing_control_node",
        output="both",
    )

    delay_morphing_control = TimerAction(
        period=15.0,  # Wait for ROS2 Control + controllers to initialize
        actions=[morphing_control_node],
    )

    # ══════════════════════════════════════════════════════════════════
    # 6. Safety Monitor Node
    # ══════════════════════════════════════════════════════════════════
    safety_monitor_node = Node(
        package="control",
        executable="safety_monitor_node.py",
        name="safety_monitor_node",
        output="both",
    )

    # ══════════════════════════════════════════════════════════════════
    # 7. RViz (Digital Twin Visualization)
    # ══════════════════════════════════════════════════════════════════
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("morphobot_urdf"), "config", "morphobot.rviz"]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    # )

    # # Delay RViz slightly to allow the robot_state_publisher from ros2_control to initialize
    # delay_rviz = TimerAction(
    #     period=5.0,
    #     actions=[rviz_node],
    # )

    # ══════════════════════════════════════════════════════════════════

    return LaunchDescription(
        declared_arguments
        + [
            # MAVROS starts immediately
            mavros_launch,
            # ST3215 Service Node starts immediately
            st3215_service_node,
            # Safety Monitor starts immediately
            safety_monitor_node,
            # Delay ROS2 Control for ST3215 service node to start
            delay_ros2_control,
            # PX4 RC Bridge after MAVROS has time to start
            delay_rc_bridge,
            # Control nodes after controllers are active
            delay_ugv_control,
            delay_morphing_control
            # RViz visualization
            # delay_rviz,
        ]
    )


