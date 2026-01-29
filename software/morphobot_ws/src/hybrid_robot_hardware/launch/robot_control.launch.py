#!/usr/bin/env python3
"""
Launch file for ST3215 hardware interface with ROS 2 Control.
Starts Python service node, hardware interface, controller manager, and controllers.
"""

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyUSB0",
            description="Serial port for ST3215 servos",
        )
    )
    
    serial_port = LaunchConfiguration("serial_port")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hybrid_robot_hardware"),
                    "urdf",
                    "hybrid_robot.urdf.xacro",
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Get controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hybrid_robot_hardware"),
            "config",
            "controllers.yaml",
        ]
    )

    # Get ST3215 service node parameters
    st3215_params = PathJoinSubstitution(
        [
            FindPackageShare("hybrid_robot_hardware"),
            "config",
            "st3215_params.yaml",
        ]
    )

    # ST3215 Service Node (Python) - MUST START FIRST!
    st3215_service_node = Node(
        package="hybrid_robot_hardware",
        executable="st3215_service_node.py",
        name="st3215_service_node",
        output="both",
        parameters=[st3215_params],
    )

    # Controller manager node (starts after service node)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Position controller spawner
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    # Velocity controller spawner
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay control node start until service node is ready (give it 3 seconds)
    delay_control_node_after_service = TimerAction(
        period=3.0,
        actions=[control_node],
    )

    # Delay position controller spawner after joint state broadcaster
    delay_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    # Delay velocity controller spawner after position controller
    delay_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controller_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    nodes = [
        st3215_service_node,
        delay_control_node_after_service,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_position_controller_spawner,
        delay_velocity_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
