#!/usr/bin/env python3
"""
Launch file for ST3215 hardware interface with ROS 2 Control.
Starts the hardware interface, controller manager, and controllers.
"""

import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("hybrid_robot_hardware"),
                    "urdf",
                    "robot.urdf.xacro",
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

    # Controller manager node
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

    # Velocity controller spawner (for driving servo)
    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    # Position controller spawner (for pan/tilt servos)
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay controller spawners until joint_state_broadcaster is active
    delay_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )

    delay_position_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=velocity_controller_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_velocity_controller_spawner,
        delay_position_controller_spawner,
    ]

    return LaunchDescription(nodes)
