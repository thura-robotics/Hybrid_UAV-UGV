import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    use_gui = LaunchConfiguration("use_gui")
    
    declared_arguments = [
        DeclareLaunchArgument(
            "use_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for manual control",
        ),
    ]

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("morphobot_urdf"),
                    "urdf",
                    "hybrid_robot.urdf.xacro",
                ]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state publisher (GUI or regular)
    # If not using hardware, this provides the joint states
    joint_state_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(use_gui),
    )
    
    joint_state_pub_no_gui_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(use_gui),
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("morphobot_urdf"), "config", "morphobot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription(declared_arguments + [
        robot_state_pub_node,
        joint_state_pub_node,
        joint_state_pub_no_gui_node,
        rviz_node,
    ])
