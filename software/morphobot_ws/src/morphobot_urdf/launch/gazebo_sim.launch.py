import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = "morphobot_urdf"

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(package_name),
            "urdf",
            "hybrid_robot.urdf.xacro",
        ]),
        " ",
        "sim_mode:=true",
    ])

    robot_description = {"robot_description": robot_description_content}

    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
    launch_arguments={"gz_args": "-r empty.sdf"}.items(),
)

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "morphobot",
            "-topic", "robot_description"
        ],
        output="screen",
    )

    spawn_robot_delayed = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot_delayed
    ])