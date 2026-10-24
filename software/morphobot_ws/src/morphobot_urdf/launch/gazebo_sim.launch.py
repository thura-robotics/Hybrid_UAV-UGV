import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = "morphobot_urdf"
    pkg_share = get_package_share_directory(package_name)
    model_path = os.path.dirname(pkg_share)  # points to install/.../share

    set_env_vars = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH',
        model_path
    )

    set_gz_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        model_path
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare(package_name),
            "urdf",
            "hybrid_robot_sim.urdf.xacro",
        ]),
        " ",
        "sim_mode:=true",
    ])

    robot_description = {"robot_description": robot_description_content}

    # Start Ignition Gazebo (Fortress)
    # Using 'gz_args' with '-r' to run unpaused. 
    # Adding '-s' (headless) can be done if GUI issues persist.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf",
            "on_exit_shutdown": "True"
        }.items(),
    )

    # Path to sim controllers config
    controllers_yaml = os.path.join(pkg_share, "config", "sim_controllers.yaml")

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    # Bridge for /clock and potentially others
    # Important: Map world clock to /clock for ROS 2 nodes
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        remappings=[('/world/empty/clock', '/clock')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "morphobot",
            "-topic", "robot_description",
            "-z", "0.1" # Adjusted height slightly
        ],
        output="screen",
    )

    spawn_robot_delayed = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    # Controllers
    # Joint State Broadcaster (starts first)
    node_spawner_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    # Position Controller (starts after 2s)
    node_spawner_position_controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawner_position_controller",
                arguments=["position_controller", "-c", "/controller_manager"],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    # Velocity Controller (starts after 4s)
    node_spawner_velocity_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                name="spawner_velocity_controller",
                arguments=["velocity_controller", "-c", "/controller_manager"],
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    # Delay controllers to ensure robot is spawned and plugin is active
    spawn_controllers_delayed = TimerAction(
        period=10.0,
        actions=[
            node_spawner_joint_state_broadcaster,
            node_spawner_position_controller,
            node_spawner_velocity_controller
        ]
    )

    return LaunchDescription([
        set_env_vars,
        set_gz_env_vars,
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot_delayed,
        spawn_controllers_delayed
    ])