"""Simulated robot in Gazebo Harmonic with the same ROS topics as the real one.

    ros2 launch roomba_sim sim.launch.py                 # headless (Docker / Mac / server)
    ros2 launch roomba_sim sim.launch.py headless:=false # with the Gazebo window (Linux desktop)

View and drive it from Foxglove at ws://localhost:8765.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('roomba_sim')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_foxglove = LaunchConfiguration('use_foxglove')

    # -r: start running immediately. Headless adds -s (server only) and renders sensors offscreen.
    gz_args = PythonExpression([
        "('-r -s --headless-rendering ' if '", headless, "' == 'true' else '-r ')",
        " + '", world, "'",
    ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'roomba_sim.urdf.xacro')]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description', '-name', 'roomba',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
            '-z', '0.01', '-Y', LaunchConfiguration('yaw'),
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765, 'address': '0.0.0.0', 'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'apartment.sdf')),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        # Spawn just east of the dock, facing east.
        DeclareLaunchArgument('x', default_value='-3.4'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        foxglove_bridge,
    ])
