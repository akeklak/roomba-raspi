"""Bring up the whole robot base: Roomba driver, lidar, robot model, power guard, Foxglove bridge.

Usage on the Pi:
    ros2 launch roomba_bringup robot.launch.py
    ros2 launch roomba_bringup robot.launch.py use_lidar:=false   # Roomba only (day-one test)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    share = get_package_share_directory('roomba_bringup')

    def cfg(name):
        return os.path.join(share, 'config', name)

    roomba_port = LaunchConfiguration('roomba_port')
    lidar_port = LaunchConfiguration('lidar_port')
    use_lidar = LaunchConfiguration('use_lidar')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_power_guard = LaunchConfiguration('use_power_guard')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'roomba.urdf.xacro')]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    create_driver = Node(
        package='create_driver',
        executable='create_driver',
        name='create_driver',
        output='screen',
        parameters=[cfg('create_driver.yaml'), {'dev': roomba_port}],
    )

    lidar_container = ComposableNodeContainer(
        condition=IfCondition(use_lidar),
        name='lidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='ldlidar_component',
                plugin='ldlidar::LdLidarComponent',
                name='ldlidar_node',
                parameters=[cfg('ldlidar.yaml'), {'comm.serial_port': lidar_port}],
                # The component publishes on ~/scan; everything else expects /scan.
                remappings=[('/ldlidar_node/scan', '/scan')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    # The lidar is a lifecycle node: this configures and activates it automatically.
    lidar_lifecycle = Node(
        condition=IfCondition(use_lidar),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lidar_lifecycle_manager',
        output='screen',
        parameters=[cfg('lidar_lifecycle.yaml')],
    )

    power_guard = Node(
        condition=IfCondition(use_power_guard),
        package='roomba_bringup',
        executable='power_guard',
        name='power_guard',
        output='screen',
        parameters=[cfg('power_guard.yaml')],
    )

    # Lets Foxglove (app or foxglove.dev in a browser) show the scan, TF and robot model
    # at ws://<pi-address>:8765 until our own web UI exists.
    foxglove_bridge = Node(
        condition=IfCondition(use_foxglove),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{'port': 8765}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('roomba_port', default_value='/dev/roomba',
                              description='Serial device of the Roomba USB cable'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ldlidar',
                              description='Serial device of the LD19 lidar adapter'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('use_power_guard', default_value='true'),
        robot_state_publisher,
        create_driver,
        lidar_container,
        lidar_lifecycle,
        power_guard,
        foxglove_bridge,
    ])
