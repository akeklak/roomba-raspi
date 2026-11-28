"""Bring up the whole robot: Roomba driver, lidar, robot model, power guard, web bridge, and
mapping and navigation (roomba_nav).

Usage on the Pi:
    ros2 launch roomba_bringup robot.launch.py
    ros2 launch roomba_bringup robot.launch.py use_lidar:=false   # Roomba only (day-one test)
    ros2 launch roomba_bringup robot.launch.py use_nav:=false     # no mapping or navigation
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    use_web_bridge = LaunchConfiguration('use_web_bridge')
    use_power_guard = LaunchConfiguration('use_power_guard')
    use_nav = LaunchConfiguration('use_nav')

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

    # Our web UI connects here (ws://<pi-address>:8765) to see the robot and drive it.
    web_bridge = Node(
        condition=IfCondition(use_web_bridge),
        package='roomba_web',
        executable='web_bridge',
        name='web_bridge',
        output='screen',
    )

    # Mapping (slam_toolbox), localization (AMCL) and Nav2, run by roomba_nav's map_manager. Maps
    # are saved in ~/roomba_maps.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('roomba_nav'), 'launch', 'nav.launch.py')),
        condition=IfCondition(use_nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument('roomba_port', default_value='/dev/roomba',
                              description='Serial device of the Roomba USB cable'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ldlidar',
                              description='Serial device of the LD19 lidar adapter'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_web_bridge', default_value='true'),
        DeclareLaunchArgument('use_power_guard', default_value='true'),
        DeclareLaunchArgument('use_nav', default_value=LaunchConfiguration('use_lidar'),
                              description='Mapping and navigation (needs the lidar)'),
        robot_state_publisher,
        create_driver,
        lidar_container,
        lidar_lifecycle,
        power_guard,
        web_bridge,
        nav,
    ])
