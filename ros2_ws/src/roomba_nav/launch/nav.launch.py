"""Mapping, localization and navigation: map_manager plus Nav2. The robot (roomba_bringup's
robot.launch.py) and the sim (roomba_sim's sim.launch.py) both include this.

    ros2 launch roomba_nav nav.launch.py                    # on the robot, once the base is up
    ros2 launch roomba_nav nav.launch.py start_mode:=mapping

Maps are saved in maps_dir (default: $ROOMBA_MAPS_DIR, else ~/roomba_maps).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('roomba_nav'), 'launch', 'navigation.launch.py'])),
        # map_manager brings Nav2 up once the robot is on a map (it can't start before that).
        launch_arguments={'use_sim_time': use_sim_time, 'autostart': 'false'}.items(),
    )
    map_manager = Node(
        package='roomba_nav',
        executable='map_manager',
        name='map_manager',
        output='screen',
        parameters=[{
            # Not use_sim_time: map_manager runs on wall time and passes sim time on to what it starts.
            'sim': use_sim_time,
            'maps_dir': LaunchConfiguration('maps_dir'),
            'map_name': LaunchConfiguration('map_name'),
            'start_mode': LaunchConfiguration('start_mode'),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('maps_dir', default_value=EnvironmentVariable(
            'ROOMBA_MAPS_DIR', default_value=os.path.join(os.path.expanduser('~'), 'roomba_maps'))),
        DeclareLaunchArgument('map_name', default_value='home'),
        DeclareLaunchArgument('start_mode', default_value='auto', choices=['auto', 'mapping', 'navigation', 'off'],
                              description='auto: use the saved map if there is one, else start mapping'),
        navigation,
        map_manager,
    ])
