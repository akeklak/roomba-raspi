"""A saved map (map_server) and AMCL, which works out where the robot is on it. map_manager starts
and stops this; to run it by hand:

    ros2 launch roomba_nav localization.launch.py map:=$HOME/roomba_maps/home.yaml initial_pose:=0,0,0

map_manager passes autostart:=false and brings the nodes up itself (see lifecycle.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    params = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    x, y, yaw = (float(v) for v in LaunchConfiguration('initial_pose').perform(context).split(','))
    autostart = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time,
                                  'yaml_filename': LaunchConfiguration('map').perform(context)}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time, 'set_initial_pose': True,
                                  'initial_pose.x': x, 'initial_pose.y': y, 'initial_pose.z': 0.0,
                                  'initial_pose.yaw': yaw}]),
    ] + ([
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server', 'amcl'], 'bond_timeout': 10.0}]),
    ] if autostart else [])


def generate_launch_description():
    share = get_package_share_directory('roomba_nav')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true', description='Configure and activate the nodes'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('map', description='The saved map\'s .yaml file'),
        DeclareLaunchArgument('initial_pose', default_value='0,0,0',
                              description='Where the robot is on the map: x,y,yaw'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        OpaqueFunction(function=nodes),
    ])
