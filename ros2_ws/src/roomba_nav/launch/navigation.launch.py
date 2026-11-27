"""Nav2's planner, controller and recovery behaviors. Started by nav.launch.py and always running:
it plans on /map whether that comes from mapping (slam_toolbox) or a saved map (map_server).

    ros2 launch roomba_nav navigation.launch.py use_sim_time:=true

Send it goals with the web UI, or: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose ...
Commands go controller -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> the wheels.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from roomba_nav.lifecycle import NAV2_NODES


def generate_launch_description():
    share = get_package_share_directory('roomba_nav')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    def nav2_node(package, executable, remappings=()):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=[params], remappings=list(remappings))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2.yaml')),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Bring the servers up right away. nav.launch.py sets false: '
                                          'map_manager brings them up once the robot is on a map'),
        # Print log lines as they happen rather than in bursts.
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        GroupAction([
            SetParameter('use_sim_time', use_sim_time),
            nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
            nav2_node('nav2_planner', 'planner_server'),
            nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
            nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                      [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            nav2_node('nav2_bt_navigator', 'bt_navigator'),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True, 'node_names': NAV2_NODES, 'bond_timeout': 10.0}],
                condition=IfCondition(LaunchConfiguration('autostart')),
            ),
        ]),
    ])
