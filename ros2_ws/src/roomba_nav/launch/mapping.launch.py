"""slam_toolbox, building a map from the lidar. map_manager starts and stops this; to run it by
hand instead (with navigation.launch.py for driving):

    ros2 launch roomba_nav mapping.launch.py use_sim_time:=true

To add to a saved map instead of starting from scratch, give the saved map (path without
extension) and where the robot is on it:  map_file:=/path/home start_pose:=x,y,yaw
map_manager passes autostart:=false and brings slam_toolbox up itself (see lifecycle.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def slam_node(context):
    params = [LaunchConfiguration('params_file').perform(context), {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
    }]
    map_file = LaunchConfiguration('map_file').perform(context)
    if map_file:
        pose = [float(v) for v in LaunchConfiguration('start_pose').perform(context).split(',')]
        params.append({'map_file_name': map_file, 'map_start_pose': pose})

    # slam_toolbox is a lifecycle node: configure it, then activate it once configured.
    node = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
                         name='slam_toolbox', namespace='', output='screen', parameters=params)
    if LaunchConfiguration('autostart').perform(context).lower() != 'true':
        return [node]
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                            transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state='configuring', goal_state='inactive',
        entities=[
            LogInfo(msg='slam_toolbox configured; activating'),
            EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                        transition_id=Transition.TRANSITION_ACTIVATE)),
        ]))
    return [node, configure, activate]


def generate_launch_description():
    share = get_package_share_directory('roomba_nav')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true', description='Configure and activate slam_toolbox'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'slam.yaml')),
        DeclareLaunchArgument('map_file', default_value='',
                              description='Saved map to add to (path without extension); empty = new map'),
        DeclareLaunchArgument('start_pose', default_value='0,0,0',
                              description='Where the robot is on that map: x,y,yaw'),
        OpaqueFunction(function=slam_node),
    ])
