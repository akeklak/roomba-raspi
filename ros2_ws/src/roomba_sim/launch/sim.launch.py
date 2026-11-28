"""Simulated robot in Gazebo Harmonic with the same ROS topics as the real one, plus mapping and
navigation (roomba_nav), exactly as the robot runs them.

    ros2 launch roomba_sim sim.launch.py                 # headless (Docker / Mac / server)
    ros2 launch roomba_sim sim.launch.py headless:=false # with the Gazebo window (Linux desktop)
    ros2 launch roomba_sim sim.launch.py start_mode:=mapping  # start a new map even if one is saved
    ros2 launch roomba_sim sim.launch.py localization:=ground_truth  # no SLAM or Nav2: Gazebo's
                                                         # true pose instead (localization:=odom: none)

View and drive it from the web UI (http://localhost:8080), which connects to our web bridge
at ws://localhost:8765.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('roomba_sim')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_web_bridge = LaunchConfiguration('use_web_bridge')

    # -r: start running immediately. Headless adds -s (server only) and renders sensors offscreen.
    gz_args = PythonExpression([
        "('-r -s --headless-rendering ' if '", headless, "' == 'true' else '-r ')",
        " + '", world, "'",
    ])
    # The robot wears the Roomba 675 meshes from roomba_bringup (package://roomba_bringup/...).
    # Gazebo looks those up as roomba_bringup/... under each folder in GZ_SIM_RESOURCE_PATH.
    mesh_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory('roomba_bringup')))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    # Keeps sim time in step with the wall clock while Gazebo renders the camera and lidar
    # (roomba_sim/realtime_pacer.py). It also sets 2 ms physics steps.
    realtime_pacer = Node(
        package='roomba_sim',
        executable='realtime_pacer',
        output='screen',
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

    # "world" is the apartment's frame. Wheel odometry ("odom") drifts when the wheels slip.
    # localization:=slam (default) runs mapping and navigation as on the robot: SLAM or AMCL
    # publish map -> odom, and the map frame starts where the robot spawned (the dock), so
    # world -> map is fixed at the spawn pose. Comparing the map with the 3D apartment in the web
    # UI then shows how well SLAM is doing.
    # localization:=ground_truth publishes world -> odom from Gazebo's true pose instead (perfect,
    # no Nav2); localization:=odom ties world to odom at the spawn pose and lets the drift show.
    world_to_map = Node(
        condition=LaunchConfigurationEquals('localization', 'slam'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        arguments=[
            '--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', 'world', '--child-frame-id', 'map',
        ],
        parameters=[{'use_sim_time': True}],
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('roomba_nav'), 'launch', 'nav.launch.py')),
        condition=LaunchConfigurationEquals('localization', 'slam'),
        launch_arguments={'use_sim_time': 'true', 'start_mode': LaunchConfiguration('start_mode')}.items(),
    )
    ground_truth_localizer = Node(
        condition=LaunchConfigurationEquals('localization', 'ground_truth'),
        package='roomba_sim',
        executable='ground_truth_localizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    world_to_odom = Node(
        condition=LaunchConfigurationEquals('localization', 'odom'),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=[
            '--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'),
            '--yaw', LaunchConfiguration('yaw'),
            '--frame-id', 'world', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # The apartment's walls and furniture, for drawing in the web UI.
    world_markers = Node(
        package='roomba_sim',
        executable='world_markers',
        output='screen',
        parameters=[{'world_file': world, 'frame_id': 'world'}],
    )

    # Carries the topics above to the web UI and its drive commands back (roomba_web).
    web_bridge = Node(
        condition=IfCondition(use_web_bridge),
        package='roomba_web',
        executable='web_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'apartment.sdf')),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('use_web_bridge', default_value='true'),
        DeclareLaunchArgument('localization', default_value='slam',
                              choices=['slam', 'ground_truth', 'odom'],
                              description='Where the robot\'s position comes from (see above)'),
        DeclareLaunchArgument('start_mode', default_value='auto',
                              choices=['auto', 'mapping', 'navigation', 'off'],
                              description='auto: navigate on the saved map if there is one, else map'),
        # Spawn just east of the dock, facing east.
        DeclareLaunchArgument('x', default_value='-3.4'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        mesh_path,
        gazebo,
        realtime_pacer,
        robot_state_publisher,
        spawn,
        bridge,
        world_to_map,
        nav,
        ground_truth_localizer,
        world_to_odom,
        world_markers,
        web_bridge,
    ])
