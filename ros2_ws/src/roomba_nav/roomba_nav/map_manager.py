"""Runs mapping or localization, and saves and loads the map.

    ros2 run roomba_nav map_manager        (nav.launch.py starts it)

The robot is always in one of two modes:

  mapping     slam_toolbox builds a map from the lidar while the robot drives (you driving it, or
              Nav2 driving to spots you click). The map grows on /map as it goes.
  navigation  the saved map is loaded (Nav2's map_server) and AMCL keeps track of where the robot
              is on it by matching the lidar against the map.

Both publish /map and the map -> odom transform, so Nav2 (navigation.launch.py, always running)
plans and drives the same way in either mode.

Each mode is its own `ros2 launch` (mapping.launch.py or localization.launch.py), run as a child
process, so switching modes means stopping one and starting the other. When it starts, this node
navigates on the saved map if there is one, and starts mapping otherwise.

It also brings the lifecycle nodes up (see lifecycle.py): slam_toolbox, or map_server and AMCL,
each time a mode starts, and Nav2 once the robot is first placed on a map, because Nav2 can't
start before that.

Services (std_srvs/Trigger), used by the web UI through the web bridge:
  /map_manager/save_map       mapping: save the map so far (the previous save is kept as a backup)
  /map_manager/use_saved_map  stop mapping (saving first) and navigate on the saved map, keeping
                              the robot's position; or reload the saved map
  /map_manager/new_map        start a new map from scratch (the saved map stays until you save)
  /map_manager/extend_map     go back to mapping on top of the saved map, to add rooms to it

Its state is published as JSON on /map_manager/state (std_msgs/String, latched):
  {mode: "mapping" | "navigation" | "off", busy, message, error, extending, map_name,
   nav_ready, saved: {saved_at, width_m, height_m, can_extend} | null}
Maps are stored in `maps_dir` (see map_files.py).
"""

import json
import os
import threading
import time
from typing import Callable, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from roomba_nav.lifecycle import NAV2_NODES, Lifecycle
from roomba_nav.map_files import Grid, MapStore, yaw_from_quaternion
from roomba_nav.process import ChildProcess

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

Pose = Tuple[float, float, float]


def grid_from_msg(msg: OccupancyGrid) -> Grid:
    o = msg.info.origin
    q = o.orientation
    return Grid(
        width=msg.info.width,
        height=msg.info.height,
        resolution=msg.info.resolution,
        origin=(o.position.x, o.position.y, yaw_from_quaternion(q.x, q.y, q.z, q.w)),
        data=msg.data,
    )


def pose_arg(pose) -> str:
    return ','.join(f'{float(v):.4f}' for v in pose)


class MapManager(Node):

    def __init__(self) -> None:
        super().__init__('map_manager')
        share = get_package_share_directory('roomba_nav')
        p = self.declare_parameter
        maps_dir = p('maps_dir', '~/roomba_maps').value
        map_name = p('map_name', 'home').value
        self.start_mode = p('start_mode', 'auto').value  # auto | mapping | navigation | off
        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_frame', 'base_footprint').value
        self.ready_timeout = p('ready_timeout', 45.0).value
        self.slam_params = p('slam_params_file', os.path.join(share, 'config', 'slam.yaml')).value
        self.nav_params = p('nav_params_file', os.path.join(share, 'config', 'nav2.yaml')).value
        # Bring Nav2 up once the robot is on a map. False if something else starts Nav2.
        self.manage_nav2 = p('manage_nav2', True).value
        # Run mapping and localization on sim time. This node itself stays on wall time: following
        # the sim's 1 kHz /clock would cost it most of a CPU core, and it only needs timeouts.
        self.sim = p('sim', False).value or bool(self.get_parameter('use_sim_time').value)

        self.store = MapStore(maps_dir, map_name)
        self.child = ChildProcess(log=self.get_logger().info)
        self.op_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.state = {
            'mode': 'off', 'busy': None, 'message': '', 'error': None, 'extending': False,
            'nav_ready': False,
        }
        self.nav2_retry_at = 0.0
        # Where the robot was (in the map) when the current map was started. Saved with the map:
        # the robot usually starts there (on its dock), so it is where localization starts too.
        self.start_pose: Optional[List[float]] = None
        self.latest_map: Optional[OccupancyGrid] = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state_pub = self.create_publisher(String, '~/state', LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)

        services = ReentrantCallbackGroup()
        clients = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/save_map', self.service(self.save, 'Saving the map'),
                            callback_group=services)
        self.create_service(Trigger, '~/use_saved_map',
                            self.service(self.use_saved_map, 'Switching to the saved map'),
                            callback_group=services)
        self.create_service(Trigger, '~/new_map', self.service(self.new_map, 'Starting a new map'),
                            callback_group=services)
        self.create_service(Trigger, '~/extend_map',
                            self.service(self.extend_map, 'Adding to the saved map'),
                            callback_group=services)
        self.dynamic_map = self.create_client(GetMap, '/slam_toolbox/dynamic_map', callback_group=clients)
        self.lifecycle = Lifecycle(self, self.call, clients)
        self.serialize = None
        try:
            from slam_toolbox.srv import SerializePoseGraph
            self.serialize_type = SerializePoseGraph
            self.serialize = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map',
                                                callback_group=clients)
        except ImportError:
            self.get_logger().warn('slam_toolbox is not installed: maps are saved without the pose graph')

        self.create_timer(1.0, self.watch, callback_group=MutuallyExclusiveCallbackGroup())
        self.publish_state()
        threading.Thread(target=self.startup, daemon=True).start()

    # State ---------------------------------------------------------------------------------------

    def set_state(self, **changes) -> None:
        with self.state_lock:
            self.state.update(changes)
        self.publish_state()

    def publish_state(self) -> None:
        with self.state_lock:
            state = dict(self.state)
        info = self.store.info()
        state['map_name'] = self.store.name
        state['saved'] = None if info is None else {
            'saved_at': info.saved_at,
            'width_m': info.width_m,
            'height_m': info.height_m,
            'can_extend': info.can_extend,
        }
        self.state_pub.publish(String(data=json.dumps(state)))

    @property
    def mode(self) -> str:
        with self.state_lock:
            return self.state['mode']

    @property
    def nav_ready(self) -> bool:
        with self.state_lock:
            return self.state['nav_ready']

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def watch(self) -> None:
        """Notices if mapping or localization died, records where a new map started, and starts
        Nav2 if it didn't start when the mode did (say the lidar was late)."""
        if self.op_lock.locked():
            return
        mode = self.mode
        if mode in ('mapping', 'navigation') and self.child.exit_code() is not None:
            what = 'Mapping' if mode == 'mapping' else 'Localization'
            error = (f'{what} stopped unexpectedly (exit code {self.child.exit_code()}). '
                     'See the robot log for why, then start it again from the map panel.')
            self.get_logger().error(error)
            self.child.stop()
            self.set_state(mode='off', error=error)
        elif mode == 'mapping' and self.start_pose is None:
            pose = self.robot_pose(wait=0.0)
            if pose is not None:
                self.start_pose = [round(v, 4) for v in pose]
                self.get_logger().info(f'new map started with the robot at {self.start_pose}')
        if (self.mode in ('mapping', 'navigation') and self.manage_nav2 and not self.nav_ready
                and time.monotonic() > self.nav2_retry_at and self.robot_pose(wait=0.0) is not None):
            self.run(lambda: self.start_nav2() or (True, 'Navigation is ready.'), 'Starting navigation')

    # Operations (one at a time) -------------------------------------------------------------------

    def service(self, operation: Callable[[], Tuple[bool, str]], label: str):
        def handle(request, response):
            response.success, response.message = self.run(operation, label)
            return response
        return handle

    def run(self, operation: Callable[[], Tuple[bool, str]], label: str) -> Tuple[bool, str]:
        if not self.op_lock.acquire(blocking=False):
            with self.state_lock:
                busy = self.state['busy']
            return False, f'Busy ({busy}). Try again in a moment.'
        try:
            self.set_state(busy=label)
            ok, message = operation()
        except Exception as e:  # noqa: BLE001 - report anything to the UI rather than crash
            self.get_logger().error(f'{label} failed: {e!r}')
            ok, message = False, f'{label} failed: {e}'
        finally:
            self.op_lock.release()
        (self.get_logger().info if ok else self.get_logger().warn)(message)
        self.set_state(busy=None, message=message)
        return ok, message

    def startup(self) -> None:
        mode = self.start_mode
        if mode == 'auto':
            mode = 'navigation' if self.store.exists() else 'mapping'
        if mode == 'navigation' and not self.store.exists():
            self.get_logger().warn(f'no saved map at {self.store.yaml_path}; starting a new map')
            mode = 'mapping'
        if mode == 'navigation':
            self.run(self.use_saved_map, 'Loading the saved map')
        elif mode == 'mapping':
            self.run(self.new_map, 'Starting a new map')

    def save(self) -> Tuple[bool, str]:
        if self.mode != 'mapping':
            return False, 'Not mapping right now, so there is nothing new to save.'
        grid = self.current_map()
        if grid is None or grid.width == 0:
            return False, 'There is no map yet. Drive around a little first.'
        self.store.begin_save()
        has_graph = self.serialize_pose_graph(self.store.temp_prefix)
        self.store.write_grid(grid, self.start_pose or [0.0, 0.0, 0.0])
        self.store.commit()
        size = f'{grid.width * grid.resolution:.1f} m × {grid.height * grid.resolution:.1f} m'
        note = '' if has_graph else ' It could not save the SLAM pose graph, so it cannot be extended later.'
        self.get_logger().info(f'saved map to {self.store.prefix}.*')
        return True, f'Map saved ({size}).{note}'

    def start_nav2(self) -> Optional[Tuple[bool, str]]:
        """Activates Nav2 if it isn't yet. None once it runs, else what went wrong."""
        if not self.manage_nav2 or self.nav_ready:
            return None
        failed = self.lifecycle.bring_up(NAV2_NODES, timeout=90.0)
        if failed is None:
            self.get_logger().info('Nav2 is up')
            self.set_state(nav_ready=True)
            return None
        self.nav2_retry_at = time.monotonic() + 30.0
        return False, f'Navigation (Nav2) did not start: {failed} is not active. See the robot log for why.'

    def use_saved_map(self) -> Tuple[bool, str]:
        mode = self.mode
        pose = None
        if mode == 'mapping':
            pose = self.robot_pose()
            ok, message = self.save()
            if not ok:
                return False, message
        elif mode == 'navigation':
            pose = self.robot_pose()
        info = self.store.info()
        if info is None:
            return False, 'There is no saved map yet. Make one first.'
        error = self.start_navigation(pose or info.start_pose) or self.start_nav2()
        if error:
            return error
        return True, 'Navigating on the saved map.'

    def new_map(self) -> Tuple[bool, str]:
        error = self.start_mapping() or self.start_nav2()
        if error:
            return error
        if self.store.exists():
            return True, 'Started a new map. The saved map stays until you save this one.'
        return True, 'Started a new map. Drive around, then save it.'

    def extend_map(self) -> Tuple[bool, str]:
        info = self.store.info()
        if info is None:
            return False, 'There is no saved map to add to yet.'
        if not info.can_extend:
            return False, 'This map was saved without the SLAM pose graph, so it cannot be added to.'
        pose = self.robot_pose() if self.mode == 'navigation' else None
        error = self.start_mapping(extend_start=info.start_pose, pose=pose or info.start_pose) or self.start_nav2()
        if error:
            return error
        return True, 'Mapping again, adding to the saved map. Save when you are done.'

    # Modes ---------------------------------------------------------------------------------------

    def launch_args(self, launch_file: str, params_file: str) -> List[str]:
        # autostart:=false because start() brings the nodes up itself.
        return ['ros2', 'launch', 'roomba_nav', launch_file, 'autostart:=false',
                f'use_sim_time:={"true" if self.sim else "false"}', f'params_file:={params_file}']

    def start(self, argv: List[str], nodes: List[str], what: str) -> Optional[Tuple[bool, str]]:
        """Starts a mode's launch file and brings its nodes up. None once the robot is placed on
        the map (a map -> odom transform from the new nodes), else what went wrong."""
        self.child.stop()
        time.sleep(0.5)  # let the old process's last transforms arrive
        previous = self.map_odom_stamp()  # so placed means newer than this
        self.latest_map = None
        self.child.start(argv)
        failed = self.lifecycle.bring_up(nodes, self.ready_timeout, self.child.running)
        if failed is not None:
            if not self.child.running():
                return False, f'{what} stopped while starting. See the robot log for why.'
            return False, f'{what} did not start: {failed} is not active. See the robot log for why.'
        if not self.wait_until_placed(previous):
            return False, f'{what} started, but has not placed the robot on the map yet. Is the lidar running?'
        return None

    def start_mapping(self, extend_start: Optional[List[float]] = None, pose=None) -> Optional[Tuple[bool, str]]:
        """Starts slam_toolbox. None once it publishes where the robot is, else what went wrong."""
        argv = self.launch_args('mapping.launch.py', self.slam_params)
        if extend_start is not None:
            argv += [f'map_file:={self.store.prefix}', f'start_pose:={pose_arg(pose)}']
        self.start_pose = list(extend_start) if extend_start is not None else None
        self.set_state(mode='mapping', extending=extend_start is not None, error=None)
        return self.start(argv, ['slam_toolbox'], 'Mapping')

    def start_navigation(self, pose) -> Optional[Tuple[bool, str]]:
        """Starts map_server and AMCL. None once AMCL publishes where the robot is."""
        argv = self.launch_args('localization.launch.py', self.nav_params)
        argv += [f'map:={self.store.yaml_path}', f'initial_pose:={pose_arg(pose)}']
        self.set_state(mode='navigation', extending=False, error=None)
        return self.start(argv, ['map_server', 'amcl'], 'Localization')

    def map_odom_stamp(self) -> Optional[Time]:
        """Stamp of the newest map -> odom transform (from SLAM or AMCL), or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.odom_frame, Time())
        except TransformException:
            return None
        return Time.from_msg(t.header.stamp)

    def wait_until_placed(self, previous: Optional[Time]) -> bool:
        """Waits for a map -> odom transform newer than `previous`, i.e. from the process just
        started rather than the one before it. Nav2 can't drive until there is one."""
        end = time.monotonic() + self.ready_timeout
        while time.monotonic() < end and self.child.running():
            stamp = self.map_odom_stamp()
            if stamp is not None and (previous is None or stamp > previous):
                return True
            time.sleep(0.2)
        return False

    # ROS helpers ---------------------------------------------------------------------------------

    def robot_pose(self, wait: float = 1.0) -> Optional[Pose]:
        """The robot's (x, y, yaw) on the map, or None if it isn't known right now."""
        try:
            if not self.tf_buffer.can_transform(self.map_frame, self.base_frame, Time(),
                                                timeout=Duration(seconds=wait)):
                return None
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return (tr.x, tr.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def call(self, client, request, timeout: float):
        if client is None or not client.wait_for_service(timeout_sec=2.0):
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            client.remove_pending_request(future)
            return None
        return future.result()

    def current_map(self) -> Optional[Grid]:
        """The newest map from slam_toolbox (asked for fresh, else the last one it published)."""
        response = self.call(self.dynamic_map, GetMap.Request(), timeout=10.0)
        if response is not None and response.map.info.width > 0:
            return grid_from_msg(response.map)
        return grid_from_msg(self.latest_map) if self.latest_map is not None else None

    def serialize_pose_graph(self, prefix: str) -> bool:
        if self.serialize is None:
            return False
        request = self.serialize_type.Request()
        request.filename = prefix
        response = self.call(self.serialize, request, timeout=30.0)
        ok = (response is not None and response.result == 0
              and os.path.isfile(prefix + '.posegraph') and os.path.isfile(prefix + '.data'))
        if not ok:
            self.get_logger().warn('could not save the SLAM pose graph')
        return ok

    def shutdown(self) -> None:
        self.child.stop()


def main() -> None:
    rclpy.init()
    node = MapManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
