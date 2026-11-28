"""Websocket bridge between ROS and the Roomba web UI.

    ros2 run roomba_web web_bridge          (both launch files start it)

Serves ws://<robot>:8765. It subscribes to the topics the UI draws, sends them to every
connected page as JSON (camera pictures as JPEG, maps as compressed grids), publishes /cmd_vel
from the page's drive buttons, sends navigation goals to Nav2, and passes map commands (save,
switch to the saved map, new map) to map_manager. The message format is described in
roomba_web/protocol.py.

Safety: the page repeats its drive command ten times a second while a key or button is held.
If those messages stop for `drive_timeout` seconds (tab closed, wifi dropped), the robot stops.
Driving by hand also cancels any navigation goal, so the two never fight over the wheels.
The page's top speed (the speed slider) applies to both: it scales the drive keys on the page,
and the bridge passes it to Nav2 as a speed limit for Go to.

ROS runs in a background thread; the websocket server runs on asyncio in the main thread. Each
page gets the newest value of each kind of message, so a slow connection skips old scans and
pictures instead of falling behind. Anything that calls into ROS (goals, services) is handed to
the ROS thread through `in_ros`.
"""

import asyncio
import collections
import json
import math
import queue
import threading
import time

import rclpy
import websockets
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from roomba_web import protocol

MAX_QUEUED_TRANSFORMS = 500
NAV_FEEDBACK_PERIOD = 0.25

LATCHED = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
LATCHED_ONE = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

# Streams a page only gets while it asks for them (they are the heaviest).
OPTIONAL_STREAMS = ('camera', 'costmap')
# How far ahead hand driving looks for obstacles (enough to slow down from top speed), in meters.
DRIVE_HORIZON = 0.3
# Older than this, the lidar has stopped and hand driving isn't held back.
SCAN_MAX_AGE = 0.5


class Client:
    """One connected page."""

    def __init__(self, ws) -> None:
        self.ws = ws
        self.wants = set()     # which of OPTIONAL_STREAMS this page wants
        self.slots = {}        # kind -> newest frame (str or bytes)
        self.transforms = []   # every transform since the last send
        self.wake = asyncio.Event()

    def put(self, kind: str, data) -> None:
        self.slots[kind] = data
        self.wake.set()

    def put_transforms(self, items: list) -> None:
        self.transforms.extend(items)
        if len(self.transforms) > MAX_QUEUED_TRANSFORMS:
            del self.transforms[:-MAX_QUEUED_TRANSFORMS]
        self.wake.set()


class WebBridge(Node):

    def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
        super().__init__('web_bridge')
        self.loop = loop
        p = self.declare_parameter
        self.host = p('host', '0.0.0.0').value
        self.port = p('port', 8765).value
        self.camera_period = 1.0 / p('camera_fps', 10.0).value
        self.jpeg_quality = p('jpeg_quality', 70).value
        self.drive_timeout = p('drive_timeout', 0.5).value
        self.max_linear = p('max_linear', 0.5).value
        self.max_angular = p('max_angular', 2.0).value
        self.map_frame = p('map_frame', 'map').value
        # Hand driving stops this far short of what the lidar sees (see protocol.free_travel).
        self.base_frame = p('base_frame', 'base_footprint').value
        self.robot_radius = p('robot_radius', 0.17).value
        self.drive_margin = p('drive_margin', 0.02).value
        camera_topic = p('camera_topic', '/camera/image_raw').value
        camera_info_topic = p('camera_info_topic', '/camera/camera_info').value

        self.pages = set()
        # Sent to each page when it connects, since these rarely or never repeat.
        self.static_transforms = {}
        self.latched = {}
        self.optional_latest = {}  # stream -> newest frame, sent when a page starts wanting it
        self.camera_model = None
        self.last_camera = 0.0
        self.wanted = set()        # optional streams at least one page wants
        self.driving = False
        self.last_drive = 0.0
        self.obstacles = (0.0, [])  # (when, lidar hits near the robot), from the newest scan
        self.odom_pose = None       # (x, y, yaw) from the newest /odom
        # (odom x, y, yaw, lidar points) per scan for the last 2.5 s, to tell when it's stuck
        self.motion = collections.deque(maxlen=25)
        self.stuck = None           # (direction it can't drive, yaw then) after it got stuck
        self.last_blocked_notice = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static, LATCHED)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(MarkerArray, '/sim/world', self.on_world, LATCHED_ONE)
        self.create_subscription(Float32, '/battery/charge_ratio', self.on_battery, 10)
        # The camera is only subscribed while a page shows it (see update_wanted). In the sim that
        # is what makes Gazebo render it, which costs real-time speed.
        self.camera_topics = (camera_topic, camera_info_topic)
        self.camera_subs = []
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED_ONE)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.on_costmap, LATCHED_ONE)
        self.create_subscription(Path, '/plan', self.on_plan, 1)
        self.create_subscription(String, '/map_manager/state', self.on_mapping_state, LATCHED_ONE)
        try:  # Only the real robot has this message type (from create_robot).
            from create_msgs.msg import ChargingState
            self.create_subscription(ChargingState, '/battery/charging_state', self.on_charging, 10)
        except ImportError:
            pass

        # Navigation (Nav2) and map commands (map_manager). Either may be missing or not running.
        self.map_clients = {c: self.create_client(Trigger, f'/map_manager/{c}') for c in protocol.MAP_COMMANDS}
        try:
            from nav2_msgs.action import NavigateToPose
            from nav2_msgs.msg import SpeedLimit
            self.NavigateToPose = NavigateToPose
            self.SpeedLimit = SpeedLimit
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            # Nav2's controller drives at most this fast (absolute, m/s): the page's top speed.
            self.speed_limit_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
        except ImportError:
            self.NavigateToPose = None
            self.nav_client = None
            self.speed_limit_pub = None
        self.top_speed = None      # from the page's speed slider; None: leave Nav2's own speed
        self.goal_seq = 0          # bumped per goal; replies for older goals are ignored
        self.goal_handle = None
        self.canceled_early = set()
        self.cancel_reason = ''
        self.last_feedback = 0.0
        self.nav = {'op': 'nav', 'state': 'idle', 'goal': None, 'distance': None, 'eta': None,
                    'recoveries': 0, 'message': ''}
        self.latch('nav', self.nav)

        self.ros_calls = queue.SimpleQueue()
        self.ros_wake = self.create_guard_condition(self.run_ros_calls)

        self.create_timer(0.1, self.drive_watchdog)
        self.create_timer(0.5, self.send_status)

    # ROS side (runs in the ROS thread) ---------------------------------------------------------

    def in_ros(self, fn) -> None:
        """Runs `fn` in the ROS thread (callable from any thread)."""
        self.ros_calls.put(fn)
        self.ros_wake.trigger()

    def run_ros_calls(self) -> None:
        while True:
            try:
                fn = self.ros_calls.get_nowait()
            except queue.Empty:
                return
            try:
                fn()
            except Exception as e:  # noqa: BLE001 - one bad request must not stop the bridge
                self.get_logger().error(f'request from the web page failed: {e!r}')

    def to_pages(self, kind: str, data, stream: str = '') -> None:
        def deliver():
            if stream:
                self.optional_latest[stream] = data
            for c in self.pages:
                if not stream or stream in c.wants:
                    c.put(kind, data)
        self.loop.call_soon_threadsafe(deliver)

    def latch(self, kind: str, msg) -> None:
        """Sends to every page now, and to pages that connect later."""
        data = msg if isinstance(msg, bytes) else json.dumps(msg)
        self.loop.call_soon_threadsafe(self.latched.__setitem__, kind, data)
        self.to_pages(kind, data)

    def notice(self, text: str, level: str = 'info') -> None:
        self.to_pages('notice', json.dumps({'op': 'notice', 'level': level, 'text': text}))

    def on_tf(self, msg: TFMessage) -> None:
        items = protocol.transforms(msg, static=False)
        self.loop.call_soon_threadsafe(self._deliver_transforms, items)

    def on_tf_static(self, msg: TFMessage) -> None:
        items = protocol.transforms(msg, static=True)

        def remember():
            for t in items:
                self.static_transforms[t['child']] = t
            self._deliver_transforms(items)
        self.loop.call_soon_threadsafe(remember)

    def _deliver_transforms(self, items: list) -> None:
        for c in self.pages:
            c.put_transforms(items)

    def on_scan(self, msg: LaserScan) -> None:
        self.to_pages('scan', json.dumps(protocol.scan(msg)))
        pose = protocol.static_pose(self.static_transforms, msg.header.frame_id, self.base_frame)
        if pose is None:
            self.get_logger().warn(
                f'no fixed transform from {msg.header.frame_id} to {self.base_frame} yet, '
                'so driving by hand does not stop for obstacles', once=True)
            return
        reach = self.robot_radius + self.drive_margin + DRIVE_HORIZON + 0.05
        now = time.monotonic()
        self.obstacles = (now, protocol.nearby_points(msg, pose, reach))
        if self.odom_pose is not None:
            self.motion.append((*self.odom_pose, protocol.nearby_points(msg, pose, 4.0, every=3)))

    def on_odom(self, msg: Odometry) -> None:
        self.to_pages('odom', json.dumps(protocol.odom(msg)))
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (p.x, p.y, yaw)

    def on_world(self, msg: MarkerArray) -> None:
        self.latch('world', protocol.world(msg))

    def on_battery(self, msg: Float32) -> None:
        self.latch('battery', {'op': 'battery', 'charge': round(float(msg.data), 3)})

    def on_charging(self, msg) -> None:
        self.latch('charging', {'op': 'charging', 'state': int(msg.state)})

    def on_map(self, msg: OccupancyGrid) -> None:
        self.latch('map', protocol.grid_frame(msg, 'map'))

    def on_costmap(self, msg: OccupancyGrid) -> None:
        # Kept even while no page wants it, so turning the layer on shows it straight away
        # instead of after Nav2's next update.
        self.to_pages('costmap', protocol.grid_frame(msg, 'costmap'), stream='costmap')

    def on_mapping_state(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(state, dict):
            state['op'] = 'mapping'
            self.latch('mapping', state)

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_model = protocol.camera_model(msg)

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if 'camera' not in self.wanted or now - self.last_camera < self.camera_period:
            return
        self.last_camera = now
        data = protocol.jpeg(msg, self.jpeg_quality)
        if data is None:
            self.get_logger().warn(f'cannot send camera encoding {msg.encoding}', once=True)
            return
        self.to_pages('camera', protocol.camera_frame(msg, data, self.camera_model), stream='camera')

    def drive_watchdog(self) -> None:
        if self.driving and time.monotonic() - self.last_drive > self.drive_timeout:
            self.get_logger().info('drive commands stopped arriving; stopping the robot')
            self.stop()

    def send_status(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self.to_pages('status', json.dumps({'op': 'status', 'time': round(now, 2)}))

    def stop(self) -> None:
        self.driving = False
        self.cmd_pub.publish(Twist())

    # Navigation (ROS thread) -------------------------------------------------------------------

    def set_nav(self, **changes) -> None:
        self.nav.update(changes)
        self.latch('nav', self.nav)

    def nav_active(self) -> bool:
        return self.nav['state'] in ('starting', 'active')

    def send_goal(self, pose: tuple) -> None:
        x, y, yaw = pose
        goal_info = {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3), 'frame': self.map_frame}
        if self.nav_client is None:
            self.set_nav(state='failed', goal=goal_info, message='Navigation (Nav2) is not installed on the robot.')
            return
        if not self.nav_client.server_is_ready():
            self.set_nav(state='failed', goal=goal_info,
                         message='Navigation is not running yet. Give it a few seconds after startup.')
            return
        self.publish_speed_limit()  # again, in case Nav2 restarted since (it forgets it)
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        _, _, qz, qw = protocol.yaw_quaternion(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_handle = None  # Nav2 drops the previous goal when it accepts this one
        self.cancel_reason = ''
        self.set_nav(state='starting', goal=goal_info, distance=None, eta=None, recoveries=0, message='')
        future = self.nav_client.send_goal_async(goal, feedback_callback=lambda fb: self.on_nav_feedback(seq, fb))
        future.add_done_callback(lambda f: self.on_goal_response(seq, f))

    def on_goal_response(self, seq: int, future) -> None:
        handle = future.result()
        if seq != self.goal_seq:
            # A newer goal replaced this one (Nav2 switches to it by itself), or it was canceled
            # before it was accepted.
            if seq in self.canceled_early:
                self.canceled_early.discard(seq)
                if handle.accepted:
                    handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.set_nav(state='failed', message='Navigation is not running yet, so it refused the goal. '
                                                 'Give it a few seconds.')
            return
        self.goal_handle = handle
        self.set_nav(state='active')
        handle.get_result_async().add_done_callback(lambda f: self.on_nav_result(seq, f))

    def on_nav_feedback(self, seq: int, msg) -> None:
        now = time.monotonic()
        if seq != self.goal_seq or now - self.last_feedback < NAV_FEEDBACK_PERIOD:
            return
        self.last_feedback = now
        fb = msg.feedback
        self.set_nav(distance=round(float(fb.distance_remaining), 2),
                     eta=round(protocol.seconds(fb.estimated_time_remaining), 1),
                     recoveries=int(fb.number_of_recoveries))

    def on_nav_result(self, seq: int, future) -> None:
        if seq != self.goal_seq:
            return
        self.goal_handle = None
        response = future.result()
        status = response.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.set_nav(state='succeeded', distance=0.0, eta=0.0, message='Arrived.')
        elif status == GoalStatus.STATUS_CANCELED:
            self.set_nav(state='canceled', eta=None, message=self.cancel_reason or 'Stopped.')
        else:
            result = response.result
            detail = getattr(result, 'error_msg', '') or ''
            code = getattr(result, 'error_code', 0)
            self.set_nav(state='failed', eta=None, message=f'Could not get there: {protocol.nav_error(code, detail)}.')
        self.latch('plan', {'op': 'plan', 'frame': self.map_frame, 'points': []})

    def cancel_goal(self, reason: str) -> None:
        if not self.nav_active():
            return
        self.cancel_reason = reason
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        else:
            # Not accepted yet: stop tracking it, and cancel it as soon as it is accepted.
            self.canceled_early.add(self.goal_seq)
            self.goal_seq += 1
            self.set_nav(state='canceled', eta=None, message=reason)
            self.latch('plan', {'op': 'plan', 'frame': self.map_frame, 'points': []})

    def publish_speed_limit(self) -> None:
        if self.speed_limit_pub is None or self.top_speed is None:
            return
        msg = self.SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = float(self.top_speed)
        self.speed_limit_pub.publish(msg)

    def set_camera_subscribed(self, wanted: bool) -> None:
        if wanted and not self.camera_subs:
            image_topic, info_topic = self.camera_topics
            self.camera_subs = [
                self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data),
                self.create_subscription(CameraInfo, info_topic, self.on_camera_info, qos_profile_sensor_data),
            ]
        elif not wanted and self.camera_subs:
            for sub in self.camera_subs:
                self.destroy_subscription(sub)
            self.camera_subs = []

    def on_plan(self, msg: Path) -> None:
        if self.nav_active():
            self.latch('plan', protocol.path(msg))

    def set_initial_pose(self, pose: tuple) -> None:
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        _, _, qz, qw = protocol.yaw_quaternion(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # About 30 cm and 15 degrees of uncertainty: AMCL spreads its guesses over that much.
        msg.pose.covariance[0] = 0.09
        msg.pose.covariance[7] = 0.09
        msg.pose.covariance[35] = 0.07
        self.initial_pose_pub.publish(msg)
        self.notice('Robot position set. It firms up as the robot moves.')

    def map_command(self, command: str) -> None:
        client = self.map_clients.get(command)
        if client is None:
            return
        if not client.service_is_ready():
            self.notice('Mapping is not running on the robot.', 'error')
            return
        if command != 'save_map':
            self.cancel_goal('Stopped while the map changes.')

        def done(future) -> None:
            response = future.result()
            if response is not None:
                self.notice(response.message, 'info' if response.success else 'error')
        client.call_async(Trigger.Request()).add_done_callback(done)

    # Websocket side (runs on asyncio) ----------------------------------------------------------

    async def serve(self) -> None:
        async with websockets.serve(self.on_page, self.host, self.port, compression=None, max_size=2 ** 20):
            self.get_logger().info(f'web bridge listening on ws://{self.host}:{self.port}')
            await asyncio.Future()

    async def on_page(self, ws, path=None) -> None:
        client = Client(ws)
        self.pages.add(client)
        peer = getattr(ws, 'remote_address', None)
        self.get_logger().info(f'web page connected from {peer}')
        hello = {'op': 'hello', 'sim': bool(self.get_parameter('use_sim_time').value)}
        await ws.send(json.dumps(hello))
        if self.static_transforms:
            client.put_transforms(list(self.static_transforms.values()))
        for kind, data in self.latched.items():
            client.put(kind, data)
        sender = asyncio.ensure_future(self.send_loop(client))
        try:
            async for raw in ws:
                self.on_page_message(client, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            sender.cancel()
            self.pages.discard(client)
            self.update_wanted()
            self.get_logger().info(f'web page disconnected from {peer}')

    async def send_loop(self, client: Client) -> None:
        try:
            while True:
                await client.wake.wait()
                client.wake.clear()
                if client.transforms:
                    batch, client.transforms = client.transforms, []
                    await client.ws.send(json.dumps({'op': 'tf', 'transforms': batch}))
                slots, client.slots = client.slots, {}
                for data in slots.values():
                    await client.ws.send(data)
        except websockets.ConnectionClosed:
            pass

    def on_page_message(self, client: Client, raw) -> None:
        try:
            msg = json.loads(raw)
        except (TypeError, ValueError):
            return
        op = msg.get('op') if isinstance(msg, dict) else None
        if op == 'drive':
            linear, angular = protocol.clamp_drive(msg, self.max_linear, self.max_angular)
            moving = linear != 0.0 or angular != 0.0
            cmd = Twist()
            cmd.linear.x = self.guard(linear, angular)
            cmd.angular.z = angular
            if self.nav_active():
                if not moving:
                    # Letting go of the keys while Nav2 drives: leave the wheels to Nav2.
                    self.driving = False
                    return
                self.in_ros(lambda: self.cancel_goal('Stopped: you took over driving.'))
            self.cmd_pub.publish(cmd)
            self.driving = moving
            self.last_drive = time.monotonic()
        elif op in OPTIONAL_STREAMS:
            if msg.get('enabled'):
                client.wants.add(op)
                if op in self.optional_latest and op != 'camera':
                    client.put(op, self.optional_latest[op])
            else:
                client.wants.discard(op)
            self.update_wanted()
        elif op == 'top_speed':
            speed = protocol.parse_speed(msg, self.max_linear)
            if speed is not None and speed != self.top_speed:
                self.top_speed = speed
                self.in_ros(self.publish_speed_limit)
        elif op == 'goal':
            pose = protocol.parse_pose(msg)
            if pose is not None:
                self.in_ros(lambda: self.send_goal(pose))
        elif op == 'cancel':
            self.in_ros(lambda: self.cancel_goal('Stopped.'))
        elif op == 'initial_pose':
            pose = protocol.parse_pose(msg)
            if pose is not None:
                self.in_ros(lambda: self.set_initial_pose(pose))
        elif op == 'map_command':
            command = msg.get('command')
            if command in protocol.MAP_COMMANDS:
                self.in_ros(lambda: self.map_command(command))

    def guard(self, linear: float, angular: float) -> float:
        """`linear`, slowed or stopped so the robot doesn't push against things: what the lidar
        sees in its way, and anything it turns out to be stuck against."""
        now = time.monotonic()
        when, points = self.obstacles
        if linear == 0.0 or now - when > SCAN_MAX_AGE:
            return linear  # no fresh scan (lidar off?): drive as asked
        way = 'ahead' if linear > 0 else 'behind'
        if self.stuck is not None:
            # Stays stuck that way until it drives the other way or turns a little.
            direction, yaw = self.stuck
            pose = self.odom_pose
            turned = pose is not None and abs(math.remainder(pose[2] - yaw, 2 * math.pi)) > 0.35
            if direction != (linear > 0) or turned:
                self.stuck = None
        if self.stuck is None:
            history = list(self.motion)  # a copy: the ROS thread adds to it
            if protocol.stalled(history, linear):
                self.stuck = (linear > 0, history[-1][2])
                self.get_logger().info(f'the robot is stuck against something {way}')
        if self.stuck is not None:
            allowed = 0.0
            why = f'the robot is pushing against something {way} it that the lidar can\'t see'
        else:
            free = protocol.free_travel(points, linear, angular, self.robot_radius + self.drive_margin,
                                        self.robot_radius - 0.02, DRIVE_HORIZON)
            allowed = protocol.guard_speed(linear, free)
            why = f'something is in the way {way}'
        if allowed == 0.0 and now - self.last_blocked_notice > 3.0:
            self.last_blocked_notice = now
            self.notice(f'Stopped: {why}. Turn or drive the other way.')
        return allowed

    def update_wanted(self) -> None:
        self.wanted = set().union(*(c.wants for c in self.pages)) if self.pages else set()
        camera = 'camera' in self.wanted
        self.in_ros(lambda: self.set_camera_subscribed(camera))


def main() -> None:
    rclpy.init()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node = WebBridge(loop)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        loop.run_until_complete(node.serve())
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001 - best effort on the way out
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
