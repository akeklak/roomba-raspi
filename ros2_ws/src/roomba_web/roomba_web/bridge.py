"""Websocket bridge between ROS and the Roomba web UI.

    ros2 run roomba_web web_bridge          (both launch files start it)

Serves ws://<robot>:8765. It subscribes to the topics the UI draws, sends them to every
connected page as JSON (camera pictures as JPEG), and publishes /cmd_vel from the page's drive
buttons. The message format is described in roomba_web/protocol.py.

Safety: the page repeats its drive command ten times a second while a key or button is held.
If those messages stop for `drive_timeout` seconds (tab closed, wifi dropped), the robot stops.

ROS runs in a background thread; the websocket server runs on asyncio in the main thread. Each
page gets the newest value of each kind of message, so a slow connection skips old scans and
pictures instead of falling behind.
"""

import asyncio
import json
import threading
import time

import rclpy
import websockets
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from roomba_web import protocol

MAX_QUEUED_TRANSFORMS = 500

LATCHED = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class Client:
    """One connected page."""

    def __init__(self, ws) -> None:
        self.ws = ws
        self.wants_camera = False
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
        camera_topic = p('camera_topic', '/camera/image_raw').value
        camera_info_topic = p('camera_info_topic', '/camera/camera_info').value

        self.pages = set()
        # Sent to each page when it connects, since these rarely or never repeat.
        self.static_transforms = {}
        self.latched = {}
        self.camera_model = None
        self.last_camera = 0.0
        self.any_camera = False
        self.driving = False
        self.last_drive = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static, LATCHED)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(MarkerArray, '/sim/world', self.on_world,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Float32, '/battery/charge_ratio', self.on_battery, 10)
        self.create_subscription(Image, camera_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera_info_topic, self.on_camera_info, qos_profile_sensor_data)
        try:  # Only the real robot has this message type (from create_robot).
            from create_msgs.msg import ChargingState
            self.create_subscription(ChargingState, '/battery/charging_state', self.on_charging, 10)
        except ImportError:
            pass
        self.create_timer(0.1, self.drive_watchdog)
        self.create_timer(0.5, self.send_status)

    # ROS side (runs in the ROS thread) ---------------------------------------------------------

    def to_pages(self, kind: str, data, camera: bool = False) -> None:
        def deliver():
            for c in self.pages:
                if not camera or c.wants_camera:
                    c.put(kind, data)
        self.loop.call_soon_threadsafe(deliver)

    def latch(self, kind: str, msg: dict) -> None:
        text = json.dumps(msg)
        self.loop.call_soon_threadsafe(self.latched.__setitem__, kind, text)
        self.to_pages(kind, text)

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

    def on_odom(self, msg: Odometry) -> None:
        self.to_pages('odom', json.dumps(protocol.odom(msg)))

    def on_world(self, msg: MarkerArray) -> None:
        self.latch('world', protocol.world(msg))

    def on_battery(self, msg: Float32) -> None:
        self.latch('battery', {'op': 'battery', 'charge': round(float(msg.data), 3)})

    def on_charging(self, msg) -> None:
        self.latch('charging', {'op': 'charging', 'state': int(msg.state)})

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_model = protocol.camera_model(msg)

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if not self.any_camera or now - self.last_camera < self.camera_period:
            return
        self.last_camera = now
        data = protocol.jpeg(msg, self.jpeg_quality)
        if data is None:
            self.get_logger().warn(f'cannot send camera encoding {msg.encoding}', once=True)
            return
        self.to_pages('camera', protocol.camera_frame(msg, data, self.camera_model), camera=True)

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

    # Websocket side (runs on asyncio) ----------------------------------------------------------

    async def serve(self) -> None:
        async with websockets.serve(self.on_page, self.host, self.port, compression=None):
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
        for kind, text in self.latched.items():
            client.put(kind, text)
        sender = asyncio.ensure_future(self.send_loop(client))
        try:
            async for raw in ws:
                self.on_page_message(client, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            sender.cancel()
            self.pages.discard(client)
            self.update_camera_demand()
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
            cmd = Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular
            self.cmd_pub.publish(cmd)
            self.driving = linear != 0.0 or angular != 0.0
            self.last_drive = time.monotonic()
        elif op == 'camera':
            client.wants_camera = bool(msg.get('enabled'))
            self.update_camera_demand()

    def update_camera_demand(self) -> None:
        self.any_camera = any(c.wants_camera for c in self.pages)


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
