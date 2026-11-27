"""Perfect localization for the simulation: publishes world -> odom from Gazebo's true pose.

Wheel odometry (odom -> base_footprint) drifts whenever the wheels slip, for example when the
robot pushes against furniture. On the real robot, SLAM corrects that by publishing map -> odom.
Until mapping is set up, this node does the same job in the sim using the true pose, so the
lidar and the robot always line up with the simulated apartment in the web UI.

Subscribes to /odom (wheel odometry) and /ground_truth/odom (true pose in "world") and publishes
the world -> odom transform on /tf.
"""

from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from roomba_sim.pose2d import quaternion_yaw, world_to_odom, yaw_from_quaternion

# Pair a true pose with the wheel odometry from the same moment, within this tolerance.
MAX_STAMP_GAP_NS = 30_000_000


def _stamp_ns(msg: Odometry) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _pose2d(msg: Odometry):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))


class GroundTruthLocalizer(Node):

    def __init__(self) -> None:
        super().__init__('ground_truth_localizer')
        self.world_frame = self.declare_parameter('world_frame', 'world').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.wheel = deque(maxlen=50)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self.on_wheel, 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 20)

    def on_wheel(self, msg: Odometry) -> None:
        self.wheel.append((_stamp_ns(msg), _pose2d(msg)))

    def on_truth(self, msg: Odometry) -> None:
        if not self.wheel:
            return
        t = _stamp_ns(msg)
        stamp, odom_pose = min(self.wheel, key=lambda w: abs(w[0] - t))
        if abs(stamp - t) > MAX_STAMP_GAP_NS:
            return
        x, y, yaw = world_to_odom(_pose2d(msg), odom_pose)
        out = TransformStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = x
        out.transform.translation.y = y
        qx, qy, qz, qw = quaternion_yaw(yaw)
        out.transform.rotation.x = qx
        out.transform.rotation.y = qy
        out.transform.rotation.z = qz
        out.transform.rotation.w = qw
        self.tf.sendTransform(out)


def main() -> None:
    rclpy.init()
    node = GroundTruthLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
