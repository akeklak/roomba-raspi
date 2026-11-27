"""Publishes the simulated world's static geometry so the web UI can draw the apartment in 3D.

Reads the Gazebo SDF world file, turns every static box and cylinder into a marker, and
publishes them once as a latched visualization_msgs/MarkerArray on /sim/world (frame "world").
"""

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from roomba_sim.sdf_shapes import parse_world, quaternion


def to_markers(shapes: List[dict], frame_id: str = 'world') -> MarkerArray:
    array = MarkerArray()
    for i, shape in enumerate(shapes):
        m = Marker()
        m.header.frame_id = frame_id
        m.ns = shape['name'] or 'world'
        m.id = i
        m.type = Marker.CUBE if shape['kind'] == 'box' else Marker.CYLINDER
        m.action = Marker.ADD
        x, y, z, roll, pitch, yaw = shape['pose']
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        q = quaternion(roll, pitch, yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x, m.scale.y, m.scale.z = shape['size']
        m.color.r, m.color.g, m.color.b, m.color.a = shape['rgba']
        m.frame_locked = False
        array.markers.append(m)
    return array


class WorldMarkers(Node):

    def __init__(self) -> None:
        super().__init__('world_markers')
        world_file = self.declare_parameter('world_file', '').value
        frame_id = self.declare_parameter('frame_id', 'world').value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/sim/world', qos)
        shapes = parse_world(world_file)
        self.pub.publish(to_markers(shapes, frame_id))
        self.get_logger().info(f'published {len(shapes)} world shapes from {world_file}')


def main() -> None:
    rclpy.init()
    node = WorldMarkers()
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
