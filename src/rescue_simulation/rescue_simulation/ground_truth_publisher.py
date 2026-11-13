"""Republish a world's ground-truth survivor poses on /ground_truth/survivors.

Simulation only. The evaluation harness is the only intended consumer; the
matching markers exist so a person can see the truth in RViz2.
"""
import math

from builtin_interfaces.msg import Time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_msgs.msg import Survivor, SurvivorArray
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def survivors_from_truth(truth):
    """Convert the ground-truth dict written by generate_worlds into a SurvivorArray."""
    msg = SurvivorArray()
    msg.header.frame_id = truth.get('frame_id', 'map')
    for entry in truth['survivors']:
        s = Survivor()
        s.id = int(entry['id'])
        s.pose.position.x = float(entry['position']['x'])
        s.pose.position.y = float(entry['position']['y'])
        s.pose.position.z = float(entry['position']['z'])
        yaw = float(entry.get('yaw', 0.0))
        s.pose.orientation.z = math.sin(yaw / 2.0)
        s.pose.orientation.w = math.cos(yaw / 2.0)
        s.confidence = 1.0
        s.hit_count = 0
        s.first_seen = Time()
        s.state = Survivor.STATE_VERIFIED
        msg.survivors.append(s)
    return msg


def markers_from_truth(truth):
    """One green cylinder and one text label per true survivor."""
    array = MarkerArray()
    frame = truth.get('frame_id', 'map')
    for entry in truth['survivors']:
        x, y = float(entry['position']['x']), float(entry['position']['y'])
        post = Marker()
        post.header.frame_id = frame
        post.ns, post.id = 'ground_truth_survivors', int(entry['id'])
        post.type, post.action = Marker.CYLINDER, Marker.ADD
        post.pose.position.x, post.pose.position.y, post.pose.position.z = x, y, 1.0
        post.pose.orientation.w = 1.0
        post.scale.x = post.scale.y = 0.25
        post.scale.z = 2.0
        post.color.g, post.color.a = 1.0, 0.5
        label = Marker()
        label.header.frame_id = frame
        label.ns, label.id = 'ground_truth_labels', int(entry['id'])
        label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 2.3
        label.pose.orientation.w = 1.0
        label.scale.z = 0.35
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f"GT {entry['id']}: {entry['posture']}" + (
            ' (occluded)' if entry.get('occluded') else '')
        array.markers.extend([post, label])
    return array


class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.declare_parameter('truth_file', '')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('topic', '/ground_truth/survivors')
        self.declare_parameter('marker_topic', '/ground_truth/survivor_markers')

        truth_file = self.get_parameter('truth_file').value
        if not truth_file:
            raise RuntimeError('ground_truth_publisher: parameter truth_file is required')
        with open(truth_file) as f:
            truth = yaml.safe_load(f)
        self._msg = survivors_from_truth(truth)
        self._markers = markers_from_truth(truth)

        # Transient local so a late subscriber (the scorer, RViz2) still gets the data.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(SurvivorArray, self.get_parameter('topic').value, qos)
        self._marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, qos)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._publish)
        self.get_logger().info(
            f'Publishing {len(self._msg.survivors)} ground-truth survivors from {truth_file}')

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        self._msg.header.stamp = stamp
        self._pub.publish(self._msg)
        # Marker stamps stay at zero so RViz2 places them with the latest transform.
        self._marker_pub.publish(self._markers)


def main():
    rclpy.init()
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
