"""Republish a world's ground-truth survivor poses on /ground_truth/survivors.

Simulation only. The evaluation harness is the only intended consumer.
"""
import math

from builtin_interfaces.msg import Time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_msgs.msg import Survivor, SurvivorArray
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


class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.declare_parameter('truth_file', '')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('topic', '/ground_truth/survivors')

        truth_file = self.get_parameter('truth_file').value
        if not truth_file:
            raise RuntimeError('ground_truth_publisher: parameter truth_file is required')
        with open(truth_file) as f:
            self._msg = survivors_from_truth(yaml.safe_load(f))

        # Transient local so a late subscriber (the scorer) still gets the data.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(SurvivorArray, self.get_parameter('topic').value, qos)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._publish)
        self.get_logger().info(
            f'Publishing {len(self._msg.survivors)} ground-truth survivors from {truth_file}')

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


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
