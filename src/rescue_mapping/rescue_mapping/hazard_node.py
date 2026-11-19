"""ROS node for the hazard layer: /map in, /hazard_map out."""
import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from rescue_mapping.hazard_layer import hazard_grid


class HazardNode(Node):

    def __init__(self):
        super().__init__('hazard_layer')
        self.declare_parameter('config_file', '')
        with open(self.get_parameter('config_file').value) as f:
            self.cfg = yaml.safe_load(f)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(OccupancyGrid, self.cfg['topic_out'], qos)
        self.create_subscription(OccupancyGrid, self.cfg['topic_in'], self._on_map, qos)
        self._latest = None
        self._published_stamp = None
        self.create_timer(float(self.cfg['publish_period_s']), self._tick)

    def _on_map(self, msg):
        self._latest = msg

    def _tick(self):
        msg = self._latest
        if msg is None or msg is self._published_stamp:
            return
        self._published_stamp = msg
        h, w = msg.info.height, msg.info.width
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        out = hazard_grid(
            grid, msg.info.resolution,
            robot_width_m=float(self.cfg['robot_width_m']), margin_m=float(self.cfg['margin_m']),
            min_area_m2=float(self.cfg['min_area_m2']),
            occupied_threshold=int(self.cfg['occupied_threshold']))
        res = OccupancyGrid()
        res.header = msg.header
        res.info = msg.info
        res.data = out.flatten().tolist()
        self._pub.publish(res)
        self.get_logger().info(
            f'hazard map: {int((out == 100).sum())} hazard cells of {int((out >= 0).sum())} known-free',
            throttle_duration_sec=30.0)


def main():
    rclpy.init()
    node = HazardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
