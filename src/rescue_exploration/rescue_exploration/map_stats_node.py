"""Publishes how much the robot has mapped, for the dashboard's plot (Foxglove cannot compute it).

/explore/explored_area_m2  std_msgs/Float32  known free area on /map, in square metres, once per second

This is a map statistic, not the evaluation's coverage (which needs the ground-truth grid and is
only computed offline by rescue_eval).
"""
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


def free_area_m2(data, resolution, free_max=25):
    """Area of the cells with an occupancy value in [0, free_max], in square metres."""
    arr = np.asarray(data, dtype=np.int16)
    return float(np.count_nonzero((arr >= 0) & (arr <= free_max))) * resolution * resolution


class MapStats(Node):

    def __init__(self):
        super().__init__('map_stats')
        self.area = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched)
        self.pub = self.create_publisher(Float32, 'explore/explored_area_m2', 10)
        self.create_timer(1.0, self._tick)

    def _on_map(self, msg):
        self.area = free_area_m2(msg.data, msg.info.resolution)

    def _tick(self):
        if self.area is not None:
            self.pub.publish(Float32(data=float(self.area)))


def main():
    rclpy.init()
    node = MapStats()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:                      # a second Ctrl-C from the launch shutdown can land here
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
