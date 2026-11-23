"""Seed slam_toolbox's map, then exit.

slam_toolbox publishes only a tiny seed map until the robot has translated: at rest, a frontier
explorer finds nothing to explore. This node waits for sim time and the map -> base_link transform,
drives straight ahead for a few seconds, stops, and exits (so a launch file can start Nav2 and the
rest when it has finished).
"""
import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


def main():
    rclpy.init()
    node = Node('wakeup')
    node.declare_parameter('speed_mps', 0.3)
    node.declare_parameter('duration_s', 3.0)
    node.declare_parameter('settle_s', 3.0)
    node.declare_parameter('timeout_s', 120.0)       # wall seconds to wait for the simulator and SLAM
    speed = float(node.get_parameter('speed_mps').value)
    duration = float(node.get_parameter('duration_s').value)
    settle = float(node.get_parameter('settle_s').value)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    clock = node.get_clock()

    wall_deadline = node.get_parameter('timeout_s').value
    import time
    start_wall = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if clock.now().nanoseconds > 1e9 and buffer.can_transform('map', 'base_link', Time()):
            break
        if time.monotonic() - start_wall > wall_deadline:
            node.get_logger().error('wakeup: simulator or SLAM did not come up')
            return 1
    end = clock.now() + Duration(seconds=duration)
    msg = Twist()
    msg.linear.x = speed
    while rclpy.ok() and clock.now() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
    pub.publish(Twist())
    end = clock.now() + Duration(seconds=settle)
    while rclpy.ok() and clock.now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info('wakeup done: the map has been seeded')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
