#!/usr/bin/env python3
"""Negative control for tf_audit: publish a second copy of a transform on /tf.

  scripts/duplicate_tf_publisher.py odom base_link

Run it next to the real stack; `ros2 run rescue_eval tf_audit` must then fail.
"""
import sys

from geometry_msgs.msg import TransformStamped
import rclpy
from tf2_ros import TransformBroadcaster

parent, child = sys.argv[1], sys.argv[2]
rclpy.init()
node = rclpy.create_node('duplicate_tf_publisher')
broadcaster = TransformBroadcaster(node)


def publish():
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.rotation.w = 1.0
    broadcaster.sendTransform(t)


node.create_timer(0.05, publish)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
