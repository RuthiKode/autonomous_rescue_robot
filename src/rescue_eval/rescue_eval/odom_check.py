"""Print the latest pose from /odom_raw and /odometry/filtered as x, y, yaw.

  ros2 run rescue_eval odom_check

Compare with Gazebo's true pose: gz model -m rescue_robot -p
"""
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data

TOPICS = ('/odom_raw', '/odometry/filtered')


def main():
    rclpy.init()
    node = rclpy.create_node('odom_check')
    latest = {}
    for topic in TOPICS:
        node.create_subscription(Odometry, topic,
                                 lambda msg, t=topic: latest.__setitem__(t, msg),
                                 qos_profile_sensor_data)
    deadline = node.get_clock().now().nanoseconds + int(10e9)
    while len(latest) < len(TOPICS) and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    for _ in range(10):  # take the newest message of each topic
        rclpy.spin_once(node, timeout_sec=0.05)
    for topic in TOPICS:
        if topic not in latest:
            print(f'{topic:<20} no message received')
            continue
        msg = latest[topic]
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        print(f'{topic:<20} x {p.x:8.4f}  y {p.y:8.4f}  yaw {yaw:8.4f} rad'
              f'  ({msg.header.frame_id} -> {msg.child_frame_id})')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
