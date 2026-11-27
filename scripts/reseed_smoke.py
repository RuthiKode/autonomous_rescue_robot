#!/usr/bin/env python3
"""Smoke test: can slam_toolbox (async mapping) be re-seeded at a given pose while it runs?

For each shift s in SHIFTS: read map->base_link, serialize the pose graph, deserialize it with
START_AT_GIVEN_POSE at the current pose + (s, 0), wait, and print where map->base_link ends up,
the robot's own motion by odometry, and whether /map keeps updating.
"""
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.time import Time
from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph

SHIFTS = [0.0, 1.0]


def main():
    rclpy.init()
    n = rclpy.create_node('reseed_smoke', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    maps = []
    n.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(time.time()),
                          rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)

    def pose(parent):
        t = buf.lookup_transform(parent, 'base_link', Time(), Duration(seconds=1.0)).transform
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def call(cli, req):
        assert cli.wait_for_service(timeout_sec=20.0), f'{cli.srv_name} not available'
        f = cli.call_async(req)
        end = time.time() + 30
        while not f.done() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        return f.result()

    ser = n.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    des = n.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
    spin(3.0)
    for s in SHIFTS:
        b, ob = pose('map'), pose('odom')
        r = SerializePoseGraph.Request()
        r.filename = '/tmp/reseed_smoke'
        res = call(ser, r)
        d = DeserializePoseGraph.Request()
        d.filename = '/tmp/reseed_smoke'
        d.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        d.initial_pose = Pose2D(x=b[0] + s, y=b[1], theta=b[2])
        call(des, d)
        k = len(maps)
        spin(15.0)
        a, oa = pose('map'), pose('odom')
        moved = math.hypot(oa[0] - ob[0], oa[1] - ob[1])
        print(f'shift {s:+.2f}: serialize {res.result}; map pose ({b[0]:.2f},{b[1]:.2f}) -> ({a[0]:.2f},{a[1]:.2f}); '
              f'given ({b[0] + s:.2f},{b[1]:.2f}); error to given {math.hypot(a[0] - b[0] - s, a[1] - b[1]):.2f} m; '
              f'odom motion {moved:.2f} m; odom pose ({oa[0]:.2f},{oa[1]:.2f}); /map msgs {len(maps) - k}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
