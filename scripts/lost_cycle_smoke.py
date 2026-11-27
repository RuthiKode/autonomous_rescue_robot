#!/usr/bin/env python3
"""Smoke test of the lost-mode cycle on a running slam_toolbox (async mapping), robot held still:
serialize the graph, reset SLAM (a fresh map), then deserialize the saved graph with
START_AT_GIVEN_POSE at the pose from before. Prints known map area and map->base_link at each stage."""
import math
import time

from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from slam_toolbox.srv import DeserializePoseGraph, Reset, SerializePoseGraph
import tf2_ros


def main():
    rclpy.init()
    n = rclpy.create_node('lost_cycle_smoke', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    last = {}
    n.create_subscription(OccupancyGrid, '/map', lambda m: last.update(map=m),
                          rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)

    def pose():
        t = buf.lookup_transform('map', 'base_link', Time(), Duration(seconds=1.0)).transform
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def known():
        m = last.get('map')
        if m is None:
            return float('nan')
        return float((np.array(m.data) >= 0).sum()) * m.info.resolution ** 2

    def call(name, typ, req):
        cli = n.create_client(typ, name)
        assert cli.wait_for_service(timeout_sec=20.0), f'{name} not available'
        f = cli.call_async(req)
        end = time.time() + 30
        while not f.done() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        return f.result()

    spin(3.0)
    p0, k0 = pose(), known()
    print(f'before:        pose ({p0[0]:.2f}, {p0[1]:.2f}, {p0[2]:.2f}), known map {k0:.1f} m2')
    r = SerializePoseGraph.Request()
    r.filename = '/tmp/lost_cycle_smoke'
    print('serialize:', call('/slam_toolbox/serialize_map', SerializePoseGraph, r).result)
    print('reset:', call('/slam_toolbox/reset', Reset, Reset.Request()).result)
    spin(10.0)
    p1, k1 = pose(), known()
    print(f'after reset:   pose ({p1[0]:.2f}, {p1[1]:.2f}, {p1[2]:.2f}), known map {k1:.1f} m2')
    d = DeserializePoseGraph.Request()
    d.filename = '/tmp/lost_cycle_smoke'
    d.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
    d.initial_pose = Pose2D(x=p0[0], y=p0[1], theta=p0[2])
    call('/slam_toolbox/deserialize_map', DeserializePoseGraph, d)
    spin(10.0)
    p2, k2 = pose(), known()
    print(f'after reload:  pose ({p2[0]:.2f}, {p2[1]:.2f}, {p2[2]:.2f}), known map {k2:.1f} m2; '
          f'pose error to before {math.hypot(p2[0] - p0[0], p2[1] - p0[1]):.2f} m')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
