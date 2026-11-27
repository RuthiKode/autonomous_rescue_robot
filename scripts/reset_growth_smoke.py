#!/usr/bin/env python3
"""Does slam_toolbox keep mapping after /slam_toolbox/reset when it was paused (toggle) before?
Stages, as the mission does them: pause (toggle) -> serialize -> reset(pause_new_measurements=False)
-> drive and measure the known map area; if it did not grow: toggle pause once more -> drive -> measure."""
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from slam_toolbox.srv import Pause, Reset, SerializePoseGraph


def main():
    rclpy.init()
    n = rclpy.create_node('reset_growth_smoke', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    last = {}
    n.create_subscription(OccupancyGrid, '/map', lambda m: last.update(map=m),
                          rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    pub = n.create_publisher(Twist, '/cmd_vel', 10)

    def spin(sec, vx=0.0, wz=0.0):
        end = time.time() + sec
        while time.time() < end:
            t = Twist()
            t.linear.x, t.angular.z = vx, wz
            pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.1)

    def known():
        m = last.get('map')
        return float('nan') if m is None else float((np.array(m.data) >= 0).sum()) * m.info.resolution ** 2

    def call(name, typ, req):
        cli = n.create_client(typ, name)
        assert cli.wait_for_service(timeout_sec=20.0), f'{name} not available'
        f = cli.call_async(req)
        end = time.time() + 30
        while not f.done() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        return f.result()

    def drive():
        spin(8.0, vx=0.15)
        spin(8.0, wz=0.6)
        spin(6.0)                      # stopped: let the map update

    spin(3.0)
    print(f'start:                        known {known():.1f} m2')
    call('/slam_toolbox/pause_new_measurements', Pause, Pause.Request())
    r = SerializePoseGraph.Request()
    r.filename = '/tmp/reset_growth_smoke'
    call('/slam_toolbox/serialize_map', SerializePoseGraph, r)
    call('/slam_toolbox/reset', Reset, Reset.Request(pause_new_measurements=False))
    spin(4.0)
    print(f'after pause+serialize+reset:  known {known():.1f} m2')
    drive()
    print(f'after driving 1.2 m + turn:   known {known():.1f} m2')
    res = call('/slam_toolbox/pause_new_measurements', Pause, Pause.Request())
    print(f'pause toggled again: status {res.status}')
    spin(8.0, vx=-0.15)
    spin(8.0, wz=-0.6)
    spin(6.0)
    print(f'after toggle + driving again: known {known():.1f} m2')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
