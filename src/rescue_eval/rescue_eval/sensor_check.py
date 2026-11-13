"""Measure sensor rates in sim time and look for sensor noise on a stationary robot.

  ros2 run rescue_eval sensor_check --duration 10

For every sensor topic it reports the frame_id, the rate computed from message
header stamps (sim time, so it is independent of the real-time factor; dropped
messages show up as a lower `received` count), and a noise figure:
the spread between consecutive readings, which is zero for a noise-free sensor
on a robot that is not moving.
"""
import argparse

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image, Imu, LaserScan


def _stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Track:

    def __init__(self, name):
        self.name = name
        self.frame_id = ''
        self.stamps = []
        self.prev = None
        self.diffs = []

    def add(self, msg, values):
        self.frame_id = msg.header.frame_id
        self.stamps.append(_stamp(msg))
        if self.prev is not None and self.prev.shape == values.shape:
            with np.errstate(invalid='ignore'):  # inf - inf for out-of-range readings
                d = values - self.prev
            d = d[np.isfinite(d)]
            if d.size:
                self.diffs.append(float(np.std(d)))
        self.prev = values

    def rate(self):
        if len(self.stamps) < 2:
            return float('nan')
        deltas = np.diff(np.array(self.stamps))
        # The median period is robust to dropped messages.
        return 1.0 / float(np.median(deltas))

    def noise(self):
        return float(np.mean(self.diffs)) if self.diffs else float('nan')


class SensorCheck(Node):

    def __init__(self):
        super().__init__('sensor_check')
        self.tracks = {
            '/scan': Track('/scan'),
            '/imu': Track('/imu'),
            '/camera/image_raw': Track('/camera/image_raw'),
            '/camera/depth': Track('/camera/depth'),
        }
        qos = qos_profile_sensor_data
        self.create_subscription(LaserScan, '/scan', self._scan, qos)
        self.create_subscription(Imu, '/imu', self._imu, qos)
        # Reliable for images: best-effort loses large frames (docs/PHASE2.md, step 0a).
        image_qos = QoSProfile(depth=30)
        self.create_subscription(Image, '/camera/image_raw', self._image, image_qos)
        self.create_subscription(Image, '/camera/depth', self._depth, image_qos)

    def _scan(self, msg):
        self.tracks['/scan'].add(msg, np.array(msg.ranges, dtype=np.float64))

    def _imu(self, msg):
        v = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                      msg.linear_acceleration.x, msg.linear_acceleration.y])
        self.tracks['/imu'].add(msg, v)

    def _image(self, msg):
        v = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float64)
        self.tracks['/camera/image_raw'].add(msg, v)

    def _depth(self, msg):
        v = np.frombuffer(msg.data, dtype=np.float32).astype(np.float64)
        self.tracks['/camera/depth'].add(msg, v)

    def elapsed(self):
        spans = [t.stamps[-1] - t.stamps[0] for t in self.tracks.values() if len(t.stamps) > 1]
        return min(spans) if len(spans) == len(self.tracks) else 0.0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--duration', type=float, default=10.0, help='sim seconds to observe')
    parser.add_argument('--timeout', type=float, default=120.0, help='wall-clock limit, seconds')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = SensorCheck()
    start = node.get_clock().now()
    try:
        while rclpy.ok() and node.elapsed() < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
            if (node.get_clock().now() - start).nanoseconds * 1e-9 > args.timeout:
                print('TIMEOUT: not every sensor topic delivered data')
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    units = {'/scan': 'm', '/imu': 'rad/s, m/s^2', '/camera/image_raw': 'grey levels',
             '/camera/depth': 'm'}
    print(f"{'topic':<20}{'frame_id':<22}{'received':>9}{'sim span s':>12}"
          f"{'rate Hz':>9}  frame-to-frame noise (std)")
    for name, t in node.tracks.items():
        span = t.stamps[-1] - t.stamps[0] if len(t.stamps) > 1 else 0.0
        print(f'{name:<20}{t.frame_id:<22}{len(t.stamps):>9}{span:>12.2f}'
              f'{t.rate():>9.2f}  {t.noise():.5f} {units[name]}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
