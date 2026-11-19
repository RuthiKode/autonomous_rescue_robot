"""Measure a stamped topic's rate in sim time and wall time, and count dropped frames.

  ros2 run rescue_eval camera_rate_check --duration 60

A dropped frame is a gap between consecutive header stamps of about two or more
nominal periods. /camera/camera_info is measured alongside the image: it carries
the same stamps in a tiny message, so if it has no gaps the sensor rendered every
frame and any image gaps were lost in transport.

--mode raw         count serialized messages without decoding them (cheap subscriber)
--mode deserialize let rclpy build a Python sensor_msgs/Image per frame (what a
                   normal Python node pays)
"""
import argparse
import struct
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


def analyse(stamps, walls):
    """Return a dict of rate statistics from header stamps and arrival wall times."""
    n = len(stamps)
    if n < 3:
        return {'received': n}
    deltas = sorted(b - a for a, b in zip(stamps, stamps[1:]) if b - a > 1e-6)
    duplicates = n - 1 - len(deltas)
    if not deltas:
        return {'received': n}
    period = deltas[len(deltas) // 2]
    dropped = sum(max(0, round((b - a) / period) - 1) for a, b in zip(stamps, stamps[1:]))
    sim_span, wall_span = stamps[-1] - stamps[0], walls[-1] - walls[0]
    return {
        'received': n,
        'duplicates': duplicates,
        'dropped': dropped,
        'drop_pct': 100.0 * dropped / (n + dropped),
        'nominal_hz': 1.0 / period,
        'sim_hz': (n - 1) / sim_span,
        'wall_hz': (n - 1) / wall_span,
        'rtf': sim_span / wall_span,
        'max_gap_frames': max(round((b - a) / period) for a, b in zip(stamps, stamps[1:])),
    }


class Counter:

    def __init__(self, node, topic, msg_type, raw, qos):
        self.topic, self.raw = topic, raw
        self.stamps, self.walls = [], []
        node.create_subscription(msg_type, topic, self._cb, qos, raw=raw)

    def _cb(self, msg):
        if self.raw:
            # CDR: 4-byte encapsulation header, then std_msgs/Header.stamp (int32, uint32).
            little = msg[1] == 1
            sec, nsec = struct.unpack_from('<iI' if little else '>iI', msg, 4)
        else:
            sec, nsec = msg.header.stamp.sec, msg.header.stamp.nanosec
        self.stamps.append(sec + nsec * 1e-9)
        self.walls.append(time.monotonic())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--type', choices=['Image', 'PointCloud2'], default='Image',
                        help='message type of --topic (only the header stamp is read)')
    parser.add_argument('--info-topic', default='/camera/camera_info')
    parser.add_argument('--duration', type=float, default=60.0, help='sim seconds')
    parser.add_argument('--mode', choices=['raw', 'deserialize'], default='raw')
    parser.add_argument('--reliability', choices=['best_effort', 'reliable'],
                        default='best_effort')
    parser.add_argument('--depth', type=int, default=5, help='subscriber queue depth')
    parser.add_argument('--timeout', type=float, default=600.0, help='wall-clock limit, seconds')
    args, ros_args = parser.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = rclpy.create_node('camera_rate_check')
    reliability = (ReliabilityPolicy.RELIABLE if args.reliability == 'reliable'
                   else ReliabilityPolicy.BEST_EFFORT)
    qos = QoSProfile(depth=args.depth, reliability=reliability)
    counters = [Counter(node, args.topic, Image if args.type == 'Image' else PointCloud2,
                      args.mode == 'raw', qos),
                Counter(node, args.info_topic, CameraInfo, True, qos)]
    image = counters[0]
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(image.stamps) > 1 and image.stamps[-1] - image.stamps[0] >= args.duration:
            break

    print(f'subscriber: mode={args.mode} reliability={args.reliability} depth={args.depth}')
    for c in counters:
        s = analyse(c.stamps, c.walls)
        if 'dropped' not in s:
            print(f'{c.topic}: only {s["received"]} messages received')
            continue
        print(f'{c.topic}: received {s["received"]} ({s["duplicates"]} duplicate stamps),'
              f' dropped {s["dropped"]}'
              f' ({s["drop_pct"]:.1f}%), longest gap {s["max_gap_frames"]} frames,'
              f' nominal {s["nominal_hz"]:.2f} Hz, sim-time rate {s["sim_hz"]:.2f} Hz,'
              f' wall rate {s["wall_hz"]:.2f} Hz, real-time factor {s["rtf"]:.3f}')
    for info in node.get_publishers_info_by_topic(args.topic):
        q = info.qos_profile
        print(f'publisher {info.node_name}: reliability={q.reliability.name}'
              f' durability={q.durability.name} depth={q.depth}')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
