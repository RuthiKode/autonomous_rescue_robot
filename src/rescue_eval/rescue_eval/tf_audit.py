"""Check that every transform has exactly one publisher.

  ros2 run rescue_eval tf_audit --duration 5

Two sources of evidence are combined:

1. The tf2 buffer's frame YAML (all_frames_as_yaml). It gives each frame's
   parent and "broadcaster". In ROS 2 the broadcaster field is always
   "default_authority", because /tf messages do not carry the sender's name,
   so on its own it cannot tell two publishers apart. It is used here for the
   tree shape and printed for reference.

2. The per-publisher sequence number that the middleware attaches to every
   message. One publisher numbers its messages 1, 2, 3, ... so the numbers seen
   for one transform must strictly increase. If two nodes publish the same
   transform, their two counters interleave and the sequence stops increasing.

A violation is any of: sequence numbers that do not strictly increase for a
transform; a child frame with two different parents; a transform that appears
on both /tf and /tf_static.
"""
import argparse
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
import tf2_ros
import yaml


def find_violations(observations):
    """Return a list of problems found in [(topic, parent, child, sequence_number)].

    Pure function, so it can be unit-tested without ROS.
    """
    problems = []
    last_seq, parents, topics = {}, {}, {}
    reported = set()
    for topic, parent, child, seq in observations:
        key = (topic, parent, child)
        parents.setdefault(child, set()).add(parent)
        topics.setdefault((parent, child), set()).add(topic)
        if seq is not None:
            if key in last_seq and seq <= last_seq[key] and key not in reported:
                reported.add(key)
                problems.append(
                    f'{parent} -> {child} on {topic}: sequence number {seq} after'
                    f' {last_seq[key]}; more than one publisher sends this transform')
            last_seq[key] = seq
    for child, ps in sorted(parents.items()):
        if len(ps) > 1:
            problems.append(f'{child} has more than one parent: {sorted(ps)}')
    for (parent, child), ts in sorted(topics.items()):
        if len(ts) > 1:
            problems.append(f'{parent} -> {child} is published on both /tf and /tf_static')
    return problems


class TfAudit:
    """Attach to a node and record every transform message with its sequence number."""

    def __init__(self, node):
        self.node = node
        self.observations = []
        self.missing_sequence_numbers = False
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(TFMessage, '/tf',
                                 lambda m, i: self._on_tf(m, i, '/tf'), QoSProfile(depth=1000))
        node.create_subscription(TFMessage, '/tf_static',
                                 lambda m, i: self._on_tf(m, i, '/tf_static'), static_qos)

    def _on_tf(self, msg, info, topic):
        seq = info.get('publication_sequence_number')
        # Middleware without sequence numbers reports None or the max uint64.
        if seq is None or seq >= 2 ** 63:
            seq = None
            self.missing_sequence_numbers = True
        for t in msg.transforms:
            self.observations.append((topic, t.header.frame_id, t.child_frame_id, seq))
            if topic == '/tf_static':
                self.buffer.set_transform_static(t, 'default_authority')
            else:
                self.buffer.set_transform(t, 'default_authority')

    def frames(self):
        """The tf2 buffer's frame YAML as a dict: child -> {parent, broadcaster, rate, ...}."""
        return yaml.safe_load(self.buffer.all_frames_as_yaml()) or {}

    def transforms(self):
        """Sorted unique (parent, child, topic)."""
        return sorted({(p, c, t) for t, p, c, _ in self.observations})

    def publishers(self):
        return {topic: sorted(i.node_name for i in self.node.get_publishers_info_by_topic(topic))
                for topic in ('/tf', '/tf_static')}

    def violations(self):
        problems = find_violations(self.observations)
        if self.missing_sequence_numbers:
            problems.append('the middleware did not provide publication sequence numbers,'
                            ' so duplicate publishers cannot be ruled out')
        return problems

    def report(self):
        lines = ['transforms seen:']
        counts = {}
        for topic, parent, child, _ in self.observations:
            counts[(parent, child, topic)] = counts.get((parent, child, topic), 0) + 1
        for (parent, child, topic), n in sorted(counts.items()):
            lines.append(f'  {parent} -> {child}  [{topic}, {n} messages]')
        lines.append(f'publisher nodes: {self.publishers()}')
        broadcasters = sorted({str(v.get('broadcaster')) for v in self.frames().values()})
        lines.append(f'tf2 frame YAML broadcaster values: {broadcasters}')
        return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--duration', type=float, default=5.0, help='wall seconds to listen')
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = rclpy.create_node('tf_audit')
    audit = TfAudit(node)
    end = time.monotonic() + args.duration
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(audit.report())
    problems = audit.violations()
    print('RESULT: ' + ('every transform has one publisher' if not problems
                        else f'{len(problems)} problem(s)'))
    for p in problems:
        print('  - ' + p)
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
