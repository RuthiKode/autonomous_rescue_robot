"""Record /cmd_vel while someone drives, and replay the recording in sim time.

  ros2 run rescue_eval cmd_vel_record --output drives/rooms.csv     # Ctrl-C to finish
  ros2 run rescue_eval cmd_vel_replay --input drives/rooms.csv

The file holds one row per received command: sim seconds since the first
command, linear x, angular z. Replay publishes each row when the sim clock
reaches the same offset.

REPLAY IS NOT BIT-EXACT. The commands are identical, the motion is not: each
command lands within about a millisecond of its recorded offset rather than on
the same physics step, the wheels slip, and sensor noise differs per seed. Two
replays of one file end in slightly different poses, and the difference grows
with the length of the drive. Use replay for repeatable mapping runs, not as
a substitute for a controller.
"""
import argparse
import csv
import os
import signal
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rosgraph_msgs.msg import Clock

HEADER = ['t_sim_s', 'linear_x', 'angular_z']


def load_drive(path):
    """Return [(t, linear_x, angular_z)] sorted by time."""
    with open(path, newline='') as f:
        rows = [(float(r['t_sim_s']), float(r['linear_x']), float(r['angular_z']))
                for r in csv.DictReader(line for line in f if not line.startswith('#'))]
    return sorted(rows)


def save_drive(path, rows, comment=''):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        if comment:
            f.write(f'# {comment}\n')
        writer = csv.writer(f)
        writer.writerow(HEADER)
        for t, v, w in rows:
            writer.writerow([f'{t:.3f}', f'{v:.4f}', f'{w:.4f}'])


class ClockNode(Node):
    """A node that tracks /clock directly (1 ms resolution)."""

    def __init__(self, name):
        super().__init__(name)
        self.sim_time = None
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9


class Recorder(ClockNode):

    def __init__(self):
        super().__init__('cmd_vel_recorder')
        self.rows = []
        self._t0 = None
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, QoSProfile(depth=100))

    def _on_cmd(self, msg):
        if self.sim_time is None:
            return
        if self._t0 is None:
            self._t0 = self.sim_time
        self.rows.append((self.sim_time - self._t0, msg.linear.x, msg.angular.z))


class Replayer:
    """Feeds a recorded drive to /cmd_vel. Call step() from a spin loop."""

    def __init__(self, node, rows, start_delay=1.0):
        self._node, self._rows, self._i = node, rows, 0
        self._pub = node.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=100))
        self._t0 = None
        self._delay = start_delay
        self.late_max = 0.0

    def duration(self):
        return self._rows[-1][0] if self._rows else 0.0

    def step(self, sim_time):
        """Publish every command that is due. Returns True once the drive is finished."""
        if sim_time is None:
            return False
        if self._t0 is None:
            self._t0 = sim_time + self._delay
        while self._i < len(self._rows) and sim_time >= self._t0 + self._rows[self._i][0]:
            t, v, w = self._rows[self._i]
            self.late_max = max(self.late_max, sim_time - (self._t0 + t))
            self._publish(v, w)
            self._i += 1
        if self._i >= len(self._rows):
            self._publish(0.0, 0.0)
            return True
        return False

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self._pub.publish(msg)


def record_main(argv=None):
    parser = argparse.ArgumentParser(description='Record /cmd_vel with sim-time offsets.')
    parser.add_argument('--output', required=True)
    parser.add_argument('--comment', default='')
    args, ros_args = parser.parse_known_args(argv)
    # Own signal handling: a recorder started in the background of a script has
    # SIGINT ignored, so SIGTERM must also end the recording and save the file.
    stop = []
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    node = Recorder()
    print('Recording /cmd_vel. Drive the robot, then press Ctrl-C.', flush=True)
    try:
        while not stop and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    save_drive(args.output, node.rows, args.comment)
    span = node.rows[-1][0] if node.rows else 0.0
    print(f'Saved {len(node.rows)} commands over {span:.1f} sim seconds to {args.output}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


def replay_main(argv=None):
    parser = argparse.ArgumentParser(description='Replay a recorded /cmd_vel file in sim time.')
    parser.add_argument('--input', required=True)
    args, ros_args = parser.parse_known_args(argv)
    rows = load_drive(args.input)
    rclpy.init(args=ros_args)
    node = ClockNode('cmd_vel_replayer')
    replayer = Replayer(node, rows)
    print(f'Replaying {len(rows)} commands, {replayer.duration():.1f} sim seconds.'
          ' Replay is not bit-exact (timing, slip, noise).', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if replayer.step(node.sim_time):
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    print(f'Done. Largest publish delay: {replayer.late_max * 1000:.1f} ms of sim time.')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(replay_main())
