"""Scripted driving for tests: open-loop commands and a ground-truth waypoint follower.

Nothing here is part of the robot's autonomy. The follower steers on Gazebo's
true pose so that a test drive is repeatable; the estimators are what get judged.
Sim time is taken from the stamps of /ground_truth/odom (50 Hz).
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from rescue_eval.sim_runner import spin_until


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def pose2d(msg):
    """(x, y, yaw) of a nav_msgs/Odometry pose."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (p.x, p.y, yaw)


def relative(start, end):
    """Motion from start to end, expressed in the start frame: (dx, dy, dyaw)."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(end[2] - start[2]))


class DriveNode(Node):
    """Publishes /cmd_vel and keeps the latest truth, raw and filtered odometry."""

    def __init__(self, name='rescue_eval_driver'):
        super().__init__(name)
        self.truth = self.raw = self.ekf = None
        self.sim_time = None
        self.twist_log = None  # set to [] to collect (raw - truth) twist samples
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth, qos)
        self.create_subscription(Odometry, '/odom_raw', self._on_raw, qos)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_ekf, qos)

    def _on_truth(self, msg):
        self.truth = msg
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_raw(self, msg):
        self.raw = msg
        if self.twist_log is not None and self.truth is not None:
            t = self.truth.twist.twist
            self.twist_log.append((msg.twist.twist.linear.x - t.linear.x,
                                   msg.twist.twist.angular.z - t.angular.z,
                                   abs(t.linear.x), abs(t.angular.z)))

    def _on_ekf(self, msg):
        self.ekf = msg

    def ready(self):
        return None not in (self.truth, self.raw, self.ekf)

    def poses(self):
        return {'truth': pose2d(self.truth), 'raw': pose2d(self.raw), 'ekf': pose2d(self.ekf)}

    def command(self, v, w):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def hold(self, v, w, sim_seconds, rate_hz=20.0, alive=lambda: True, wall_timeout=None):
        """Publish (v, w) at rate_hz for sim_seconds of sim time."""
        end = self.sim_time + sim_seconds
        next_pub = [self.sim_time]

        def step():
            if self.sim_time >= next_pub[0]:
                self.command(v, w)
                next_pub[0] = self.sim_time + 1.0 / rate_hz
            return self.sim_time >= end

        return spin_until(self, step, wall_timeout or sim_seconds * 20 + 30, alive)

    def stop_and_settle(self, sim_seconds, alive=lambda: True):
        return self.hold(0.0, 0.0, sim_seconds, alive=alive)

    def follow(self, waypoints, cfg, alive=lambda: True):
        """Drive through (x, y) waypoints in the world frame using the true pose."""
        for wx, wy in waypoints:
            def step():
                x, y, yaw = pose2d(self.truth)
                dist = math.hypot(wx - x, wy - y)
                if dist < cfg['tolerance_m']:
                    return True
                err = wrap(math.atan2(wy - y, wx - x) - yaw)
                w = max(-cfg['max_angular'], min(cfg['max_angular'], cfg['k_angular'] * err))
                if abs(err) > cfg['turn_in_place_rad']:
                    v = 0.0
                else:
                    v = min(cfg['max_linear'], cfg['k_linear'] * dist + cfg['min_linear'])
                    v *= max(0.0, math.cos(err))
                self.command(v, w)
                return False

            last = [None]

            def throttled():
                # one control step per truth message
                if self.sim_time == last[0]:
                    return False
                last[0] = self.sim_time
                return step()

            result = spin_until(self, throttled, cfg['waypoint_timeout_wall_s'], alive)
            if result != 'ok':
                self.command(0.0, 0.0)
                return result
        self.command(0.0, 0.0)
        return 'ok'
