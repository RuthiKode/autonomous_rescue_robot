"""Send random reachable goals through Nav2's NavigateToPose and log what happens.

  ros2 run rescue_eval nav_goals --world rooms --seed 1 --output results/nav/goals.csv
  ros2 run rescue_eval nav_goals --world rooms --seed 1 --warmup-drive drives/rooms.csv

Needs the stack of rescue_bringup navigation_sim.launch.py running. The success
rate is the share of goals for which Nav2 reported success and Gazebo's true pose
ended within success_distance_m of the goal; the true pose is used only for
scoring, never given to Nav2.

For every goal one CSV row records: the goal, status, error code, sim time, true
path length, true final error, the recoveries Nav2 reported, and the recoveries
seen on /behavior_tree_log. A goal that ended without success and without any
recovery firing is flagged stuck_without_recovery (a gate item of docs/PLAN.md).
"""
import argparse
import csv
import math
import os
import re
import sys
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from scipy import ndimage
import yaml

from rescue_eval.cmd_vel_io import load_drive, Replayer
from rescue_eval.driving import pose2d
from rescue_eval.sim_runner import spin_until

GOAL_COLUMNS = [
    'run_id', 'world', 'seed', 'goal_index', 'goal_x', 'goal_y', 'goal_yaw', 'start_x', 'start_y',
    'status', 'nav_error_code', 'success', 'sim_time_s', 'wall_time_s', 'path_length_m',
    'straight_distance_m',
    'final_error_m', 'recoveries_reported', 'recovery_events', 'recovery_event_count',
    'stuck_without_recovery',
]
STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_ABORTED: 'aborted',
                GoalStatus.STATUS_CANCELED: 'canceled'}


def sample_goals(planner, free_in_live_map, count, rng, start_xy, min_distance, min_separation):
    """Pick up to `count` goals; consecutive goals (and the start) are >= min_distance apart.

    planner         an auto_drive.Planner (ground-truth grid, planning inflation)
    free_in_live_map(x, y) -> bool
    """
    rows, cols = np.nonzero(planner.reachable())
    goals, previous = [], start_xy
    for i in rng.permutation(len(rows)):
        x, y = planner.point(int(rows[i]), int(cols[i]))
        if math.hypot(x - previous[0], y - previous[1]) < min_distance:
            continue
        if any(math.hypot(x - g[0], y - g[1]) < min_separation for g in goals):
            continue
        if not free_in_live_map(x, y):
            continue
        goals.append((x, y, float(rng.uniform(-math.pi, math.pi))))
        previous = (x, y)
        if len(goals) == count:
            break
    return goals


def known_free_mask(grid, radius_m):
    """Cells whose whole disk of radius_m is known free (value 0).

    Indexing follows OccupancyGrid: row 0 is the lowest y. The grid border counts as not free.
    """
    res = grid.info.resolution
    h, w = grid.info.height, grid.info.width
    free = (np.array(grid.data, dtype=np.int16).reshape(h, w) == 0)
    k = max(1, int(round(radius_m / res)))
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    return ndimage.binary_erosion(free, structure=(xx ** 2 + yy ** 2 <= k * k))


class NavGoalRunner(Node):

    def __init__(self, cfg, name='rescue_eval_nav_goals'):
        super().__init__(name)
        self.cfg = cfg
        self.truth = self.grid = None
        self.sim_time = None
        self._path_length = 0.0
        self._last_xy = None
        self._tracking = False
        self._feedback_recoveries = 0
        self._events = []
        self._recovery_re = re.compile(cfg['recovery_node_pattern'])
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth,
                                 qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, '/map', lambda m: setattr(self, 'grid', m),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._on_bt_log,
                                 QoSProfile(depth=100))

    # ------------------------------------------------------------ callbacks
    def _on_truth(self, msg):
        self.truth = msg
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._tracking:
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            if self._last_xy is not None:
                self._path_length += math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            self._last_xy = (x, y)

    def _on_bt_log(self, msg):
        if not self._tracking:
            return
        for ev in msg.event_log:
            if ev.current_status == 'RUNNING' and ev.previous_status != 'RUNNING' \
                    and self._recovery_re.match(ev.node_name):
                self._events.append(ev.node_name)
                self.get_logger().info(f'recovery fired: {ev.node_name}')

    def _on_feedback(self, msg):
        self._feedback_recoveries = max(self._feedback_recoveries,
                                        msg.feedback.number_of_recoveries)

    # ------------------------------------------------------------ helpers
    def wait_ready(self, alive=lambda: True):
        ok = spin_until(self, lambda: self.client.server_is_ready(), self.cfg['server_wait_s'],
                        alive)
        if ok != 'ok':
            return f'no navigate_to_pose server ({ok})'
        ok = spin_until(self, lambda: self.grid is not None and self.truth is not None,
                        self.cfg['map_wait_s'], alive)
        return None if ok == 'ok' else f'no /map or /ground_truth/odom ({ok})'

    def true_xy(self):
        p = self.truth.pose.pose.position
        return p.x, p.y

    def live_free_test(self):
        """Return f(x, y) -> bool using a snapshot of the live map."""
        grid = self.grid
        mask = known_free_mask(grid, self.cfg['map_clearance_m'])
        res, ox, oy = grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y

        def test(x, y):
            c, r = int((x - ox) / res), int((y - oy) / res)
            return 0 <= r < mask.shape[0] and 0 <= c < mask.shape[1] and bool(mask[r, c])
        return test

    def stand_still(self, seconds, alive=lambda: True):
        end = self.sim_time + seconds
        return spin_until(self, lambda: self.sim_time >= end, seconds * 20 + 10, alive)

    # ------------------------------------------------------------ one goal
    def run_goal(self, goal, alive=lambda: True):
        """Send one goal. Returns a dict with the per-goal columns."""
        gx, gy, gyaw = goal
        start = self.true_xy()
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = gx, gy
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = (
            math.sin(gyaw / 2), math.cos(gyaw / 2))

        self._path_length, self._last_xy = 0.0, start
        self._feedback_recoveries, self._events = 0, []
        self._tracking = True
        row = {'goal_x': round(gx, 3), 'goal_y': round(gy, 3), 'goal_yaw': round(gyaw, 3),
               'start_x': round(start[0], 3), 'start_y': round(start[1], 3),
               'straight_distance_m': round(math.hypot(gx - start[0], gy - start[1]), 3)}
        t0 = self.sim_time
        wall0 = time.monotonic()
        status, code = 'rejected', ''
        try:
            send = self.client.send_goal_async(msg, feedback_callback=self._on_feedback)
            if spin_until(self, send.done, 20.0, alive) == 'ok' and send.result().accepted:
                handle = send.result()
                result = handle.get_result_async()
                limit = float(self.cfg['goal_timeout_sim_s'])
                done = spin_until(self, lambda: result.done() or self.sim_time - t0 > limit,
                                  limit * 20 + 60, alive)
                if not result.done():
                    status = 'timeout' if done == 'ok' else done
                    cancel = handle.cancel_goal_async()
                    spin_until(self, lambda: cancel.done() and result.done(), 20.0, alive)
                else:
                    r = result.result()
                    status = STATUS_NAMES.get(r.status, str(r.status))
                    code = r.result.error_code
        finally:
            self._tracking = False
        row['sim_time_s'] = round(self.sim_time - t0, 2)
        row['wall_time_s'] = round(time.monotonic() - wall0, 2)
        row['path_length_m'] = round(self._path_length, 3)
        fx, fy = self.true_xy()
        row['final_error_m'] = round(math.hypot(gx - fx, gy - fy), 3)
        row['status'] = status
        row['nav_error_code'] = code
        row['success'] = int(status == 'succeeded'
                             and row['final_error_m'] <= self.cfg['success_distance_m'])
        row['recoveries_reported'] = self._feedback_recoveries
        row['recovery_events'] = ';'.join(self._events)
        row['recovery_event_count'] = len(self._events)
        row['stuck_without_recovery'] = int(not row['success'] and not self._events
                                            and not self._feedback_recoveries)
        return row

    # ------------------------------------------------------------ whole run
    def run(self, planner, seed, count=None, alive=lambda: True, log=print):
        """Sample goals from the live map and run them all. Returns the list of rows."""
        rng = np.random.default_rng(seed)
        goals = sample_goals(planner, self.live_free_test(),
                             count or int(self.cfg['count']), rng, self.true_xy(),
                             float(self.cfg['min_goal_distance_m']),
                             float(self.cfg['min_goal_separation_m']))
        log(f'[nav_goals] {len(goals)} goals sampled (seed {seed})')
        rows = []
        for i, goal in enumerate(goals, 1):
            row = self.run_goal(goal, alive)
            row['goal_index'] = i
            rows.append(row)
            log(f"[nav_goals] goal {i:2d}/{len(goals)} -> ({row['goal_x']:.2f}, {row['goal_y']:.2f})"
                f" {row['status']:<9} success={row['success']} t={row['sim_time_s']:.1f}s"
                f" (rtf {row['sim_time_s'] / max(row['wall_time_s'], 1e-3):.2f})"
                f" path={row['path_length_m']:.2f}m err={row['final_error_m']:.2f}m"
                f" recoveries={row['recoveries_reported']}/{row['recovery_event_count']}"
                + (' STUCK-NO-RECOVERY' if row['stuck_without_recovery'] else ''))
            self.stand_still(float(self.cfg['settle_s']), alive)
        return rows


def make_planner(world, goal_clearance_m):
    """Ground-truth grid planner at the goal clearance (reuses the tour planner)."""
    from rescue_eval.auto_drive import Planner
    share = get_package_share_directory('rescue_eval')
    with open(os.path.join(share, 'config', 'auto_drive.yaml')) as f:
        acfg = yaml.safe_load(f)
    acfg['inflation_by_world'] = {}
    acfg['inflation_m'] = goal_clearance_m
    return Planner(world, acfg)


def write_goal_csv(path, rows, run_id, world, seed):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=GOAL_COLUMNS, restval='')
        w.writeheader()
        for r in rows:
            w.writerow({**r, 'run_id': run_id, 'world': world, 'seed': seed})


def summarise(rows):
    n = len(rows)
    ok = sum(r['success'] for r in rows)
    return {'goals': n, 'succeeded': ok, 'success_rate': ok / n if n else float('nan'),
            'recoveries': sum(r['recoveries_reported'] for r in rows),
            'recovery_events': sum(r['recovery_event_count'] for r in rows),
            'stuck_without_recovery': sum(r['stuck_without_recovery'] for r in rows)}


def main(argv=None):
    share = get_package_share_directory('rescue_eval')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', required=True)
    parser.add_argument('--seed', type=int, default=1, help='seed of the goal sampler')
    parser.add_argument('--count', type=int, default=None)
    parser.add_argument('--output', default='results/nav_goals.csv')
    parser.add_argument('--warmup-drive', default=None,
                        help='cmd_vel recording to replay first, so there is a map to sample from')
    parser.add_argument('--config', default=os.path.join(share, 'config', 'nav_goals.yaml'))
    args, ros_args = parser.parse_known_args(argv)
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    rclpy.init(args=ros_args)
    node = NavGoalRunner(cfg)
    try:
        error = node.wait_ready()
        if error:
            print(error)
            return 1
        if args.warmup_drive:
            rows = load_drive(args.warmup_drive)
            replayer = Replayer(node, rows)
            print(f'[nav_goals] warm-up drive: {len(rows)} commands, {replayer.duration():.0f} sim s')
            done_at = [None]

            def finished():
                if done_at[0] is None:
                    if replayer.step(node.sim_time):
                        done_at[0] = node.sim_time
                    return False
                return node.sim_time - done_at[0] >= 6.0
            spin_until(node, finished, replayer.duration() * 20 + 120)
        planner = make_planner(args.world, float(cfg['goal_clearance_m']))
        rows = node.run(planner, args.seed, args.count)
        write_goal_csv(args.output, rows, 'manual', args.world, args.seed)
        s = summarise(rows)
        print(f"[nav_goals] {s['succeeded']}/{s['goals']} goals succeeded"
              f" ({100 * s['success_rate']:.0f}%), recoveries reported {s['recoveries']},"
              f" recovery events {s['recovery_events']},"
              f" stuck without recovery {s['stuck_without_recovery']}; {args.output}")
        return 0
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
