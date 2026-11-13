"""Run one seeded simulation, record a rosbag, score it, and append one row to the CSV.

  ros2 run rescue_eval run_eval --world rooms --seed 1
  ros2 run rescue_eval run_eval --world rooms --seed 1 --slam --drive drives/rooms.csv \
      --method slam_replay

Without --slam the run records for a fixed duration and has no metrics to fill.
With --slam it starts slam_toolbox, replays the drive, saves the map with
map_saver_cli, and scores the map (IoU, accuracy) and the trajectory (ATE)
against Gazebo ground truth. Metrics of later stages stay "nan".
"""
import argparse
import datetime
import math
import os
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import tf2_ros
import yaml

from rescue_eval import map_score, trajectory
from rescue_eval.cmd_vel_io import ClockNode, load_drive, Replayer
from rescue_eval.driving import pose2d
from rescue_eval.metrics import append_row, placeholder_metrics
from rescue_eval.sim_runner import launch_sim, Process, spin_until


class EvalNode(ClockNode):
    """Tracks sim time and logs map -> base_link against Gazebo's true pose."""

    def __init__(self, spawn, rate_hz, tf_wait_s):
        super().__init__('rescue_eval_runner')
        self.spawn = spawn
        self.log = trajectory.TrajectoryLog()
        self.truth_seen = False
        self.logging = False
        self._period, self._tf_wait = 1.0 / rate_hz, tf_wait_s
        self._next = 0.0
        self._pending = []
        self.dropped = 0
        self.buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._listener = tf2_ros.TransformListener(self.buffer, self, spin_thread=False)
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_truth,
                                 qos_profile_sensor_data)

    def _on_truth(self, msg):
        self.truth_seen = True
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.logging and stamp >= self._next:
            self._next = stamp + self._period
            self._pending.append((stamp, msg.header.stamp, pose2d(msg)))

    def slam_ready(self):
        return self.buffer.can_transform('map', 'base_link', Time())

    def process_pending(self):
        """Look up map -> base_link at the stamp of each queued truth sample."""
        while self._pending:
            stamp, ros_stamp, true_pose = self._pending[0]
            when = Time.from_msg(ros_stamp)
            if self.buffer.can_transform('map', 'base_link', when):
                t = self.buffer.lookup_transform('map', 'base_link', when).transform
                q = t.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                est = trajectory.map_to_world((t.translation.x, t.translation.y, yaw), self.spawn)
                self.log.add(stamp, est, true_pose)
            elif self.sim_time is not None and self.sim_time - stamp > self._tf_wait:
                self.dropped += 1
            else:
                break
            self._pending.pop(0)


def _bag_summary(bag_dir):
    """Return (total messages, number of topics with data) from the bag metadata."""
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if not os.path.isfile(meta_path):
        return 0, 0
    with open(meta_path) as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    with_data = sum(1 for t in info['topics_with_message_count'] if t['message_count'] > 0)
    return int(info['message_count']), with_data


def main(argv=None):
    share = get_package_share_directory('rescue_eval')
    sim_share = get_package_share_directory('rescue_simulation')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', required=True, help='rooms, collapsed or open_plan')
    parser.add_argument('--seed', type=int, required=True, help='Gazebo random seed for this run')
    parser.add_argument('--method', default=None, help='condition label written to the CSV')
    parser.add_argument('--duration', type=float, default=None,
                        help='sim seconds to record (ignored with --drive)')
    parser.add_argument('--slam', action='store_true', help='run slam_toolbox and score the map')
    parser.add_argument('--drive', default=None, help='cmd_vel recording to replay')
    parser.add_argument('--output-dir', default=None, help='where the CSV and bags are written')
    parser.add_argument('--config', default=os.path.join(share, 'config', 'eval.yaml'))
    args = parser.parse_args(argv)

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    method = args.method or cfg['method']
    duration = args.duration if args.duration is not None else float(cfg['duration_s'])
    out_dir = os.path.abspath(args.output_dir or cfg['output_dir'])
    drive_rows = load_drive(args.drive) if args.drive else None

    truth_file = os.path.join(sim_share, 'worlds', args.world + '_survivors.yaml')
    if not os.path.isfile(truth_file):
        sys.exit(f"Unknown world '{args.world}' (no {truth_file})")
    with open(truth_file) as f:
        truth = yaml.safe_load(f)
    spawn = (truth['robot_spawn']['x'], truth['robot_spawn']['y'], truth['robot_spawn']['yaw'])

    now = datetime.datetime.now(datetime.timezone.utc)
    run_id = f"{args.world}_{method}_seed{args.seed}_{now.strftime('%Y%m%dT%H%M%SZ')}"
    bag_dir = os.path.join(out_dir, 'bags', run_id)
    run_dir = os.path.join(out_dir, 'runs', run_id)
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, 'launch.log')

    print(f'[run_eval] {run_id}: launching (log: {log_path})', flush=True)
    sim = launch_sim(args.world, args.seed, log_path,
                     launch_file='mapping.launch.py' if args.slam else 'sim.launch.py')
    rclpy.init()
    node = EvalNode(spawn, float(cfg['slam']['trajectory_rate_hz']), float(cfg['slam']['tf_wait_s']))
    bag = None
    status = 'ok'
    sim_elapsed = wall_elapsed = 0.0
    map_yaml = ''
    try:
        result = spin_until(node, lambda: node.sim_time is not None and node.sim_time > 1.0
                            and node.truth_seen, float(cfg['startup_timeout_s']), sim.alive)
        if result == 'ok' and args.slam:
            result = spin_until(node, node.slam_ready, float(cfg['slam']['ready_timeout_s']),
                                sim.alive)
        if result != 'ok':
            status = 'startup_' + result
        else:
            bag = Process(['ros2', 'bag', 'record', '--use-sim-time', '-o', bag_dir]
                          + cfg['record_topics'], log_path)
            sim_start, wall_start = node.sim_time, time.monotonic()
            node.logging = args.slam
            if drive_rows:
                replayer = Replayer(node, drive_rows)
                settle = float(cfg['slam']['post_drive_settle_s'])
                total = replayer.duration() + settle + 1.0
                print(f'[run_eval] replaying {len(drive_rows)} commands'
                      f' ({replayer.duration():.1f} sim s)', flush=True)
                done_at = [None]

                def finished():
                    node.process_pending()
                    if done_at[0] is None:
                        if replayer.step(node.sim_time):
                            done_at[0] = node.sim_time
                        return False
                    return node.sim_time - done_at[0] >= settle
            else:
                total = duration
                print(f'[run_eval] recording {duration:.1f} sim seconds', flush=True)

                def finished():
                    node.process_pending()
                    return node.sim_time - sim_start >= duration
            result = spin_until(node, finished,
                                total * float(cfg['run_timeout_factor']) + 30.0, sim.alive)
            sim_elapsed = node.sim_time - sim_start
            wall_elapsed = time.monotonic() - wall_start
            if result != 'ok':
                status = 'run_' + result
            if args.slam and sim.alive():
                prefix = os.path.join(run_dir, 'map')
                saver = subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix,
                     '--fmt', 'pgm', '--ros-args', '-p', 'use_sim_time:=true',
                     '-p', 'map_subscribe_transient_local:=true'],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                    timeout=float(cfg['slam']['map_save_timeout_s']), check=False)
                with open(log_path, 'a') as f:
                    f.write(saver.stdout)
                if os.path.isfile(prefix + '.yaml'):
                    map_yaml = prefix + '.yaml'
                elif status == 'ok':
                    status = 'map_not_saved'
    except KeyboardInterrupt:
        status = 'interrupted'
    except subprocess.TimeoutExpired:
        status = 'map_save_timeout'
    finally:
        print('[run_eval] shutting down', flush=True)
        if bag is not None:
            bag.stop(float(cfg['shutdown_timeout_s']))
        sim.stop(float(cfg['shutdown_timeout_s']))
        log = node.log
        dropped = node.dropped
        node.destroy_node()
        rclpy.try_shutdown()

    messages, topics_with_data = _bag_summary(bag_dir)
    if status == 'ok' and messages == 0:
        status = 'empty_bag'
    row = {
        'run_id': run_id,
        'timestamp_utc': now.isoformat(timespec='seconds'),
        'world': args.world,
        'method': method,
        'seed': args.seed,
        'status': status,
        'drive': os.path.basename(args.drive) if args.drive else '',
        'sim_duration_s': f'{sim_elapsed:.3f}',
        'wall_duration_s': f'{wall_elapsed:.3f}',
        'real_time_factor': f'{sim_elapsed / wall_elapsed:.3f}' if wall_elapsed > 0 else 'nan',
        'n_true_survivors': len(truth['survivors']),
        'bag_path': os.path.relpath(bag_dir, out_dir),
        'bag_messages': messages,
        'bag_topics_with_data': topics_with_data,
        'map_path': os.path.relpath(map_yaml, out_dir) if map_yaml else '',
    }
    row.update(placeholder_metrics())

    if args.slam:
        if map_yaml:
            truth_map = map_score.load_map(
                os.path.join(sim_share, 'worlds', args.world + '_truth_map.yaml'))
            s = map_score.score(truth_map, map_score.load_map(map_yaml), spawn)
            row['map_iou'] = f"{s['iou']:.4f}"
            row['map_accuracy'] = f"{s['accuracy']:.4f}"
            print(f"[run_eval] map: IoU {s['iou']:.4f}, accuracy {s['accuracy']:.4f}"
                  f" over {s['known_cells']} cells known in both")
        log.save_tum(os.path.join(run_dir, 'trajectory_slam.tum'),
                     os.path.join(run_dir, 'trajectory_truth.tum'))
        try:
            a = trajectory.ate(log)
            row['ate_rmse_m'] = f"{a['rmse']:.4f}"
            row['ate_aligned_rmse_m'] = f"{a['rmse_aligned']:.4f}"
            print(f"[run_eval] trajectory: ATE RMSE {a['rmse']:.4f} m"
                  f" ({a['rmse_aligned']:.4f} m after rigid alignment) over {a['n']} poses,"
                  f" {dropped} samples without a transform")
        except ImportError:
            print('[run_eval] evo is not installed, trajectory error left as nan.'
                  ' Install it with: pip install --user --break-system-packages --no-deps evo')

    csv_path = os.path.join(out_dir, cfg['csv_name'])
    append_row(csv_path, row)
    print(f'[run_eval] status={status}; row appended to {csv_path}')
    return 0 if status == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
