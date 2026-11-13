"""Run one seeded simulation, record a rosbag and append one row to the results CSV.

  ros2 run rescue_eval run_eval --world rooms --seed 1

Phase 1 skeleton: the run facts (durations, real-time factor, bag contents) are
real; the metrics of docs/PLAN.md are written as "nan" until their stage exists.
"""
import argparse
import datetime
import os
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
import yaml

from rescue_eval.metrics import append_row, placeholder_metrics


class ClockWatcher(Node):

    def __init__(self):
        super().__init__('rescue_eval_clock_watcher')
        self.sim_time = None
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)

    def _on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9


def _stop(proc, timeout):
    """SIGINT the whole process group, then SIGKILL whatever is left."""
    if proc is None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, signal.SIGINT)
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        pass
    except ProcessLookupError:
        return
    try:
        os.killpg(pgid, signal.SIGKILL)  # also reaps Gazebo children that ignored SIGINT
    except ProcessLookupError:
        pass
    proc.wait()


def _wait_for(node, condition, wall_timeout, alive):
    deadline = time.monotonic() + wall_timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if condition():
            return 'ok'
        if not alive():
            return 'sim_died'
    return 'timeout'


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
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', required=True, help='rooms, collapsed or open_plan')
    parser.add_argument('--seed', type=int, required=True, help='Gazebo random seed for this run')
    parser.add_argument('--method', default=None, help='condition label written to the CSV')
    parser.add_argument('--duration', type=float, default=None, help='sim seconds to record')
    parser.add_argument('--output-dir', default=None, help='where the CSV and bags are written')
    parser.add_argument('--config', default=os.path.join(share, 'config', 'eval.yaml'))
    args = parser.parse_args(argv)

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    method = args.method or cfg['method']
    duration = args.duration if args.duration is not None else float(cfg['duration_s'])
    out_dir = os.path.abspath(args.output_dir or cfg['output_dir'])

    truth_file = os.path.join(get_package_share_directory('rescue_simulation'),
                              'worlds', args.world + '_survivors.yaml')
    if not os.path.isfile(truth_file):
        sys.exit(f"Unknown world '{args.world}' (no {truth_file})")
    with open(truth_file) as f:
        n_true = len(yaml.safe_load(f)['survivors'])

    now = datetime.datetime.now(datetime.timezone.utc)
    run_id = f"{args.world}_{method}_seed{args.seed}_{now.strftime('%Y%m%dT%H%M%SZ')}"
    bag_dir = os.path.join(out_dir, 'bags', run_id)
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
    log_path = os.path.join(out_dir, 'bags', run_id + '.launch.log')

    print(f'[run_eval] {run_id}: launching the simulation (log: {log_path})')
    log = open(log_path, 'w')
    launch = subprocess.Popen(
        ['ros2', 'launch', 'rescue_bringup', 'sim.launch.py',
         f'world:={args.world}', 'headless:=true', f'seed:={args.seed}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    rclpy.init()
    node = ClockWatcher()
    bag = None
    status = 'ok'
    sim_elapsed = wall_elapsed = 0.0
    try:
        def alive():
            return launch.poll() is None

        # Sim time must be running before anything is recorded.
        result = _wait_for(node, lambda: node.sim_time is not None and node.sim_time > 1.0,
                           float(cfg['startup_timeout_s']), alive)
        if result != 'ok':
            status = 'startup_' + result
        else:
            print(f'[run_eval] sim time is running; recording {duration:.1f} sim seconds')
            bag = subprocess.Popen(
                ['ros2', 'bag', 'record', '--use-sim-time', '-o', bag_dir] + cfg['record_topics'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            sim_start, wall_start = node.sim_time, time.monotonic()
            result = _wait_for(node, lambda: node.sim_time - sim_start >= duration,
                               duration * float(cfg['run_timeout_factor']) + 30.0, alive)
            sim_elapsed = node.sim_time - sim_start
            wall_elapsed = time.monotonic() - wall_start
            if result != 'ok':
                status = 'run_' + result
    except KeyboardInterrupt:
        status = 'interrupted'
    finally:
        print('[run_eval] shutting down')
        _stop(bag, float(cfg['shutdown_timeout_s']))
        _stop(launch, float(cfg['shutdown_timeout_s']))
        log.close()
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
        'sim_duration_s': f'{sim_elapsed:.3f}',
        'wall_duration_s': f'{wall_elapsed:.3f}',
        'real_time_factor': f'{sim_elapsed / wall_elapsed:.3f}' if wall_elapsed > 0 else 'nan',
        'n_true_survivors': n_true,
        'bag_path': os.path.relpath(bag_dir, out_dir),
        'bag_messages': messages,
        'bag_topics_with_data': topics_with_data,
    }
    row.update(placeholder_metrics())
    csv_path = os.path.join(out_dir, cfg['csv_name'])
    append_row(csv_path, row)
    print(f'[run_eval] status={status}; row appended to {csv_path}')
    return 0 if status == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
