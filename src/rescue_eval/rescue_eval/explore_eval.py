"""One autonomous exploration run: simulation, SLAM, Nav2, explore_lite and perception.

  ros2 run rescue_eval explore_eval --world open_plan --seed 1

Starts the stack, lets explore_lite drive until the time limit or until coverage stops
growing, then scores the run against Gazebo ground truth and appends one row to the CSV:

  coverage, time_to_90_coverage_s      explored reachable free cells (coverage.py)
  det_precision, det_recall,
  duplicate_rate, pin_error_m          confirmed survivor pins (survivor_score.py)
  map_iou, map_iou_tol, map_accuracy   the final SLAM map
  ate_rmse_m, ate_aligned_rmse_m       SLAM pose against truth over the whole run

Per run it also writes coverage.csv (coverage over sim time) and survivors.csv (the final
pins) next to the map. Nav2 starts together with SLAM here: there is no replayed drive
that its /cmd_vel traffic could disturb.
"""
import argparse
import csv
import datetime
import os
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rescue_msgs.msg import SurvivorArray
import yaml

from rescue_eval import map_score, trajectory
from rescue_eval.coverage import coverage, grid_to_map, reachable_truth_mask, time_to
from rescue_eval.metrics import append_row, placeholder_metrics
from rescue_eval.run_eval import _bag_summary, _wait_nav_active, EvalNode
from rescue_eval.sim_runner import launch_sim, Process, spin_until
from rescue_eval.survivor_score import score_pins

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ExploreEvalNode(EvalNode):
    """EvalNode that also samples coverage from /map and keeps the latest /survivors."""

    def __init__(self, spawn, rate_hz, tf_wait_s, truth_map, mask):
        super().__init__(spawn, rate_hz, tf_wait_s)
        self._truth_map, self._mask = truth_map, mask
        self.curve = []                       # [(sim time, coverage)]
        self.survivors = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, LATCHED)
        self.create_subscription(SurvivorArray, '/survivors', self._on_survivors, LATCHED)

    def _on_map(self, msg):
        if self.sim_time is None:
            return
        self.curve.append((self.sim_time, coverage(self._truth_map, self._mask, grid_to_map(msg),
                                                   self.spawn)))

    def _on_survivors(self, msg):
        self.survivors = msg


def plateaued(curve, now, window_s, min_gain):
    """True if coverage grew by less than min_gain over the last window_s of sim time."""
    past = [c for t, c in curve if t <= now - window_s]
    return bool(past) and curve[-1][1] - past[-1] < min_gain


def main(argv=None):
    share = get_package_share_directory('rescue_eval')
    sim_share = get_package_share_directory('rescue_simulation')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--world', required=True)
    parser.add_argument('--seed', type=int, required=True, help='Gazebo random seed')
    parser.add_argument('--method', default=None,
                        help='label in the CSV (default: explore_<detector>)')
    parser.add_argument('--time-limit', type=float, default=None, help='sim seconds')
    parser.add_argument('--detector', choices=['yolo', 'oracle', 'none'], default='yolo',
                        help='yolo: pretrained YOLO; oracle: ground-truth TEST DOUBLE (never a'
                             ' detection result); none: explore only')
    parser.add_argument('--model', default='',
                        help='weights file for --detector yolo (default: perception.yaml)')
    parser.add_argument('--min-hit-rate', default='',
                        help='localizer min_hit_rate override (default: perception.yaml)')
    parser.add_argument('--min-confidence', default='',
                        help='localizer min_confidence override (default: perception.yaml)')
    parser.add_argument('--explore-params', default=None,
                        help='explore_lite parameter file to use instead of the project default')
    parser.add_argument('--worlds-dir', default=None,
                        help='generated worlds to use instead of the installed ones')
    parser.add_argument('--collect-frames', default=None, metavar='DIR',
                        help='dataset collection: add the segmentation camera and save labelled'
                             ' frames to DIR while exploring (not for evaluation runs)')
    parser.add_argument('--output-dir', default=None)
    parser.add_argument('--config', default=os.path.join(share, 'config', 'eval.yaml'))
    args = parser.parse_args(argv)
    worlds = os.path.abspath(args.worlds_dir) if args.worlds_dir else os.path.join(sim_share, 'worlds')

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    ecfg = cfg['explore']
    limit = args.time_limit if args.time_limit is not None else float(ecfg['time_limit_s'])
    out_dir = os.path.abspath(args.output_dir or cfg['output_dir'])

    with open(os.path.join(worlds, args.world + '_survivors.yaml')) as f:
        truth = yaml.safe_load(f)
    spawn = (truth['robot_spawn']['x'], truth['robot_spawn']['y'], truth['robot_spawn']['yaw'])
    truth_map = map_score.load_map(os.path.join(worlds, args.world + '_truth_map.yaml'))
    mask = reachable_truth_mask(truth_map, spawn, float(ecfg['robot_radius_m']))
    true_pos = [(s['position']['x'], s['position']['y']) for s in truth['survivors']]

    args.method = args.method or f'explore_{args.detector}'
    now = datetime.datetime.now(datetime.timezone.utc)
    run_id = f"{args.world}_{args.method}_seed{args.seed}_{now.strftime('%Y%m%dT%H%M%SZ')}"
    bag_dir = os.path.join(out_dir, 'bags', run_id)
    run_dir = os.path.join(out_dir, 'runs', run_id)
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
    os.makedirs(run_dir, exist_ok=True)
    log_path = os.path.join(run_dir, 'launch.log')
    print(f'[explore_eval] {run_id}: launching (log: {log_path})', flush=True)

    sim_extra = []
    if args.worlds_dir:
        sim_extra.append(f'worlds_dir:={worlds}')
    if args.collect_frames:
        sim_extra.append('seg_camera:=true')
    sim = launch_sim(args.world, args.seed, log_path, launch_file='mapping.launch.py',
                     extra_args=sim_extra)
    rclpy.init()
    node = ExploreEvalNode(spawn, float(cfg['slam']['trajectory_rate_hz']),
                           float(cfg['slam']['tf_wait_s']), truth_map, mask)
    procs = []
    bag = None
    status, map_yaml = 'ok', ''
    t0 = sim_elapsed = wall_elapsed = 0.0
    stop_reason = ''
    try:
        result = spin_until(node, lambda: node.sim_time is not None and node.sim_time > 1.0
                            and node.truth_seen, float(cfg['startup_timeout_s']), sim.alive)
        if result == 'ok':
            result = spin_until(node, node.slam_ready, float(cfg['slam']['ready_timeout_s']),
                                sim.alive)
        if result != 'ok':
            status = 'startup_' + result
        else:
            bag = Process(['ros2', 'bag', 'record', '--use-sim-time', '-o', bag_dir]
                          + cfg['record_topics'], log_path)
            # slam_toolbox only grows its map once the robot has translated: at rest the
            # map is a seed of about 1,600 cells, and explore_lite would see nothing to
            # explore. Drive forward a little, before Nav2 starts, to seed a real map.
            pub = node.create_publisher(Twist, '/cmd_vel', 10)
            wake_end = node.sim_time + float(ecfg['wakeup_s'])

            def wake_up():
                msg = Twist()
                msg.linear.x = float(ecfg['wakeup_speed_mps'])
                pub.publish(msg)
                return node.sim_time >= wake_end
            spin_until(node, wake_up, float(ecfg['wakeup_s']) * 30 + 30, sim.alive)
            pub.publish(Twist())
            spin_until(node, lambda: False, 3.0, sim.alive)
            nav = None
            for attempt in range(1, int(cfg['nav']['start_attempts']) + 1):
                nav = Process(['ros2', 'launch', 'rescue_navigation', 'navigation.launch.py'],
                              log_path)
                print(f'[explore_eval] starting Nav2 (attempt {attempt})', flush=True)
                if _wait_nav_active(float(cfg['nav']['startup_timeout_s']), sim.alive):
                    break
                nav.stop(float(cfg['shutdown_timeout_s']))
                nav = None
            if nav is None:
                status = 'nav_not_active'
            else:
                procs.append(nav)
                if args.detector != 'none':
                    procs.append(Process(['ros2', 'launch', 'rescue_perception',
                                          'perception.launch.py', f'detector:={args.detector}',
                                          f'world:={args.world}']
                                         + ([f'model:={os.path.abspath(args.model)}']
                                            if args.model else [])
                                         + ([f'min_hit_rate:={args.min_hit_rate}']
                                            if args.min_hit_rate else [])
                                         + ([f'min_confidence:={args.min_confidence}']
                                            if args.min_confidence else []), log_path))
                    time.sleep(float(ecfg['perception_warmup_s'])
                               if args.detector == 'yolo' else 5.0)
                if args.collect_frames:
                    procs.append(Process(
                        ['ros2', 'run', 'rescue_perception', 'frame_logger', '--ros-args',
                         '-p', f'out_dir:={os.path.abspath(args.collect_frames)}',
                         '-p', f'tag:={run_id}', '-p', 'use_sim_time:=true'], log_path))
                explore_cmd = ['ros2', 'launch', 'rescue_exploration', 'explore_baseline.launch.py']
                if args.explore_params:
                    explore_cmd.append(f'params_file:={os.path.abspath(args.explore_params)}')
                procs.append(Process(explore_cmd, log_path))
                node.logging = True
                t0, wall0 = node.sim_time, time.monotonic()
                print(f'[explore_eval] exploring (limit {limit:.0f} sim s)', flush=True)
                next_report = [t0 + 60.0]

                def finished():
                    nonlocal stop_reason
                    node.process_pending()
                    t = node.sim_time - t0
                    if node.sim_time >= next_report[0] and node.curve:
                        next_report[0] += 60.0
                        pins = len(node.survivors.survivors) if node.survivors else 0
                        print(f'[explore_eval] t={t:5.0f}s coverage {node.curve[-1][1]:.3f}'
                              f' pins {pins}', flush=True)
                    if t >= limit:
                        stop_reason = 'time limit'
                    elif t >= float(ecfg['min_time_s']) and plateaued(
                            node.curve, node.sim_time, float(ecfg['plateau_window_s']),
                            float(ecfg['plateau_gain'])):
                        stop_reason = 'coverage plateau'
                    return bool(stop_reason)
                result = spin_until(node, finished, limit * float(cfg['run_timeout_factor']) + 60,
                                    sim.alive)
                sim_elapsed, wall_elapsed = node.sim_time - t0, time.monotonic() - wall0
                if result != 'ok':
                    status = 'run_' + result
                # let the last detections reach the localizer, then take the map
                spin_until(node, lambda: False, 3.0, sim.alive)
                if sim.alive():
                    prefix = os.path.join(run_dir, 'map')
                    saver = subprocess.run(
                        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix,
                         '--fmt', 'pgm', '--ros-args', '-p', 'use_sim_time:=true',
                         '-p', 'map_subscribe_transient_local:=true',
                         '-p', f"save_map_timeout:={float(cfg['slam']['map_wait_s'])}"],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                        timeout=float(cfg['slam']['map_save_timeout_s']), check=False)
                    with open(log_path, 'a') as f:
                        f.write(saver.stdout)
                    if os.path.isfile(prefix + '.yaml'):
                        map_yaml = prefix + '.yaml'
    except KeyboardInterrupt:
        status = 'interrupted'
    except subprocess.TimeoutExpired:
        status = 'map_save_timeout'
    finally:
        print('[explore_eval] shutting down', flush=True)
        for p in reversed(procs):
            p.stop(float(cfg['shutdown_timeout_s']))
        if bag is not None:
            bag.stop(float(cfg['shutdown_timeout_s']))
        sim.stop(float(cfg['shutdown_timeout_s']))
        log, curve, survivors = node.log, list(node.curve), node.survivors
        node.destroy_node()
        rclpy.try_shutdown()

    messages, topics_with_data = _bag_summary(bag_dir)
    pins = [(s.pose.position.x, s.pose.position.y) for s in survivors.survivors
            if s.state == 'confirmed'] if survivors else []
    candidates = [(s.pose.position.x, s.pose.position.y) for s in survivors.survivors
                  if s.state != 'confirmed'] if survivors else []
    row = {
        'run_id': run_id, 'timestamp_utc': now.isoformat(timespec='seconds'),
        'world': args.world, 'method': args.method, 'seed': args.seed, 'status': status,
        'drive': '', 'sim_duration_s': f'{sim_elapsed:.3f}', 'wall_duration_s': f'{wall_elapsed:.3f}',
        'real_time_factor': f'{sim_elapsed / wall_elapsed:.3f}' if wall_elapsed > 0 else 'nan',
        'n_true_survivors': len(true_pos), 'bag_path': os.path.relpath(bag_dir, out_dir),
        'bag_messages': messages, 'bag_topics_with_data': topics_with_data,
        'map_path': os.path.relpath(map_yaml, out_dir) if map_yaml else '',
    }
    row.update(placeholder_metrics())

    rel = [(t - curve[0][0], c) for t, c in curve] if curve else []
    if rel:
        with open(os.path.join(run_dir, 'coverage.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sim_time_s', 'coverage'])
            w.writerows((f'{t:.1f}', f'{c:.4f}') for t, c in rel)
        row['coverage'] = f'{rel[-1][1]:.4f}'
        row['time_to_90_coverage_s'] = f"{time_to(rel, 0.9):.1f}"
        print(f"[explore_eval] coverage {rel[-1][1]:.3f} after {rel[-1][0]:.0f} s;"
              f" 90% at {row['time_to_90_coverage_s']} s; stopped: {stop_reason or status}")
    with open(os.path.join(run_dir, 'survivors.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['state', 'x', 'y'])
        w.writerows([('confirmed', f'{x:.3f}', f'{y:.3f}') for x, y in pins]
                    + [('candidate', f'{x:.3f}', f'{y:.3f}') for x, y in candidates])
    if args.detector != 'none':
        s = score_pins(pins, true_pos)
        row['det_precision'] = f"{s['precision']:.4f}"
        row['det_recall'] = f"{s['recall']:.4f}"
        row['duplicate_rate'] = f"{s['duplicate_rate']:.4f}"
        row['pin_error_m'] = f"{s['pin_error_m']:.4f}"
        print(f"[explore_eval] pins: {s['pins']} confirmed ({len(candidates)} candidates left),"
              f" {s['matched']} of {s['truth']} true survivors matched, precision"
              f" {s['precision']:.2f}, recall {s['recall']:.2f}, duplicates {s['duplicate_rate']:.2f},"
              f" pin error {s['pin_error_m']:.2f} m")
    if map_yaml:
        s = map_score.score(truth_map, map_score.load_map(map_yaml), spawn,
                            int(cfg['scoring']['map_tolerance_cells']))
        row['map_iou'], row['map_iou_tol'] = f"{s['iou']:.4f}", f"{s['iou_tol']:.4f}"
        row['map_accuracy'] = f"{s['accuracy']:.4f}"
        print(f"[explore_eval] map: IoU {s['iou']:.4f}, tolerance IoU {s['iou_tol']:.4f}")
    log.save_tum(os.path.join(run_dir, 'trajectory_slam.tum'),
                 os.path.join(run_dir, 'trajectory_truth.tum'))
    try:
        a = trajectory.ate(log)
        row['ate_rmse_m'], row['ate_aligned_rmse_m'] = f"{a['rmse']:.4f}", f"{a['rmse_aligned']:.4f}"
        print(f"[explore_eval] trajectory: ATE RMSE {a['rmse']:.3f} m over {a['n']} poses")
    except ImportError:
        print('[explore_eval] evo is not installed; trajectory error left as nan')

    append_row(os.path.join(out_dir, cfg['csv_name']), row)
    print(f'[explore_eval] status={status}; row appended to {os.path.join(out_dir, cfg["csv_name"])}')
    return 0 if status == 'ok' else 1


if __name__ == '__main__':
    sys.exit(main())
