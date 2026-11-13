"""Compare raw wheel odometry and the EKF against Gazebo truth over repeated drives.

  ros2 run rescue_eval ekf_validation --trials 10 --square-runs 3 --output results/ekf_validation.csv

Every trial starts a fresh headless simulation with its own Gazebo seed. Errors
are computed on the motion between the start and the end of the drive, each
expressed in its own start frame, so a constant offset between frames cancels.
"""
import argparse
import csv
import math
import os
import statistics
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
import rclpy
import yaml

from rescue_eval.driving import DriveNode, relative, wrap
from rescue_eval.sim_runner import launch_sim, spin_until

FIELDS = ['world', 'test', 'trial', 'seed', 'status', 'true_dx', 'true_dy', 'true_dyaw',
          'raw_yaw_err_rad', 'ekf_yaw_err_rad', 'raw_pos_err_m', 'ekf_pos_err_m',
          'vx_err_var', 'vyaw_err_var']


def _errors(start, end):
    true = relative(start['truth'], end['truth'])
    out = {'true_dx': true[0], 'true_dy': true[1], 'true_dyaw': true[2]}
    for name in ('raw', 'ekf'):
        est = relative(start[name], end[name])
        out[f'{name}_yaw_err_rad'] = wrap(est[2] - true[2])
        out[f'{name}_pos_err_m'] = math.hypot(est[0] - true[0], est[1] - true[1])
    return out


def run_trial(world, test, seed, cfg, log_path, launch_args=()):
    sim = launch_sim(world, seed, log_path, extra_args=launch_args)
    node = DriveNode()
    row = {'status': 'ok'}
    try:
        result = spin_until(node, lambda: node.ready() and node.sim_time > 2.0,
                            cfg['startup_timeout_s'], sim.alive)
        if result != 'ok':
            row['status'] = 'startup_' + result
            return row
        node.stop_and_settle(cfg['settle_s'], sim.alive)
        start = node.poses()
        node.twist_log = []
        if test == 'drive':
            d = cfg['drive']
            node.hold(d['forward_speed'], 0.0, d['forward_s'], alive=sim.alive)
            node.hold(0.0, 0.0, d['pause_s'], alive=sim.alive)
            node.hold(0.0, d['turn_rate'], d['turn_s'], alive=sim.alive)
        else:
            corners = [tuple(c) for c in cfg['square']['corners']] * cfg['square']['laps']
            result = node.follow(corners, cfg['follower'], sim.alive)
            if result != 'ok':
                row['status'] = 'follow_' + result
        node.stop_and_settle(cfg['settle_s'], sim.alive)
        samples, node.twist_log = node.twist_log, None
        row.update(_errors(start, node.poses()))
        # Twist error while actually moving: the basis for the odometry covariances.
        moving_v = [s[0] for s in samples if s[2] > 0.05]
        moving_w = [s[1] for s in samples if s[3] > 0.05]
        row['vx_err_var'] = statistics.pvariance(moving_v) + statistics.fmean(moving_v) ** 2 \
            if len(moving_v) > 1 else float('nan')
        row['vyaw_err_var'] = statistics.pvariance(moving_w) + statistics.fmean(moving_w) ** 2 \
            if len(moving_w) > 1 else float('nan')
    finally:
        node.destroy_node()
        sim.stop()
    return row


def summarise(rows):
    """Print mean and standard deviation per world and test."""
    print(f"\n{'world':<10}{'test':<8}{'n':>3}  {'raw yaw err rad':>22}  {'EKF yaw err rad':>22}"
          f"  {'raw pos drift m':>20}  {'EKF pos drift m':>20}")
    for key in sorted({(r['world'], r['test']) for r in rows}):
        sel = [r for r in rows if (r['world'], r['test']) == key and r['status'] == 'ok']
        if not sel:
            continue

        def ms(field, fmt):
            vals = [r[field] for r in sel]
            sd = statistics.stdev(vals) if len(vals) > 1 else float('nan')
            return f'{statistics.fmean(vals):{fmt}} +/- {sd:{fmt}}'
        print(f'{key[0]:<10}{key[1]:<8}{len(sel):>3}  {ms("raw_yaw_err_rad", "+.4f"):>22}'
              f'  {ms("ekf_yaw_err_rad", "+.4f"):>22}  {ms("raw_pos_err_m", ".4f"):>20}'
              f'  {ms("ekf_pos_err_m", ".4f"):>20}')
    ok = [r for r in rows if r['status'] == 'ok']
    for field in ('vx_err_var', 'vyaw_err_var'):
        vals = [r[field] for r in ok if not math.isnan(r[field])]
        if vals:
            print(f'mean square {field[:-8]} twist error (odom - truth) while moving:'
                  f' {statistics.fmean(vals):.3e}')
    failed = [r for r in rows if r['status'] != 'ok']
    if failed:
        print(f'{len(failed)} trial(s) did not finish cleanly:',
              ', '.join(f"{r['world']}/{r['test']}/{r['trial']}={r['status']}" for r in failed))


def main(argv=None):
    share = get_package_share_directory('rescue_eval')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--worlds', nargs='+', default=['rooms', 'collapsed', 'open_plan'])
    parser.add_argument('--trials', type=int, default=10, help='open-loop drive trials per world')
    parser.add_argument('--square-runs', type=int, default=3, help='square-loop runs per world')
    parser.add_argument('--output', default='results/ekf_validation.csv')
    parser.add_argument('--config', default=os.path.join(share, 'config', 'ekf_validation.yaml'))
    parser.add_argument('--launch-arg', action='append', default=[],
                        help='extra launch argument, e.g. ekf:=true')
    args = parser.parse_args(argv)
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    log_path = os.path.join(tempfile.gettempdir(), 'ekf_validation.launch.log')
    rows = []
    rclpy.init()
    try:
        with open(args.output, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=FIELDS, restval='nan')
            writer.writeheader()
            for world in args.worlds:
                plan = [('drive', i) for i in range(1, args.trials + 1)]
                plan += [('square', i) for i in range(1, args.square_runs + 1)]
                for test, trial in plan:
                    seed = trial if test == 'drive' else 100 + trial
                    row = run_trial(world, test, seed, cfg, log_path, args.launch_arg)
                    row.update({'world': world, 'test': test, 'trial': trial, 'seed': seed})
                    rows.append(row)
                    writer.writerow({k: (f'{v:.6g}' if isinstance(v, float) else v)
                                     for k, v in row.items()})
                    f.flush()
                    if row['status'] == 'ok':
                        print(f"{world} {test} {trial}: raw yaw {row['raw_yaw_err_rad']:+.4f}"
                              f" ekf yaw {row['ekf_yaw_err_rad']:+.4f}"
                              f" raw pos {row['raw_pos_err_m']:.4f}"
                              f" ekf pos {row['ekf_pos_err_m']:.4f}", flush=True)
                    else:
                        print(f"{world} {test} {trial}: {row['status']}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
    summarise(rows)
    print(f'rows written to {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
