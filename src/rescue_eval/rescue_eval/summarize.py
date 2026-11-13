"""Print mean and standard deviation of the measured metrics per world, as a Markdown table.

  ros2 run rescue_eval summarize --csv results/runs.csv --method slam_replay
"""
import argparse
import csv
import math
import statistics
import sys

METRICS = [('map_iou', 'IoU (occupied)'), ('map_accuracy', 'Accuracy'),
           ('ate_rmse_m', 'ATE RMSE (m)'), ('ate_aligned_rmse_m', 'ATE RMSE, aligned (m)')]


def summarise(rows):
    """Return {world: {metric: (mean, stdev, n)}} for rows with status ok."""
    out = {}
    for world in sorted({r['world'] for r in rows}):
        out[world] = {}
        for key, _ in METRICS:
            vals = [float(r[key]) for r in rows if r['world'] == world and r['status'] == 'ok'
                    and not math.isnan(float(r[key]))]
            if vals:
                sd = statistics.stdev(vals) if len(vals) > 1 else float('nan')
                out[world][key] = (statistics.fmean(vals), sd, len(vals))
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--csv', default='results/runs.csv')
    parser.add_argument('--method', default=None, help='only rows with this method label')
    args = parser.parse_args(argv)
    with open(args.csv, newline='') as f:
        rows = [r for r in csv.DictReader(f) if args.method in (None, r['method'])]
    not_ok = [r for r in rows if r['status'] != 'ok']
    print('| World | Runs | ' + ' | '.join(label for _, label in METRICS) + ' |')
    print('| --- | --- | ' + ' | '.join('---' for _ in METRICS) + ' |')
    for world, stats in summarise(rows).items():
        n = max((s[2] for s in stats.values()), default=0)
        cells = [f'{stats[k][0]:.4f} +/- {stats[k][1]:.4f}' if k in stats else 'n/a'
                 for k, _ in METRICS]
        print(f'| {world} | {n} | ' + ' | '.join(cells) + ' |')
    if not_ok:
        print(f'\n{len(not_ok)} run(s) excluded because their status is not ok: '
              + ', '.join(f"{r['run_id']} ({r['status']})" for r in not_ok))
    return 0


if __name__ == '__main__':
    sys.exit(main())
