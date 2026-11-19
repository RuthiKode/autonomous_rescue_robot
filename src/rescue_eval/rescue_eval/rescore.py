"""Add the current map metrics to an existing results CSV by rescoring the saved maps.

  ros2 run rescue_eval rescore --csv results/phase2_slip/runs.csv

Rows keep every other value. The old file is kept next to it as <name>.before_rescore.
Only rows with a saved map (map_path) are rescored.
"""
import argparse
import csv
import os
import shutil
import sys

from ament_index_python.packages import get_package_share_directory
import yaml

from rescue_eval import map_score
from rescue_eval.metrics import CSV_COLUMNS


def main(argv=None):
    share = get_package_share_directory('rescue_eval')
    sim_share = get_package_share_directory('rescue_simulation')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--csv', required=True)
    parser.add_argument('--config', default=os.path.join(share, 'config', 'eval.yaml'))
    args = parser.parse_args(argv)
    with open(args.config) as f:
        tol = int(yaml.safe_load(f)['scoring']['map_tolerance_cells'])
    base = os.path.dirname(os.path.abspath(args.csv))
    with open(args.csv, newline='') as f:
        rows = list(csv.DictReader(f))
    shutil.copyfile(args.csv, args.csv + '.before_rescore')
    done = 0
    for r in rows:
        r.setdefault('map_iou_tol', 'nan')
        if not r.get('map_path'):
            continue
        truth = map_score.load_map(os.path.join(sim_share, 'worlds', r['world'] + '_truth_map.yaml'))
        with open(os.path.join(sim_share, 'worlds', r['world'] + '_survivors.yaml')) as f:
            spawn = yaml.safe_load(f)['robot_spawn']
        s = map_score.score(truth, map_score.load_map(os.path.join(base, r['map_path'])),
                            (spawn['x'], spawn['y'], spawn['yaw']), tol)
        r['map_iou'], r['map_iou_tol'] = f"{s['iou']:.4f}", f"{s['iou_tol']:.4f}"
        r['map_accuracy'] = f"{s['accuracy']:.4f}"
        done += 1
    with open(args.csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, restval='nan', extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    print(f'rescored {done} of {len(rows)} rows with tolerance {tol} cell(s): {args.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
