#!/usr/bin/env python3
"""Where do false pins sit? For each false track: distance to the nearest true survivor and to the
nearest obstacle centre, with its hits and confidence. Usage: false_pin_where.py <runs-dir> <csv-row-index>"""
import csv
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from pin_diagnostic import WORLDS, last_survivors  # noqa: E402
from rescue_simulation import generate_worlds as gw  # noqa: E402


def main():
    root, idx = sys.argv[1], int(sys.argv[2])
    row = list(csv.DictReader(open(f'{root}/runs.csv')))[idx]
    msg = last_survivors(os.path.join(root, row['bag_path']))
    cfg = gw.load_config()
    b = gw.build(row['world'], dict(cfg['worlds'][row['world']]), cfg['common'], cfg['seed'])
    truth = yaml.safe_load(open(f"{WORLDS}/{row['world']}_survivors.yaml"))['survivors']
    t_xy = np.array([[s['position']['x'], s['position']['y']] for s in truth])
    obst = np.array([[s['pose'][0], s['pose'][1]] for s in b.shapes])
    print(row['world'], 'seed', row['seed'], 'method', row['method'])
    print(f"{'hits':>5} {'conf':>5} {'to survivor':>12} {'to obstacle':>12}  state")
    for s in sorted(msg.survivors, key=lambda s: -s.hit_count):
        p = np.array([s.pose.position.x, s.pose.position.y])
        ds, do = np.hypot(*(t_xy - p).T).min(), np.hypot(*(obst - p).T).min()
        print(f'{s.hit_count:>5} {s.confidence:>5.2f} {ds:>10.2f} m {do:>10.2f} m  {s.state}')


if __name__ == '__main__':
    main()
