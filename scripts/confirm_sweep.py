#!/usr/bin/env python3
"""Offline sweep of the confirmation rule on the final /survivors tracks of finished runs.

  python3 scripts/confirm_sweep.py results/phase5x x_rate0.0

The scorer only sees the final track set, and tracks form the same way whatever the
confirmation thresholds are, so keeping the confirmed tracks with hits >= N and mean confidence
>= C (and, optionally, suppressing a track that has a stronger one within `nms` metres) gives the
scores a run with those settings would have had. Prints mean pin precision, recall, duplicate
rate and number of pins over the runs, one line per setting.
"""
import csv
import itertools
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(__file__))
from pin_diagnostic import WORLDS, last_survivors  # noqa: E402
from rescue_eval.survivor_score import score_pins  # noqa: E402


def nms(tracks, radius):
    if radius <= 0:
        return tracks
    kept = []
    for t in sorted(tracks, key=lambda t: -t[2]):
        if all(math.hypot(t[0] - k[0], t[1] - k[1]) > radius for k in kept):
            kept.append(t)
    return kept


def main():
    root, method = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else None)
    runs = []
    for row in csv.DictReader(open(f'{root}/runs.csv')):
        if (method and row['method'] != method) or row['status'] != 'ok':
            continue
        msg = last_survivors(os.path.join(root, row['bag_path']))
        truth = [(s['position']['x'], s['position']['y']) for s in
                 yaml.safe_load(open(f"{WORLDS}/{row['world']}_survivors.yaml"))['survivors']]
        tracks = [(s.pose.position.x, s.pose.position.y, s.hit_count, s.confidence)
                  for s in msg.survivors if s.state == 'confirmed']
        runs.append((row['world'], row['seed'], tracks, truth))
    print(f'{len(runs)} runs from {root} ({method})')
    print(f"{'hits>=':>7} {'conf>=':>7} {'nms m':>6} | {'precision':>9} {'recall':>7} "
          f"{'dup rate':>8} {'pins/run':>8} {'runs with all found':>20}")
    for n, c, r in itertools.product((5, 20, 40, 80), (0.3, 0.6, 0.75), (0.0, 1.5)):
        res = []
        for _, _, tracks, truth in runs:
            keep = nms([t for t in tracks if t[2] >= n and t[3] >= c], r)
            res.append(score_pins([(t[0], t[1]) for t in keep], truth))
        prec = [x['precision'] for x in res if x['pins']]
        print(f"{n:>7} {c:>7.2f} {r:>6.1f} | {np.mean(prec) if prec else float('nan'):>9.2f} "
              f"{np.mean([x['recall'] for x in res]):>7.2f} "
              f"{np.mean([x['duplicate_rate'] for x in res if x['pins']] or [float('nan')]):>8.2f} "
              f"{np.mean([x['pins'] for x in res]):>8.1f} "
              f"{sum(x['matched'] == x['truth'] for x in res):>14} of {len(res)}")


if __name__ == '__main__':
    main()
