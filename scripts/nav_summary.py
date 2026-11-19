#!/usr/bin/env python3
"""Summarise the per-goal logs of Phase 3 navigation runs.

  python3 scripts/nav_summary.py results/phase3 [--method nav_baseline]

Reads runs.csv and every runs/<run_id>/goals.csv. Prints, per world: runs, goals, success
rate (mean +/- standard deviation over runs), failures by kind, recoveries by kind, mean time
and path length of successful goals, and goals flagged stuck_without_recovery.
"""
import argparse
import collections
import csv
import math
import os
import statistics
import sys

NAV_ERROR = {
    '0': 'none', '103': 'FollowPath: invalid path', '104': 'FollowPath: patience exceeded',
    '105': 'FollowPath: failed to make progress', '106': 'FollowPath: no valid control',
    '203': 'planner: start outside map', '204': 'planner: goal outside map',
    '205': 'planner: start occupied', '206': 'planner: goal occupied',
    '207': 'planner: timeout', '208': 'planner: no valid path', '': 'none (timed out)',
}


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('results_dir')
    p.add_argument('--method', default=None)
    args = p.parse_args()
    with open(os.path.join(args.results_dir, 'runs.csv'), newline='') as f:
        runs = [r for r in csv.DictReader(f) if args.method in (None, r['method'])]
    by_world = collections.defaultdict(list)
    for r in runs:
        path = os.path.join(args.results_dir, 'runs', r['run_id'], 'goals.csv')
        if os.path.isfile(path):
            with open(path, newline='') as f:
                by_world[r['world']].append((r, list(csv.DictReader(f))))
    for world in ('rooms', 'collapsed', 'open_plan'):
        if world not in by_world:
            continue
        runs_w = by_world[world]
        rates = [sum(int(g['success']) for g in goals) / len(goals) for _, goals in runs_w if goals]
        goals = [g for _, gs in runs_w for g in gs]
        ok = [g for g in goals if g['success'] == '1']
        sd = statistics.stdev(rates) if len(rates) > 1 else float('nan')
        print(f'## {world}: {len(runs_w)} run(s), {len(goals)} goals, success'
              f' {100 * len(ok) / len(goals):.1f}% (per-run rates: '
              + ', '.join(f'{100 * r:.0f}%' for r in rates)
              + f'; mean {100 * statistics.fmean(rates):.1f}% +/- {100 * sd:.1f})')
        if ok:
            print(f'   successful goals: mean {statistics.fmean(float(g["sim_time_s"]) for g in ok):.1f} s,'
                  f' path {statistics.fmean(float(g["path_length_m"]) for g in ok):.1f} m,'
                  f' final error {statistics.fmean(float(g["final_error_m"]) for g in ok):.2f} m')
        fails = collections.Counter(
            f"{g['status']} / {NAV_ERROR.get(g['nav_error_code'], g['nav_error_code'])}"
            for g in goals if g['success'] != '1')
        print('   failures: ' + ('; '.join(f'{n} x {k}' for k, n in fails.most_common()) or 'none'))
        kinds = collections.Counter(k for g in goals for k in g['recovery_events'].split(';') if k)
        print('   recovery events: ' + ('; '.join(f'{n} x {k}' for k, n in kinds.most_common()) or 'none')
              + f" (reported by Nav2: {sum(int(g['recoveries_reported']) for g in goals)})")
        stuck = [g for g in goals if g['stuck_without_recovery'] == '1']
        print(f'   stuck without recovery: {len(stuck)}')
        for r, gs in runs_w:
            row = [g for g in gs if g['success'] != '1']
            first = min((int(g['goal_index']) for g in row), default=None)
            print(f"   {r['run_id'][:34]:<36} {sum(int(g['success']) for g in gs)}/{len(gs)}"
                  f"  first failure at goal {first if first else '-'}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
