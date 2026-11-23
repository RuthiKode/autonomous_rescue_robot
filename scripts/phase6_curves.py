#!/usr/bin/env python3
"""Coverage-versus-time curves and tables for the two selectors, from results/phase6 only.

  python3 scripts/phase6_curves.py [results/phase6] [docs/img/phase6]

Needs runs.csv and each run's coverage.csv. A run that stops early keeps its last coverage for the
rest of the time axis (coverage does not drop once the robot stops). Times are medians with the
range over the seeds in brackets; with 3 seeds that is all the spread there is.
"""
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = sys.argv[1] if len(sys.argv) > 1 else 'results/phase6'
OUT = sys.argv[2] if len(sys.argv) > 2 else 'docs/img/phase6'
METHODS = {'explore_lite': ('explore_lite', 'tab:blue'), 'explore_own': ('own selector', 'tab:red')}
WORLDS = ('collapsed', 'rooms', 'open_plan')
GRID = np.arange(0, 901, 5.0)


def load():
    runs = []
    for r in csv.DictReader(open(f'{ROOT}/runs.csv')):
        if r['status'] != 'ok' or r['method'] not in METHODS:
            continue
        path = f"{ROOT}/runs/{r['run_id']}/coverage.csv"
        if not os.path.exists(path):
            continue
        rows = list(csv.DictReader(open(path)))
        t = np.array([float(x['sim_time_s']) for x in rows])
        c = np.array([float(x['coverage']) for x in rows])
        runs.append({'world': r['world'], 'method': r['method'], 'seed': int(r['seed']), 't': t, 'c': c,
                     'row': r})
    return runs


def on_grid(run):
    return np.interp(GRID, run['t'], run['c'], right=run['c'][-1])


def time_to(run, level):
    hit = np.nonzero(run['c'] >= level)[0]
    return float(run['t'][hit[0]]) if hit.size else float('nan')


def med_range(values):
    v = np.array([x for x in values if x == x])
    if v.size == 0:
        return 'n/a'
    return f'{np.median(v):.0f} [{v.min():.0f}-{v.max():.0f}]' + ('' if v.size == len(values) else f' ({v.size}/{len(values)} runs)')


def main():
    runs = load()
    os.makedirs(OUT, exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2), sharey=True)
    for ax, w in zip(axes, WORLDS):
        for m, (label, colour) in METHODS.items():
            rs = [r for r in runs if r['world'] == w and r['method'] == m]
            if not rs:
                continue
            curves = np.array([on_grid(r) for r in rs])
            ax.fill_between(GRID, curves.min(0), curves.max(0), color=colour, alpha=0.15)
            ax.plot(GRID, curves.mean(0), color=colour, lw=2, label=f'{label} (mean of {len(rs)})')
            for cv in curves:
                ax.plot(GRID, cv, color=colour, lw=0.6, alpha=0.5)
        ax.set_title(w)
        ax.set_xlabel('sim time (s)')
        ax.set_xlim(0, 600)
        ax.axhline(0.9, color='gray', ls=':', lw=0.8)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('coverage of reachable free cells')
    axes[0].legend(loc='lower right')
    fig.tight_layout()
    fig.savefig(f'{OUT}/coverage_vs_time.png', dpi=120)

    lines = ['| world | selector | runs | time to 50 % (s) | time to 90 % (s) | time to 95 % (s) | '
             'coverage at 120 s | coverage at 240 s | final coverage | mission DONE |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for w in WORLDS:
        for m, (label, _) in METHODS.items():
            rs = [r for r in runs if r['world'] == w and r['method'] == m]
            if not rs:
                continue
            at = lambda t: [float(np.interp(t, r['t'], r['c'], right=r['c'][-1])) for r in rs]  # noqa: E731
            done = ('-' if m == 'explore_lite' else
                    f"{sum(r['row']['mission_completed'] == '1' for r in rs)} of {len(rs)}")
            lines.append(
                f"| {w} | {label} | {len(rs)} | {med_range([time_to(r, 0.5) for r in rs])} | "
                f"{med_range([time_to(r, 0.9) for r in rs])} | {med_range([time_to(r, 0.95) for r in rs])} | "
                f"{np.median(at(120)):.2f} | {np.median(at(240)):.2f} | "
                f"{np.median([r['c'][-1] for r in rs]):.3f} | {done} |")
    text = '\n'.join(lines)
    print(text)
    with open(f'{OUT}/tables.md', 'w') as f:
        f.write(text + '\n')


if __name__ == '__main__':
    main()
