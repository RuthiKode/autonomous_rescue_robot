#!/usr/bin/env python3
"""Every table and plot of the evaluation, generated from the run CSV files only.

  python3 scripts/make_tables.py [results/sweep] [docs/results]

Reads <results>/runs.csv and each run's coverage.csv, writes into the output directory:
  summary.md           per world and condition: mean +- standard deviation over seeds, n
  paired.md            paired differences between conditions on the same (world, seed), with 95 % CI
  coverage_curves.png  coverage versus sim time, per world, one line per condition
  pins.png             pin precision and recall per condition
  summary.csv          the numbers behind summary.md

Runs that did not end with status ok are listed (not averaged). The standard deviation is the sample
one (n - 1); a "t95" interval uses the t distribution with n - 1 degrees of freedom.
"""
import csv
import math
import os
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

ROOT = sys.argv[1] if len(sys.argv) > 1 else 'results/sweep'
OUT = sys.argv[2] if len(sys.argv) > 2 else 'docs/results'
CONDITIONS = [('random_ft', 'random goals'), ('lite_ft', 'explore_lite'),
              ('own_ft', 'own selector'), ('ownverify_ft', 'own + verification'),
              ('ownverify_pre', 'own + verification, pretrained YOLO')]
LABEL = dict(CONDITIONS)
WORLDS = ('collapsed', 'rooms', 'open_plan')
METRICS = [  # column, title, unit/format
    ('coverage', 'final coverage', '{:.3f}'),
    ('time_to_90_coverage_s', 'time to 90 % coverage (s)', '{:.0f}'),
    ('mission_completed', 'mission DONE (share of runs)', '{:.2f}'),
    ('mission_time_s', 'mission time when DONE (s)', '{:.0f}'),
    ('det_precision', 'pin precision', '{:.2f}'),
    ('det_recall', 'pin recall', '{:.2f}'),
    ('duplicate_rate', 'duplicate rate', '{:.2f}'),
    ('pin_error_m', 'pin error (m)', '{:.2f}'),
    ('recoveries', 'recoveries per run', '{:.1f}'),
    ('map_iou_tol', 'map IoU (1-cell tolerance)', '{:.2f}'),
    ('ate_rmse_m', 'ATE RMSE (m)', '{:.3f}'),
]
PAIRS = [('own_ft', 'lite_ft', 'own selector vs explore_lite'),
         ('own_ft', 'random_ft', 'own selector vs random goals'),
         ('ownverify_ft', 'own_ft', 'verification vs none (pins counted: verified vs confirmed)'),
         ('ownverify_ft', 'ownverify_pre', 'fine-tuned vs pretrained YOLO, both with verification')]
PAIR_METRICS = ['time_to_90_coverage_s', 'coverage', 'det_precision', 'det_recall', 'duplicate_rate']


def num(x):
    try:
        v = float(x)
    except (TypeError, ValueError):
        return float('nan')
    return v


def load():
    ok, bad = [], []
    for r in csv.DictReader(open(f'{ROOT}/runs.csv')):
        (ok if r['status'] == 'ok' else bad).append(r)
    return ok, bad


def cell(values, fmt):
    v = np.array([x for x in values if x == x])
    if v.size == 0:
        return 'n/a'
    sd = float(v.std(ddof=1)) if v.size > 1 else float('nan')
    sd_text = fmt.format(sd) if sd == sd else '-'
    n_note = '' if v.size == len(values) else f' ({v.size}/{len(values)})'
    return f'{fmt.format(v.mean())} +- {sd_text}{n_note}'


def summary(ok):
    groups = defaultdict(list)
    for r in ok:
        groups[(r['world'], r['method'])].append(r)
    lines, rows = [], []
    for col, title, fmt in METRICS:
        lines += [f'\n### {title}\n', '| condition | ' + ' | '.join(WORLDS) + ' |',
                  '|---|' + '---|' * len(WORLDS)]
        for key, label in CONDITIONS:
            cells = []
            for w in WORLDS:
                rs = groups.get((w, key), [])
                if not rs:
                    cells.append('-')
                    continue
                vals = [num(r[col]) for r in rs]
                cells.append(cell(vals, fmt) + f' [n={len(rs)}]')
                rows.append({'metric': col, 'world': w, 'condition': key, 'n': len(rs),
                             'mean': np.nanmean(vals) if any(v == v for v in vals) else float('nan'),
                             'sd': np.nanstd(vals, ddof=1) if sum(v == v for v in vals) > 1 else float('nan')})
            lines.append(f'| {label} | ' + ' | '.join(cells) + ' |')
    return '\n'.join(lines), rows


def paired(ok):
    by = {(r['world'], r['method'], r['seed']): r for r in ok}
    lines = ['| comparison | metric | pairs | mean difference (A - B) | 95 % CI (t) |', '|---|---|---|---|---|']
    for a, b, title in PAIRS:
        for col in PAIR_METRICS:
            diffs = []
            for (w, m, s), r in by.items():
                if m != a:
                    continue
                other = by.get((w, b, s))
                if other is None:
                    continue
                x, y = num(r[col]), num(other[col])
                if x == x and y == y:
                    diffs.append(x - y)
            if len(diffs) < 3:
                lines.append(f'| {title} (A = {LABEL[a]}, B = {LABEL[b]}) | {col} | {len(diffs)} | n/a | n/a |')
                continue
            d = np.array(diffs)
            half = stats.t.ppf(0.975, len(d) - 1) * d.std(ddof=1) / math.sqrt(len(d))
            lines.append(f'| {title} (A = {LABEL[a]}, B = {LABEL[b]}) | {col} | {len(d)} | '
                         f'{d.mean():+.3f} | [{d.mean() - half:+.3f}, {d.mean() + half:+.3f}] |')
    return '\n'.join(lines)


def curves(ok):
    grid = np.arange(0, 901, 5.0)
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.4), sharey=True)
    colours = dict(zip([k for k, _ in CONDITIONS], ['tab:gray', 'tab:blue', 'tab:red', 'tab:green', 'tab:purple']))
    for ax, w in zip(axes, WORLDS):
        for key, label in CONDITIONS:
            ys = []
            for r in ok:
                if r['world'] != w or r['method'] != key:
                    continue
                path = f"{ROOT}/runs/{r['run_id']}/coverage.csv"
                if not os.path.exists(path):
                    continue
                rows = list(csv.DictReader(open(path)))
                if not rows:
                    continue
                t = np.array([float(x['sim_time_s']) for x in rows])
                c = np.array([float(x['coverage']) for x in rows])
                ys.append(np.interp(grid, t, c, right=c[-1]))
            if ys:
                ys = np.array(ys)
                ax.plot(grid, ys.mean(0), color=colours[key], lw=2, label=f'{label} (n={len(ys)})')
                ax.fill_between(grid, ys.mean(0) - ys.std(0), ys.mean(0) + ys.std(0), color=colours[key], alpha=0.12)
        ax.set_title(w)
        ax.set_xlim(0, 700)
        ax.set_xlabel('sim time (s)')
        ax.axhline(0.9, color='k', ls=':', lw=0.6)
        ax.grid(alpha=0.3)
    axes[0].set_ylabel('coverage (mean +- sd)')
    axes[0].legend(fontsize=7, loc='lower right')
    fig.tight_layout()
    fig.savefig(f'{OUT}/coverage_curves.png', dpi=120)
    plt.close(fig)


def pin_bars(ok):
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, (col, title) in zip(axes, [('det_precision', 'pin precision'), ('det_recall', 'pin recall')]):
        width = 0.16
        for i, (key, label) in enumerate(CONDITIONS):
            means, sds = [], []
            for w in WORLDS:
                v = np.array([num(r[col]) for r in ok if r['world'] == w and r['method'] == key])
                v = v[~np.isnan(v)]
                means.append(v.mean() if v.size else 0.0)
                sds.append(v.std(ddof=1) if v.size > 1 else 0.0)
            ax.bar(np.arange(3) + (i - 2) * width, means, width, yerr=sds, label=label, capsize=2)
        ax.set_xticks(range(3))
        ax.set_xticklabels(WORLDS)
        ax.set_ylim(0, 1.05)
        ax.set_title(title)
        ax.grid(axis='y', alpha=0.3)
    axes[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(f'{OUT}/pins.png', dpi=120)
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    ok, bad = load()
    text, rows = summary(ok)
    with open(f'{OUT}/summary.md', 'w') as f:
        f.write(f'Generated by scripts/make_tables.py from {ROOT}/runs.csv ({len(ok)} runs with status ok).\n')
        f.write('Cells: mean +- sample standard deviation over seeds [n = runs]; "(k/n)" = only k of n runs had the value.\n')
        f.write(text + '\n')
        if bad:
            f.write('\n### Runs that did not end with status ok (not averaged)\n\n| world | condition | seed | status |\n|---|---|---|---|\n')
            f.writelines(f"| {r['world']} | {r['method']} | {r['seed']} | {r['status']} |\n" for r in bad)
    with open(f'{OUT}/paired.md', 'w') as f:
        f.write(paired(ok) + '\n')
    with open(f'{OUT}/summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['metric', 'world', 'condition', 'n', 'mean', 'sd'])
        w.writeheader()
        w.writerows(rows)
    curves(ok)
    pin_bars(ok)
    print(f'{len(ok)} ok runs, {len(bad)} other; wrote {OUT}/')


if __name__ == '__main__':
    main()
