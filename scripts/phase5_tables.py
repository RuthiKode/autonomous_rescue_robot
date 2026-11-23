#!/usr/bin/env python3
"""Print the week-6 tables from results/ (CSV and JSON only): python3 scripts/phase5_tables.py"""
import csv
import json
import os

D = 'results/detector'


def f(x, n=2):
    try:
        return f'{float(x):.{n}f}'
    except (TypeError, ValueError):
        return 'n/a'


def detector_table():
    print('Held-out frame scores, conf >= 0.30, IoU >= 0.5 (all test frames of the held-out layout)')
    print('| model | held-out | pretrained P / R | fine-tuned P / R | tp / fp / fn |')
    print('|---|---|---|---|---|')
    for tag, label in (('', 'round 1'), ('_r2', 'round 2 (+3 worlds)')):
        for h in ('collapsed', 'rooms', 'open_plan'):
            p = f'{D}/{h}{tag}.json'
            if os.path.exists(p):
                d = json.load(open(p))
                a, b = d['pretrained']['all_seeds'], d['finetuned']['all_seeds']
                print(f"| {label} | {h} | {f(a['precision'])} / {f(a['recall'])} | "
                      f"{f(b['precision'])} / {f(b['recall'])} | {b['tp']} / {b['fp']} / {b['fn']} |")
    d = json.load(open(f'{D}/all.json'))
    a, b = d['pretrained']['all_seeds'], d['finetuned']['all_seeds']
    print(f"| all layouts, world seed 42 held out | all | {f(a['precision'])} / {f(a['recall'])} | "
          f"{f(b['precision'])} / {f(b['recall'])} | {b['tp']} / {b['fp']} / {b['fn']} |")

    print('\nConfidence sweep, round 1, fine-tuned')
    print('| held-out | conf 0.30 | conf 0.50 | conf 0.70 |')
    print('|---|---|---|---|')
    for h in ('collapsed', 'rooms', 'open_plan'):
        cells = []
        for suffix in ('', '_conf0.50', '_conf0.70'):
            p = f'{D}/{h}{suffix}.json'
            s = json.load(open(p))['finetuned']['all_seeds'] if os.path.exists(p) else None
            cells.append(f"{f(s['precision'])} / {f(s['recall'])}" if s else 'n/a')
        print(f'| {h} | ' + ' | '.join(cells) + ' |')

    print('\nRound 1, recall by group (fine-tuned, held-out layouts)')
    keys = ['posture=standing', 'posture=sitting', 'posture=lying', 'occluded=False',
            'occluded=True', 'height=<20px', 'height=20-60px', 'height=>=60px']
    print('| held-out | ' + ' | '.join(keys) + ' |')
    print('|---|' + '---|' * len(keys))
    for h in ('collapsed', 'rooms', 'open_plan'):
        g = json.load(open(f'{D}/{h}.json'))['finetuned']['all_seeds']['groups']
        print(f'| {h} | ' + ' | '.join(f"{f(g[k]['recall'])} (n={g[k]['n']})" if k in g else 'n/a'
                                       for k in keys) + ' |')


def run_table(root, label):
    print(f'\nPipeline runs: {label} ({root}/runs.csv)')
    print('| method | world | seed | status | coverage | true survivors pinned | confirmed pins | '
          'pin precision | duplicates | pin error (m) | RTF |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in csv.DictReader(open(f'{root}/runs.csv')):
        n = int(r['n_true_survivors'])
        rec = float(r['det_recall']) if r['det_recall'] not in ('', 'nan') else float('nan')
        pinned = round(rec * n) if rec == rec else 'n/a'
        prec = float(r['det_precision']) if r['det_precision'] not in ('', 'nan') else float('nan')
        total = round(pinned / prec) if prec == prec and prec > 0 and pinned != 'n/a' else 'n/a'
        print(f"| {r['method']} | {r['world']} | {r['seed']} | {r['status']} | "
              f"{f(r['coverage'], 3)} | {pinned} of {n} | {total} | "
              f"{f(prec)} | {f(r['duplicate_rate'])} | {f(r['pin_error_m'])} | "
              f"{f(r['real_time_factor'])} |")


if __name__ == '__main__':
    detector_table()
    for root, label in (('results/phase5', 'round 1 detectors, leave-one-layout-out'),
                        ('results/phase5r2', 'round 2 detectors, leave-one-layout-out'),
                        ('results/phase5all', 'all-layouts detector (world seed 42 unseen)'),
                        ('results/phase5x', 'round 4 detector (exploration frames), rule off / hit-rate'),
                        ('results/phase5y', 'round 4 detector + final pin rule')):
        run_table(root, label)
