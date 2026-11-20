#!/usr/bin/env python3
"""Fine-tune YOLOv8n on the synthetic survivor dataset and score it on a held-out world layout.

Run with the YOLO venv:  ~/venvs/yolo/bin/python scripts/train_detector.py <command> ...

  train  --holdout rooms   train on the other two layouts (seed 5 worlds are the validation set)
  eval   --holdout rooms   precision/recall on every frame of the held-out layout, for the
                           pretrained COCO model and the fine-tuned model, with breakdowns by
                           posture, occlusion and box height. Writes results/detector/<holdout>.json

Split by world layout: a model never sees the layout it is tested on.
"""
import argparse
import csv
import json
import os
import shutil
import sys

import numpy as np

DATA = 'results/dataset'
OUT = 'results/detector'
LAYOUTS = ('rooms', 'collapsed', 'open_plan')
VAL_SEED = '5'
TEST_SEEDS = {'42', '1', '2', '3', '4', '5'}   # worlds 6 and up are extra training data only
PRETRAINED = os.path.expanduser('~/venvs/yolo/weights/yolov8n.pt')


def read_meta():
    rows = list(csv.DictReader(open(f'{DATA}/meta.csv')))
    for r in rows:
        r['boxes'] = json.loads(r['boxes'])
    return rows


def link_split(rows, name, root):
    for sub in ('images', 'labels'):
        os.makedirs(f'{root}/{name}/{sub}', exist_ok=True)
    for r in rows:
        for sub, ext in (('images', 'png'), ('labels', 'txt')):
            dst = f"{root}/{name}/{sub}/{r['image']}.{ext}"
            if not os.path.exists(dst):
                os.symlink(os.path.abspath(f"{DATA}/{sub}/{r['image']}.{ext}"), dst)


def make_fold(holdout):
    rows = read_meta()
    root = f'{DATA}/folds/{holdout}'
    shutil.rmtree(root, ignore_errors=True)
    train = [r for r in rows if r['layout'] != holdout and r['seed'] != VAL_SEED]
    val = [r for r in rows if r['layout'] != holdout and r['seed'] == VAL_SEED]
    test = [r for r in rows if r['layout'] == holdout and r['seed'] in TEST_SEEDS]
    for name, part in (('train', train), ('val', val), ('test', test)):
        link_split(part, name, root)
    with open(f'{root}/data.yaml', 'w') as f:
        f.write(f"path: {os.path.abspath(root)}\ntrain: train/images\nval: val/images\n"
                f"test: test/images\nnames:\n  0: person\n")
    return root, (len(train), len(val), len(test))


def cmd_train(args):
    from ultralytics import YOLO
    root, n = make_fold(args.holdout)
    print(f'fold {args.holdout}: train {n[0]}, val {n[1]}, test {n[2]} frames')
    model = YOLO(PRETRAINED)
    model.train(data=f'{root}/data.yaml', epochs=args.epochs, imgsz=args.imgsz, batch=16,
                device=0, workers=2, project=os.path.abspath(f'{OUT}/runs'),
                name=args.holdout + args.tag, exist_ok=True, patience=15, seed=0, verbose=False,
                plots=False)


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def height_bucket(h):
    return '<20px' if h < 20 else '20-60px' if h < 60 else '>=60px'


def score(model, rows, conf, imgsz):
    """Match predictions to truth boxes per frame (IoU >= 0.5, highest confidence first)."""
    tp = fp = fn = 0
    groups = {}

    def bump(key, hit):
        g = groups.setdefault(key, [0, 0])
        g[0] += hit
        g[1] += 1

    def predictions():
        # chunks of 16 frames: one big list ran the 6 GB GPU out of memory
        for i in range(0, len(rows), 16):
            chunk = [f"{DATA}/images/{r['image']}.png" for r in rows[i:i + 16]]
            yield from model.predict(chunk, imgsz=imgsz, conf=conf, classes=[0], device=0,
                                     verbose=False)

    for r, res in zip(rows, predictions()):
        preds = sorted(zip(res.boxes.conf.tolist(), res.boxes.xyxy.tolist()), reverse=True)
        gts = r['boxes']
        used = set()
        for _, pb in preds:
            best, best_i = 0.5, -1
            for i, g in enumerate(gts):
                if i not in used and iou(pb, g[:4]) >= best:
                    best, best_i = iou(pb, g[:4]), i
            if best_i < 0:
                fp += 1
            else:
                used.add(best_i)
                tp += 1
        for i, g in enumerate(gts):
            hit = int(i in used)
            fn += 1 - hit
            bump(('posture', g[5] or 'unassigned'), hit)
            bump(('occluded', str(g[6])), hit)
            bump(('height', height_bucket(g[3] - g[1])), hit)
    p = tp / (tp + fp) if tp + fp else float('nan')
    r = tp / (tp + fn) if tp + fn else float('nan')
    return {'tp': tp, 'fp': fp, 'fn': fn, 'precision': p, 'recall': r,
            'frames': len(rows), 'frames_with_survivor': sum(1 for x in rows if x['boxes']),
            'groups': {f'{k[0]}={k[1]}': {'recall': v[0] / v[1], 'n': v[1]}
                       for k, v in sorted(groups.items())}}


def cmd_eval(args):
    from ultralytics import YOLO
    rows = [r for r in read_meta() if r['layout'] == args.holdout and r['seed'] in TEST_SEEDS]
    seed42 = [r for r in rows if r['seed'] == '42']
    result = {'holdout': args.holdout, 'conf': args.conf, 'imgsz': args.imgsz}
    models = {'pretrained': PRETRAINED,
              'finetuned': os.path.abspath(f'{OUT}/runs/{args.holdout}{args.tag}/weights/best.pt')}
    for name, path in models.items():
        m = YOLO(path)
        result[name] = {'all_seeds': score(m, rows, args.conf, args.imgsz),
                        'seed42': score(m, seed42, args.conf, args.imgsz)}
        s = result[name]['all_seeds']
        print(f"{args.holdout} {name:<10} P {s['precision']:.3f} R {s['recall']:.3f} "
              f"(tp {s['tp']}, fp {s['fp']}, fn {s['fn']})")
    os.makedirs(OUT, exist_ok=True)
    with open(f'{OUT}/{args.holdout}{args.tag}.json', 'w') as f:
        json.dump(result, f, indent=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    for name in ('train', 'eval'):
        p = sub.add_parser(name)
        p.add_argument('--holdout', required=True, choices=LAYOUTS)
        p.add_argument('--imgsz', type=int, default=640)
        p.add_argument('--tag', default='', help='suffix of the run name (e.g. _r2 for round 2)')
        if name == 'train':
            p.add_argument('--epochs', type=int, default=40)
        else:
            p.add_argument('--conf', type=float, default=0.30,
                           help='detection threshold (the localizer uses 0.30)')
    args = ap.parse_args()
    return {'train': cmd_train, 'eval': cmd_eval}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
