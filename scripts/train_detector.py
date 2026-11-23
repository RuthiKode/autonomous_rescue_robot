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


EXPLORE = 'results/dataset_explore'     # frames logged while exploring (collect_explore.sh)
EXPLORE_VAL_SEED = '13'                 # exploration worlds 11 and 12 train, 13 validates


def read_meta():
    rows = list(csv.DictReader(open(f'{DATA}/meta.csv')))
    for r in rows:
        r['boxes'] = json.loads(r['boxes'])
        r['dir'] = DATA
    return rows


def read_explore():
    """Frames logged along exploration paths; tag = <layout>_collect_s<seed>_seed<seed>_<time>."""
    import re
    rows = list(csv.DictReader(open(f'{EXPLORE}/meta.csv')))
    for r in rows:
        m = re.match(r'(\w+?)_collect_s(\d+)_', r['tag'])
        r['layout'], r['seed'] = m.group(1), m.group(2)
        r['boxes'] = json.loads(r['boxes'])
        r['dir'] = EXPLORE
    return rows


def link_split(rows, name, root, copies=None):
    """Symlink frames into root/name. copies = {image: n} adds n extra copies of those frames."""
    for sub in ('images', 'labels'):
        os.makedirs(f'{root}/{name}/{sub}', exist_ok=True)
    for r in rows:
        for k in range(1 + (copies or {}).get(r['image'], 0)):
            for sub, ext in (('images', 'png'), ('labels', 'txt')):
                dst = f"{root}/{name}/{sub}/{r['image']}{'' if k == 0 else f'_dup{k}'}.{ext}"
                if not os.path.exists(dst):
                    os.symlink(os.path.abspath(f"{r['dir']}/{sub}/{r['image']}.{ext}"), dst)


def mine_false_positives(weights, rows, conf=0.30, imgsz=640):
    """Names of frames where the detector fires on something that is not labelled."""
    from ultralytics import YOLO
    model = YOLO(weights)
    names = set()
    for i in range(0, len(rows), 16):
        chunk = rows[i:i + 16]
        for r, res in zip(chunk, model.predict([f"{r['dir']}/images/{r['image']}.png" for r in chunk],
                                               imgsz=imgsz, conf=conf, classes=[0], device=0,
                                               verbose=False)):
            if any(not any(iou(b, g[:4]) >= 0.5 for g in r['boxes'])
                   for b in res.boxes.xyxy.tolist()):
                names.add(r['image'])
    return names


def split(holdout, rows):
    """(train, val, test) frames.

    holdout = a layout name: train on the other layouts, test on that layout (seeds in TEST_SEEDS).
    holdout = 'all': the detector the pipeline uses. It trains on every layout but not on world
    seed 42 (the committed worlds the evaluation runs in), which is the test set; layouts are
    seen, survivor and obstacle placements are not.
    """
    if holdout == 'all':
        return ([r for r in rows if r['seed'] not in ('42', VAL_SEED)],
                [r for r in rows if r['seed'] == VAL_SEED],
                [r for r in rows if r['seed'] == '42'])
    return ([r for r in rows if r['layout'] != holdout and r['seed'] != VAL_SEED],
            [r for r in rows if r['layout'] != holdout and r['seed'] == VAL_SEED],
            [r for r in rows if r['layout'] == holdout and r['seed'] in TEST_SEEDS])


def make_fold(holdout, tag='', explore=False, mine=None, dup=2):
    rows = read_meta()
    root = f'{DATA}/folds/{holdout}{tag}'
    shutil.rmtree(root, ignore_errors=True)
    train, val, test = split(holdout, rows)
    copies = None
    if explore:
        ex = [r for r in read_explore() if holdout == 'all' or r['layout'] != holdout]
        ex_train = [r for r in ex if r['seed'] != EXPLORE_VAL_SEED]
        val = val + [r for r in ex if r['seed'] == EXPLORE_VAL_SEED]
        copies = {}
        if mine:
            hard = mine_false_positives(mine, ex_train)
            copies = {name: dup for name in hard}
            os.makedirs(OUT, exist_ok=True)
            json.dump({'weights': mine, 'frames': len(ex_train), 'false_positive_frames': len(hard),
                       'extra_copies_each': dup}, open(f'{OUT}/mining{tag}.json', 'w'), indent=1)
            print(f'mined {len(hard)} of {len(ex_train)} exploration frames with a false positive')
        train = train + ex_train
    for name, part in (('train', train), ('val', val), ('test', test)):
        link_split(part, name, root, copies if name == 'train' else None)
    with open(f'{root}/data.yaml', 'w') as f:
        f.write(f"path: {os.path.abspath(root)}\ntrain: train/images\nval: val/images\n"
                f"test: test/images\nnames:\n  0: person\n")
    return root, (len(train), len(val), len(test))


def cmd_train(args):
    from ultralytics import YOLO
    root, n = make_fold(args.holdout, args.tag, args.explore, args.mine)
    print(f'fold {args.holdout}{args.tag}: train {n[0]}, val {n[1]}, test {n[2]} frames')
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
    rows = split(args.holdout, read_meta())[2]
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


def cmd_explore_fp(args):
    """False-positive rate on exploration-path frames of the validation worlds (seed 13)."""
    from ultralytics import YOLO
    rows = [r for r in read_explore() if r['seed'] == EXPLORE_VAL_SEED]
    model = YOLO(args.weights)
    fp_boxes = fp_frames = tp = fn = 0
    for i in range(0, len(rows), 16):
        chunk = rows[i:i + 16]
        for r, res in zip(chunk, model.predict(
                [f"{r['dir']}/images/{r['image']}.png" for r in chunk], imgsz=640, conf=args.conf,
                classes=[0], device=0, verbose=False)):
            preds = res.boxes.xyxy.tolist()
            bad = [b for b in preds if not any(iou(b, g[:4]) >= 0.5 for g in r['boxes'])]
            fp_boxes += len(bad)
            fp_frames += bool(bad)
            hit = sum(any(iou(b, g[:4]) >= 0.5 for b in preds) for g in r['boxes'])
            tp += hit
            fn += len(r['boxes']) - hit
    print(f"{args.weights}: {len(rows)} exploration frames, {fp_frames} with a false positive "
          f"({100 * fp_frames / max(len(rows), 1):.1f} %), {fp_boxes} false boxes, "
          f"recall {tp / max(tp + fn, 1):.2f} ({tp + fn} labelled boxes)")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    pf = sub.add_parser('explore-fp')
    pf.add_argument('--weights', required=True)
    pf.add_argument('--conf', type=float, default=0.30)
    for name in ('train', 'eval'):
        p = sub.add_parser(name)
        p.add_argument('--holdout', required=True, choices=LAYOUTS + ('all',))
        p.add_argument('--imgsz', type=int, default=640)
        p.add_argument('--tag', default='', help='suffix of the run name (e.g. _r2 for round 2)')
        if name == 'train':
            p.add_argument('--epochs', type=int, default=40)
            p.add_argument('--explore', action='store_true',
                           help='add the frames logged along exploration paths')
            p.add_argument('--mine', default=None, metavar='WEIGHTS',
                           help='with --explore: weights used to find false-positive frames, which'
                                ' are then repeated 2 more times in the training set')
        else:
            p.add_argument('--conf', type=float, default=0.30,
                           help='detection threshold (the localizer uses 0.30)')
    args = ap.parse_args()
    return {'train': cmd_train, 'eval': cmd_eval, 'explore-fp': cmd_explore_fp}[args.cmd](args)


if __name__ == '__main__':
    sys.exit(main())
