#!/usr/bin/env python3
"""Contact sheet of dataset frames with their labelled boxes drawn on (visual label check).

  python3 scripts/label_sheet.py results/dataset results/dataset/sheet.png [--n 12] [--empty]
"""
import argparse
import csv
import json
import random

from PIL import Image, ImageDraw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset')
    ap.add_argument('out')
    ap.add_argument('--n', type=int, default=12)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    rows = [r for r in csv.DictReader(open(f'{args.dataset}/meta.csv')) if r['n_boxes'] != '0']
    random.Random(args.seed).shuffle(rows)
    rows = rows[:args.n]
    w, h, cols = 320, 240, 4
    sheet = Image.new('RGB', (w * cols, h * ((len(rows) + cols - 1) // cols)))
    for i, r in enumerate(rows):
        im = Image.open(f"{args.dataset}/images/{r['image']}.png").convert('RGB')
        d = ImageDraw.Draw(im)
        for b in json.loads(r['boxes']):
            d.rectangle(b[:4], outline=(255, 0, 0), width=3)
            d.text((b[0] + 4, b[1] + 4), f'{b[5]} occ={b[6]}', fill=(255, 255, 0))
        d.text((5, 5), f"sun el {r['sun_elevation']} str {r['sun_strength']}", fill=(255, 255, 255))
        sheet.paste(im.resize((w, h)), ((i % cols) * w, (i // cols) * h))
    sheet.save(args.out)
    print('wrote', args.out, 'with', len(rows), 'frames')


if __name__ == '__main__':
    main()
