#!/usr/bin/env python3
"""Sheet of held-out frames where the fine-tuned detector fires without a matching label.

  ~/venvs/yolo/bin/python scripts/fp_sheet.py collapsed results/detector/fp_collapsed.png
Green = labelled survivor, red = detection with no label (false positive).
"""
import csv
import json
import sys

from PIL import Image, ImageDraw
from ultralytics import YOLO

sys.path.insert(0, 'scripts')
from train_detector import DATA, OUT, iou  # noqa: E402


def main():
    layout, out = sys.argv[1], sys.argv[2]
    model = YOLO(f'{OUT}/runs/{layout}/weights/best.pt')
    rows = [r for r in csv.DictReader(open(f'{DATA}/meta.csv')) if r['layout'] == layout]
    tiles = []
    for r in rows:
        gts = json.loads(r['boxes'])
        res = model.predict(f"{DATA}/images/{r['image']}.png", conf=0.30, classes=[0], device=0,
                            verbose=False)[0]
        fps = [b for b in res.boxes.xyxy.tolist() if not any(iou(b, g[:4]) >= 0.5 for g in gts)]
        if fps:
            im = Image.open(f"{DATA}/images/{r['image']}.png").convert('RGB')
            d = ImageDraw.Draw(im)
            for g in gts:
                d.rectangle(g[:4], outline=(0, 255, 0), width=3)
            for b in fps:
                d.rectangle(b, outline=(255, 0, 0), width=3)
            tiles.append(im.resize((320, 240)))
        if len(tiles) >= 16:
            break
    sheet = Image.new('RGB', (320 * 4, 240 * ((len(tiles) + 3) // 4)))
    for i, t in enumerate(tiles):
        sheet.paste(t, ((i % 4) * 320, (i // 4) * 240))
    sheet.save(out)
    print('wrote', out, len(tiles), 'tiles')


if __name__ == '__main__':
    main()
