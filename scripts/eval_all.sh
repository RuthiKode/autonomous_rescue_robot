#!/bin/bash
# Score pretrained vs fine-tuned detectors on each held-out layout (after train_all.sh).
cd ~/autonomous_rescue_robot
PY=~/venvs/yolo/bin/python
for h in collapsed rooms open_plan; do
  $PY scripts/train_detector.py eval --holdout $h 2>&1 | grep -E "^(collapsed|rooms|open_plan) |Error"
done
echo EVAL_DONE
