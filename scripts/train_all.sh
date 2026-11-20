#!/bin/bash
# Train and score the three leave-one-layout-out detectors (needs the dataset from collect_all.sh).
cd ~/autonomous_rescue_robot
PY=~/venvs/yolo/bin/python
for h in collapsed rooms open_plan; do
  $PY scripts/train_detector.py train --holdout $h 2>&1 | tail -3
  $PY scripts/train_detector.py eval --holdout $h 2>&1 | tail -3
done
echo TRAIN_DONE
