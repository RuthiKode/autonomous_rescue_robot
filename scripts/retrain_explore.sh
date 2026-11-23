#!/bin/bash
# Retrain the deployment detector with exploration-path frames and mined hard negatives,
# compare false-positive rates on exploration frames of an unseen world (seed 13).
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
PY=~/venvs/yolo/bin/python
echo "--- before (round 3 detector)"
$PY scripts/train_detector.py explore-fp --weights results/detector/runs/all/weights/best.pt 2>&1 | tail -1
$PY scripts/train_detector.py train --holdout all --tag _x --explore \
    --mine results/detector/runs/all/weights/best.pt 2>&1 | grep -E "^mined|^fold|Error"
$PY scripts/train_detector.py eval --holdout all --tag _x 2>&1 | grep -E "^all |Error"
echo "--- after (with exploration frames)"
$PY scripts/train_detector.py explore-fp --weights results/detector/runs/all_x/weights/best.pt 2>&1 | tail -1
echo TRAIN_DONE
