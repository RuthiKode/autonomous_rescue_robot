#!/bin/bash
# Round 2: three extra training worlds per layout (seeds 6-8), retrain the three folds as *_r2,
# score them, then run the pipeline with the round-2 detectors (2 seeds per world).
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PY=~/venvs/yolo/bin/python
SEEDS="6 7 8" bash scripts/collect_all.sh 120 results/dataset
for h in collapsed rooms open_plan; do
  $PY scripts/train_detector.py train --holdout $h --tag _r2 2>&1 | grep -E "^fold|Error"
  $PY scripts/train_detector.py eval --holdout $h --tag _r2 2>&1 | grep -E "^(collapsed|rooms|open_plan) |Error"
done
OUT=results/phase5r2
for seed in 1 2; do
  for w in collapsed rooms open_plan; do
    bash ~/cleanup.sh >/dev/null 2>&1
    echo "=== finetuned_r2 $w seed $seed $(date +%T)"
    ros2 run rescue_eval explore_eval --world $w --seed $seed --detector yolo \
        --model results/detector/runs/${w}_r2/weights/best.pt --method finetuned_yolo_r2 \
        --output-dir $OUT 2>&1 | grep -E "pins:|coverage [0-9.]+ after|status"
  done
done
echo ALL_DONE
