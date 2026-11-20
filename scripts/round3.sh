#!/bin/bash
# The deployment detector: trained on all layouts (world seed 42 excluded), tested on seed 42,
# then the pipeline in the committed worlds with it (2 Gazebo seeds per world).
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PY=~/venvs/yolo/bin/python
$PY scripts/train_detector.py train --holdout all 2>&1 | grep -E "^fold|Error"
$PY scripts/train_detector.py eval --holdout all 2>&1 | grep -E "^all |Error"
OUT=results/phase5all
for seed in 1 2; do
  for w in collapsed rooms open_plan; do
    bash ~/cleanup.sh >/dev/null 2>&1
    echo "=== finetuned_all $w seed $seed $(date +%T)"
    ros2 run rescue_eval explore_eval --world $w --seed $seed --detector yolo \
        --model results/detector/runs/all/weights/best.pt --method finetuned_yolo_all \
        --output-dir $OUT 2>&1 | grep -E "pins:|coverage [0-9.]+ after|status"
  done
done
echo ALL_DONE
