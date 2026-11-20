#!/bin/bash
# Week-6 runs: confidence sweep on the held-out frames, then the full pipeline with the
# fine-tuned detector that never saw that world's layout (2 seeds per world).
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
PY=~/venvs/yolo/bin/python
for conf in 0.50 0.70; do
  for h in collapsed rooms open_plan; do
    $PY scripts/train_detector.py eval --holdout $h --conf $conf 2>&1 | grep -E "^(collapsed|rooms|open_plan) finetuned"
    cp results/detector/$h.json results/detector/${h}_conf${conf}.json
  done
done
# restore the conf 0.30 files
for h in collapsed rooms open_plan; do
  $PY scripts/train_detector.py eval --holdout $h --conf 0.30 2>&1 | grep -E "^(collapsed|rooms|open_plan) "
done
OUT=results/phase5
for seed in 1 2; do
  for w in collapsed rooms open_plan; do
    bash ~/cleanup.sh >/dev/null 2>&1
    echo "=== finetuned $w seed $seed $(date +%T)"
    ros2 run rescue_eval explore_eval --world $w --seed $seed --detector yolo \
        --model results/detector/runs/$w/weights/best.pt --method finetuned_yolo \
        --output-dir $OUT 2>&1 | grep -E "pins:|coverage|status|new candidate|confirmed" | tail -12
  done
done
echo ALL_DONE
