#!/bin/bash
# Pipeline with the detector retrained on exploration frames (all_x), committed worlds, 2 seeds per
# world: first without the hit-rate rule (isolates the retraining), then with min_hit_rate 0.25.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/phase5x
for rate in 0.0 0.25; do
  for seed in 1 2; do
    for w in collapsed rooms open_plan; do
      bash ~/cleanup.sh >/dev/null 2>&1
      echo "=== all_x rate=$rate $w seed $seed $(date +%T)"
      ros2 run rescue_eval explore_eval --world $w --seed $seed --detector yolo \
          --model results/detector/runs/all_x/weights/best.pt --min-hit-rate $rate \
          --method x_rate$rate --output-dir $OUT 2>&1 | grep -E "pins:|coverage [0-9.]+ after|status"
    done
  done
done
echo ALL_DONE
