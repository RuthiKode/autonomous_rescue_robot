#!/bin/bash
# Final pin rule (confirm_hits 20, confirm_confidence 0.6, merge_radius 1.5) with the detector
# trained on exploration frames (all_x):
#   1. committed worlds, 2 Gazebo seeds per world (the rule was chosen on these worlds' earlier runs)
#   2. generated worlds with seeds 21 and 22, never used for training, mining or choosing the rule
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/phase5y
MODEL=results/detector/runs/all_x/weights/best.pt
for seed in 21 22; do
  ros2 run rescue_simulation generate_worlds --output-dir results/unseen_worlds/s$seed --seed $seed >/dev/null
done
run() {  # world gazebo_seed method [worlds_dir]
  bash ~/cleanup.sh >/dev/null 2>&1
  echo "=== $3 $1 seed $2 $(date +%T)"
  extra=""
  [ -n "$4" ] && extra="--worlds-dir $4"
  ros2 run rescue_eval explore_eval --world $1 --seed $2 --detector yolo --model $MODEL \
      --method $3 --output-dir $OUT $extra 2>&1 | grep -E "pins:|coverage [0-9.]+ after|status"
}
for seed in 1 2; do
  for w in collapsed rooms open_plan; do run $w $seed rule_committed; done
done
for ws in 21 22; do
  for w in collapsed rooms open_plan; do run $w 1 rule_unseen$ws results/unseen_worlds/s$ws; done
done
echo ALL_DONE
