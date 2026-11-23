#!/bin/bash
# Remaining runs of phase5y_runs.sh after the WSL session was interrupted (and the one run that
# hit a startup timeout): same settings, same output directory.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/phase5y
MODEL=results/detector/runs/all_x/weights/best.pt
for seed in 21 22; do
  [ -d results/unseen_worlds/s$seed ] || ros2 run rescue_simulation generate_worlds \
      --output-dir results/unseen_worlds/s$seed --seed $seed >/dev/null
done
run() {  # world gazebo_seed method [worlds_dir]
  bash ~/cleanup.sh >/dev/null 2>&1
  echo "=== $3 $1 seed $2 $(date +%T)"
  extra=""
  [ -n "$4" ] && extra="--worlds-dir $4"
  ros2 run rescue_eval explore_eval --world $1 --seed $2 --detector yolo --model $MODEL \
      --method $3 --output-dir $OUT $extra 2>&1 | grep -E "pins:|coverage [0-9.]+ after|status"
}
run collapsed 1 rule_committed
run open_plan 1 rule_unseen21 results/unseen_worlds/s21
for w in collapsed rooms open_plan; do run $w 1 rule_unseen22 results/unseen_worlds/s22; done
echo ALL_DONE
