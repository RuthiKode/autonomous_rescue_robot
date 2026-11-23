#!/bin/bash
# Completion-time runs of the own selector: min_gain_m2 floor on, no coverage-plateau stop, so the
# mission tree ends the run itself (DONE / FAILED) or the 900 s limit does. Same worlds and seeds as
# phase6_runs.sh.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/phase6c
for seed in 1 2 3; do
  for w in collapsed rooms open_plan; do
    bash ~/cleanup.sh >/dev/null 2>&1
    echo "=== own+floor $w seed $seed $(date +%T)"
    ros2 run rescue_eval explore_eval --world $w --seed $seed --detector none --explorer own \
        --no-plateau --method explore_own_floor --output-dir $OUT 2>&1 \
        | grep -E "coverage [0-9.]+ after|status"
  done
done
echo ALL_DONE
