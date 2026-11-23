#!/bin/bash
# Week 7 gate: coverage-versus-time for explore_lite and the own selector (frontier selector + mission
# tree) on the same worlds and Gazebo seeds, no detector. 3 worlds x 3 seeds x 2 selectors.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/phase6
for seed in 1 2 3; do
  for w in collapsed rooms open_plan; do
    for ex in lite own; do
      bash ~/cleanup.sh >/dev/null 2>&1
      echo "=== $ex $w seed $seed $(date +%T)"
      ros2 run rescue_eval explore_eval --world $w --seed $seed --detector none --explorer $ex \
          --method explore_$ex --output-dir $OUT 2>&1 | grep -E "coverage [0-9.]+ after|status"
    done
  done
done
echo ALL_DONE
