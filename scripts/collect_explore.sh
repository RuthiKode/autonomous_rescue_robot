#!/bin/bash
# Labelled frames along real exploration paths, in generated worlds the evaluation never uses.
# Usage: [SEEDS="11 12 13"] scripts/collect_explore.sh      (about 7 min per world)
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/dataset_explore
for seed in ${SEEDS:-11 12 13}; do
  ros2 run rescue_simulation generate_worlds --output-dir $OUT/worlds/s$seed --seed $seed >/dev/null
  for layout in collapsed rooms open_plan; do
    bash ~/cleanup.sh >/dev/null 2>&1
    echo "=== $layout seed $seed $(date +%T)"
    ros2 run rescue_eval explore_eval --world $layout --seed $seed --detector none \
        --worlds-dir $OUT/worlds/s$seed --collect-frames $OUT \
        --method collect_s$seed --output-dir results/explore_collect 2>&1 \
        | grep -E "coverage [0-9.]+ after|status|Error|error" | tail -4
    echo "frames so far: $(ls $OUT/images 2>/dev/null | wc -l)"
  done
done
echo COLLECT_DONE
