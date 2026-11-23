#!/bin/bash
# Week 8 gate: scripted failures, 5 consecutive seeded runs each.
#   block     a wall appears 1.5 m ahead of the robot 90 s into exploring (a blocked corridor)
#   teleport  the robot is moved to another reachable spot 5+ m away 90 s into exploring
# Own selector + mission tree, no detector (the failures are about navigation and SLAM).
# Usage: scripts/failure_tests.sh [world]    (default collapsed)
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
WORLD=${1:-collapsed}
OUT=${OUT:-results/failure_tests}
for kind in block teleport; do
  for seed in 1 2 3 4 5; do
    bash ~/cleanup.sh >/dev/null 2>&1
    echo "=== $kind $WORLD seed $seed $(date +%T)"
    ros2 run rescue_eval explore_eval --world $WORLD --seed $seed --detector none --explorer own \
        --no-plateau --time-limit 600 --inject $kind --inject-at 90 --method fail_$kind \
        --output-dir $OUT 2>&1 | grep -E "coverage [0-9.]+ after|injected|injection|status"
  done
done
echo TESTS_DONE
