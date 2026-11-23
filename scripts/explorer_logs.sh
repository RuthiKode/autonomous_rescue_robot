#!/bin/bash
# Last explorer lines (explore_lite messages or mission-tree states) of each week-7 run.
cd ~/autonomous_rescue_robot/results/phase6/runs
for d in *; do
  echo "== $d"
  grep -a -E "explore_node\]|mission state|mission_tree.*(goals|goal \()" "$d/launch.log" | tail -2 | cut -c1-190
done
