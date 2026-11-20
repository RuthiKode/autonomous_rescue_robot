#!/bin/bash
# Collect the whole week-6 dataset: 3 layouts x 6 world seeds (42 = the committed worlds).
# Usage: [SEEDS="6 7 8"] scripts/collect_all.sh [frames_per_world] [out_dir]
# Seeds 6 and up are extra training worlds (round 2); the test set stays seeds 42 and 1-5.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
FRAMES=${1:-120}
OUT=${2:-results/dataset}
for seed in ${SEEDS:-42 1 2 3 4 5}; do
  for layout in rooms collapsed open_plan; do
    bash ~/cleanup.sh >/dev/null 2>&1
    python3 scripts/collect_dataset.py --layout $layout --seed $seed --frames "$FRAMES" --out "$OUT" 2>&1 | tail -2
  done
done
echo COLLECT_DONE
