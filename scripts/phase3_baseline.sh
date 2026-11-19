#!/bin/bash
# Phase 3 navigation baseline: per world, replay the warm-up drive so SLAM has a map, start
# Nav2, then send N random reachable goals and score them.
#
#   scripts/phase3_baseline.sh [goals_per_run] [output_dir] [seeds...]
#
# One row per run in <output_dir>/runs.csv, one row per goal in runs/<run_id>/goals.csv.
GOALS=${1:-20}
WS=$(cd "$(dirname "$0")/.." && pwd)
OUT=${2:-$WS/results/phase3}
shift; shift 2>/dev/null
SEEDS=${*:-1}
source "$WS/install/setup.bash"
for SEED in $SEEDS; do
  for WORLD in rooms collapsed open_plan; do
    ros2 run rescue_eval run_eval --world $WORLD --seed $SEED --slam \
      --drive "$WS/src/rescue_eval/drives/$WORLD.csv" --nav-goals $GOALS --nav-seed $SEED \
      --method nav_baseline --output-dir "$OUT"
  done
done
ros2 run rescue_eval summarize --csv "$OUT/runs.csv" --method nav_baseline
