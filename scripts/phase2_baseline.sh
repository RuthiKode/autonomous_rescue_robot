#!/bin/bash
# Phase 2 mapping baseline: N SLAM runs per world, each replaying that world's
# recorded drive with a different Gazebo seed.
#
#   scripts/phase2_baseline.sh [runs_per_world] [output_dir]
#
# Each run leaves a rosbag (bags/), a saved map and two TUM trajectories (runs/),
# and one row in <output_dir>/runs.csv.
RUNS=${1:-3}
WS=$(cd "$(dirname "$0")/.." && pwd)
OUT=${2:-$WS/results/phase2}
source "$WS/install/setup.bash"
for WORLD in rooms collapsed open_plan; do
  for SEED in $(seq 1 "$RUNS"); do
    ros2 run rescue_eval run_eval --world $WORLD --seed $SEED --slam \
      --drive "$WS/src/rescue_eval/drives/$WORLD.csv" --method slam_replay --output-dir "$OUT"
  done
done
ros2 run rescue_eval summarize --csv "$OUT/runs.csv" --method slam_replay
