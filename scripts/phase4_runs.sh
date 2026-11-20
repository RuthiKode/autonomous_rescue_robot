#!/bin/bash
# Week-5 runs, one after another (one simulation at a time). Usage: scripts/phase4_runs.sh
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=results/phase4
run() {  # world seed detector method [extra args]
  local w=$1 s=$2 d=$3 m=$4; shift 4
  bash ~/cleanup.sh >/dev/null 2>&1
  echo "=== $m $w seed $s ($d) $(date +%T)"
  ros2 run rescue_eval explore_eval --world "$w" --seed "$s" --detector "$d" --method "$m" \
      --output-dir "$OUT" "$@" 2>&1 | tail -25
}
run collapsed 1 yolo   yolo_pipeline
run collapsed 1 oracle oracle_pipeline
run rooms     1 oracle oracle_pipeline
run open_plan 1 oracle oracle_pipeline
for v in A_upstream_defaults B_project C_gain_heavy; do
  for s in 1 2; do
    run collapsed $s none tune_$v --explore-params results/phase4_tuning/$v.yaml
  done
done
echo ALL_DONE
