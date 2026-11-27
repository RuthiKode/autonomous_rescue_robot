#!/bin/bash
# Phase 13 live tests: lost-mode timeout.
#   forced: lost_timeout_s 20 (scripts/lost_timeout_20s.yaml) on teleport seeds 1-2, which go to lost mode;
#           the saved map must be given up, the mission must keep exploring and end with the reason.
#   default (360 s, mission.yaml): teleport seed 1 and one normal mission, as a regression check.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=${OUT:-results/phase13}
run() {  # world seed inject method [extra explore_eval args]
  bash scripts/hard_reset.sh --keep-sweep >/dev/null 2>&1
  echo "=== $4 $3 $1 seed $2 $(date +%T)"
  ros2 run rescue_eval explore_eval --world $1 --seed $2 --detector none --explorer own \
      --no-plateau --time-limit 600 --inject $3 --inject-at 90 --method $4 \
      --output-dir $OUT "${@:5}" 2>&1 | grep -E "coverage [0-9.]+ after|injected|injection|status"
  d=$(ls -dt $OUT/runs/$1_$4_seed$2_* 2>/dev/null | head -1)
  [ -n "$d" ] && grep -hoE "kidnapped: scan-to-map score [0-9.]+|relocalized.*|lost: (no confident|no confirmed|match).*|mission state (DONE|FAILED).*" "$d/launch.log" | cut -c1-170
}
for s in ${SEEDS:-1 2}; do run collapsed $s teleport timeout20_teleport --explore-params scripts/lost_timeout_20s.yaml; done
[ -z "$FORCED_ONLY" ] && run collapsed 1 teleport timeout360_teleport
[ -z "$FORCED_ONLY" ] && run collapsed 1 none timeout360_none
bash scripts/hard_reset.sh --keep-sweep >/dev/null 2>&1
echo TESTS_DONE
