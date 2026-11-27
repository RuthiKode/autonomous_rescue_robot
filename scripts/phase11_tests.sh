#!/bin/bash
# Phase 11 live tests with lost mode enabled (src/rescue_mission/config/mission.yaml):
#   teleport seeds 1-5, block seeds 1-2 (regression), one normal mission in collapsed.
#   TELEPORT_ONLY=1 runs the teleports only.
# Same harness and pass rule as scripts/failure_tests.sh; hard reset between runs.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=${OUT:-results/phase11}
run() {  # world seed inject
  bash scripts/hard_reset.sh --keep-sweep >/dev/null 2>&1
  echo "=== $3 $1 seed $2 $(date +%T)"
  ros2 run rescue_eval explore_eval --world $1 --seed $2 --detector none --explorer own \
      --no-plateau --time-limit 600 --inject $3 --inject-at 90 --method lost_$3 \
      --output-dir $OUT 2>&1 | grep -E "coverage [0-9.]+ after|injected|injection|status"
  d=$(ls -dt $OUT/runs/$1_lost_$3_seed$2_* 2>/dev/null | head -1)
  [ -n "$d" ] && grep -hoE "kidnapped: scan-to-map score [0-9.]+|relocalization attempt [0-9]: .*|relocalized.*|lost: .*|pre-kidnap pose graph saved" "$d/launch.log" | cut -c1-150 | sort | uniq -c | sort -rn | head -14
}
for s in 1 2 3 4 5; do run collapsed $s teleport; done
[ -z "$TELEPORT_ONLY" ] && for s in 1 2; do run collapsed $s block; done
[ -z "$TELEPORT_ONLY" ] && run collapsed 1 none
bash scripts/hard_reset.sh --keep-sweep >/dev/null 2>&1
echo TESTS_DONE
