#!/bin/bash
# Phase 10 live tests with relocalization enabled (src/rescue_mission/config/mission.yaml):
#   teleport seeds 1-5 (the week-8 test that failed 0/5), block seeds 1-3 (regression),
#   one normal mission in collapsed and one in rooms (false kidnap triggers?).
# Same harness and pass rule as scripts/failure_tests.sh; hard reset between runs.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=${OUT:-results/phase10}
run() {  # world seed inject
  bash scripts/hard_reset.sh >/dev/null 2>&1
  echo "=== $3 $1 seed $2 $(date +%T)"
  ros2 run rescue_eval explore_eval --world $1 --seed $2 --detector none --explorer own \
      --no-plateau --time-limit 600 --inject $3 --inject-at 90 --method reloc_$3 \
      --output-dir $OUT 2>&1 | grep -E "coverage [0-9.]+ after|injected|injection|status"
  d=$(ls -dt $OUT/runs/$1_reloc_$3_seed$2_* 2>/dev/null | head -1)
  [ -n "$d" ] && grep -hoE "kidnapped: scan-to-map score [0-9.]+|relocalization attempt [0-9]: .*|relocalized: .*|slam_toolbox new measurements (paused|resumed)" "$d/launch.log" | sort | uniq -c | head -12
}
for s in 1 2 3 4 5; do run collapsed $s teleport; done
for s in 1 2 3; do run collapsed $s block; done
run collapsed 1 none
run rooms 1 none
bash scripts/hard_reset.sh >/dev/null 2>&1
echo TESTS_DONE
