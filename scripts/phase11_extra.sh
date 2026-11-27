#!/bin/bash
# Phase 11 extra checks: a normal rooms mission (false kidnaps now cost a SLAM reset), then the
# README full-mission command with lost mode on.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash; source install/setup.bash
bash scripts/hard_reset.sh --keep-sweep >/dev/null 2>&1
echo "=== none rooms seed 1 $(date +%T)"
ros2 run rescue_eval explore_eval --world rooms --seed 1 --detector none --explorer own \
    --no-plateau --time-limit 600 --inject none --method lost_none --output-dir results/phase11 2>&1 \
    | grep -E "coverage [0-9.]+ after|status"
d=$(ls -dt results/phase11/runs/rooms_lost_none_seed1_* | head -1)
echo "pause/kidnap/lost lines: $(grep -cE 'new measurements paused|kidnapped:|lost:' $d/launch.log)"
echo "=== full mission check $(date +%T)"
bash scripts/full_mission_check.sh
echo EXTRA_DONE
