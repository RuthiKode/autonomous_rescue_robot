#!/bin/bash
# One run of the own selector + mission tree, to see that the pieces work together.
# Usage: scripts/smoke_own.sh [world] [time_limit_s]
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ~/cleanup.sh >/dev/null 2>&1
ros2 run rescue_eval explore_eval --world ${1:-open_plan} --seed 1 --detector none --explorer own \
    --method own_smoke --time-limit ${2:-240} --output-dir results/phase6_smoke 2>&1 | tail -15
echo "--- selector and mission lines:"
grep -a -E "mission_tree|frontier_selector" results/phase6_smoke/runs/*/launch.log \
    | grep -a -E "mission state|goal \(|Traceback|Error|ValueError" | cut -c1-200 | tail -40
