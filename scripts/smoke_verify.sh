#!/bin/bash
# One run with verification on (fine-tuned YOLO, own selector + mission tree), to see the whole chain:
# pins found, driven to, verified or rejected. Usage: scripts/smoke_verify.sh [world] [seed]
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ~/cleanup.sh >/dev/null 2>&1
rm -rf results/smoke_verify
ros2 run rescue_eval explore_eval --world ${1:-collapsed} --seed ${2:-1} --explorer own --verify \
    --detector yolo --model results/detector/runs/all_x/weights/best.pt --no-plateau \
    --method smoke_verify --output-dir results/smoke_verify 2>&1 | grep -v "^\[ros2run\]" | tail -20
echo "--- mission and localizer lines:"
grep -a -E "mission state|track [0-9]+ (verified|rejected|confirmed)|localization jump|Traceback|Error" \
    results/smoke_verify/runs/*/launch.log | cut -c1-210 | tail -50
