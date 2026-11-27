#!/bin/bash
# Live mission with the Gazebo 3D window open (needs a display; on Windows 11 WSLg provides one).
# Usage: scripts/gui_demo.sh [collapsed|rooms|open_plan]
# Closing the Gazebo window ends the simulation, and with it the launch. Stop early with
# scripts/hard_reset.sh. The launch output goes to results/gui_demo.log.
# Note (2026-11-25): the first run ended after 62 s with "gz process has finished cleanly" and exit
# code 4 from wsl.exe; the launch log shows no crash, consistent with the window being closed and
# WSL shutting down, not with a mission failure.
cd ~/autonomous_rescue_robot
bash scripts/hard_reset.sh >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=78
MODEL=results/detector/runs/all_x/weights/best.pt
[ -f "$MODEL" ] || { echo "detector weights not found: $MODEL (train with scripts/train_detector.py)"; exit 1; }
mkdir -p results
exec ros2 launch rescue_bringup full_mission.launch.py world:="${1:-collapsed}" headless:=false \
  explorer:=own verify:=true detector:=yolo model:="$MODEL" > results/gui_demo.log 2>&1
