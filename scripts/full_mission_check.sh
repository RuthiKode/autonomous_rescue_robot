#!/bin/bash
# One headless run of the README's full-mission command; reports the mission end and any traceback.
cd ~/autonomous_rescue_robot
bash scripts/hard_reset.sh >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=75
L=results/full_mission_check.log
timeout 1200 ros2 launch rescue_bringup full_mission.launch.py world:=collapsed explorer:=own verify:=true \
    detector:=yolo model:=results/detector/runs/all_x/weights/best.pt > $L 2>&1 &
for i in $(seq 1 240); do
  grep -qE "mission state (DONE|FAILED)" $L && break
  sleep 5
done
grep -hE "mission state (DONE|FAILED)|kidnapped|relocaliz|new measurements" $L | cut -c1-200
echo "tracebacks: $(grep -c Traceback $L)"
echo "process deaths: $(grep -c 'process has died' $L)"
bash scripts/hard_reset.sh >/dev/null 2>&1
