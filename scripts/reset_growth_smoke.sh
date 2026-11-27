#!/bin/bash
# Runs scripts/reset_growth_smoke.py against a headless collapsed-world mapping launch.
cd ~/autonomous_rescue_robot
bash scripts/hard_reset.sh >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=79
ros2 launch rescue_bringup mapping.launch.py world:=collapsed headless:=true > results/reset_growth_smoke_launch.log 2>&1 &
sleep 60
timeout 200 python3 scripts/reset_growth_smoke.py
echo "exit $?"
grep -iE "reset|pause|error" results/reset_growth_smoke_launch.log | grep -v "Interrupted" | tail -n 8 | cut -c1-200
bash scripts/hard_reset.sh >/dev/null 2>&1
