#!/bin/bash
# Runs scripts/reseed_smoke.py against a headless collapsed-world mapping launch.
cd ~/autonomous_rescue_robot
bash scripts/hard_reset.sh >/dev/null 2>&1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export ROS_DOMAIN_ID=79
ros2 launch rescue_bringup mapping.launch.py world:=collapsed headless:=true > results/reseed_smoke_launch.log 2>&1 &
sleep 75
# hold the robot still during the test (the wake-up drive has run by now)
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
HOLD=$!
sleep 5
timeout 120 python3 scripts/reseed_smoke.py
echo "exit $?"
kill $HOLD
grep -iE "deserial|error|fail" results/reseed_smoke_launch.log | tail -n 8 | cut -c1-200
bash scripts/hard_reset.sh >/dev/null 2>&1
