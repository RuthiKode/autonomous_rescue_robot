#!/bin/bash
# 1) rerun block seed 3 (the first run lost its simulator: gz exit -11 before the mission left STARTING)
# 2) one run of the full launch file (headless, own selector, fine-tuned detector, verify on); sample /mission/state
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ~/cleanup.sh >/dev/null 2>&1
echo "=== rerun block collapsed seed 3 $(date +%T)"
ros2 run rescue_eval explore_eval --world collapsed --seed 3 --detector none --explorer own \
    --no-plateau --time-limit 600 --inject block --inject-at 90 --method fail_block \
    --output-dir results/failure_tests 2>&1 | grep -E "coverage [0-9.]+ after|injected|injection|status"

bash ~/cleanup.sh >/dev/null 2>&1
echo "=== full launch $(date +%T)"
LOG=results/full_launch_test.log
ros2 launch rescue_bringup full_mission.launch.py world:=collapsed seed:=1 headless:=true \
    detector:=yolo model:=$HOME/autonomous_rescue_robot/results/detector/runs/all_x/weights/best.pt \
    explorer:=own verify:=true > $LOG 2>&1 &
LP=$!
for t in 60 120 180 240 300; do
  sleep 60
  echo "--- t=$t s"
  timeout 20 ros2 topic echo --once /mission/state 2>/dev/null | grep -E "state|reason|goals_sent|recoveries|verifications"
  timeout 20 ros2 topic echo --once /explore/explored_area_m2 2>/dev/null | grep data
done
kill -INT $LP; sleep 8
bash ~/cleanup.sh >/dev/null 2>&1
grep -a -E "mission state" $LOG | cut -c1-200 | tail -15
echo WEEK8_DONE
