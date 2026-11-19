#!/bin/bash
# Staged Nav2 run with explicit goals: sim + SLAM, warm-up replay, THEN Nav2, then the goals.
#
#   scripts/nav_staged.sh <world> <output.csv> <nav_goals arguments...>
#
# Example:
#   scripts/nav_staged.sh collapsed results/gaps.csv --goal=5.0,-1.7 --goal=-2.0,-1.0
#
# Nav2 starts after the replay because its collision monitor and velocity smoother also
# publish on /cmd_vel and would disturb the recorded drive. Output goes to stdout;
# the recovery lines ("recovery fired: ...") are kept.
WORLD=${1:?usage: nav_staged.sh <world> <output.csv> [nav_goals args]}
CSV=${2:?}
shift 2
WS=$(cd "$(dirname "$0")/.." && pwd)
source "$WS/install/setup.bash"
OUT=$(mktemp -d /tmp/nav_staged_XXXX)

setsid ros2 launch rescue_bringup mapping.launch.py world:=$WORLD headless:=true \
  > $OUT/mapping.log 2>&1 &
P1=$!
cleanup() {
  kill -INT -- -$P2 -$P1 2>/dev/null
  sleep 10
  kill -KILL -- -$P2 -$P1 2>/dev/null
}
trap cleanup EXIT
for _ in $(seq 60); do
  timeout 5 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once > /dev/null 2>&1 && break
  sleep 1
done
sleep 15
ros2 run rescue_eval cmd_vel_replay --input "$WS/src/rescue_eval/drives/$WORLD.csv" 2>&1 | tail -1
sleep 8
setsid ros2 launch rescue_navigation navigation.launch.py > $OUT/nav.log 2>&1 &
P2=$!
for _ in $(seq 40); do
  ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active" && break
  sleep 3
done
ros2 run rescue_eval nav_goals --world $WORLD --output "$CSV" "$@" 2>&1
echo "--- Nav2 warnings and errors"
grep -hE "ERROR|WARN" $OUT/nav.log | grep -vE "gz_frame_id|Failed to meet|Dock database|Control loop missed|No progress checker|No goal checker|verbose" \
  | sed 's/\[[0-9.]*\]//' | cut -c1-170 | sort | uniq -c | sort -rn | head -10
