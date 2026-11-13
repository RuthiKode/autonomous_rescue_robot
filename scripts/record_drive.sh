#!/bin/bash
# Record a /cmd_vel drive file for one world without a person at the keyboard.
#
#   scripts/record_drive.sh <world> [output.csv]
#
# Starts the simulation headless, starts cmd_vel_record, and lets auto_drive
# (a ground-truth tour, see rescue_eval/config/auto_drive.yaml) do the driving.
# To record your own drive instead, run the three commands in docs/PHASE2.md.
WORLD=${1:?usage: record_drive.sh <world> [output.csv]}
WS=$(cd "$(dirname "$0")/.." && pwd)
OUT=${2:-$WS/src/rescue_eval/drives/$WORLD.csv}
source "$WS/install/setup.bash"
LOG=$(mktemp /tmp/record_drive_${WORLD}_XXXX.log)

setsid ros2 launch rescue_bringup sim.launch.py world:=$WORLD headless:=true > "$LOG" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -$LAUNCH 2>/dev/null
  for _ in $(seq 20); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done
  kill -KILL -- -$LAUNCH 2>/dev/null
}
trap cleanup EXIT
for _ in $(seq 60); do
  timeout 5 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once > /dev/null 2>&1 && break
  sleep 1
done
sleep 5

ros2 run rescue_eval cmd_vel_record --output "$OUT" \
  --comment "world=$WORLD driver=auto_drive (ground-truth tour, not a human)" &
REC=$!
sleep 2
ros2 run rescue_eval auto_drive --world $WORLD
STATUS=$?
echo "true end pose: $(gz model -m rescue_robot -p 2>/dev/null | tail -2 | tr '\n' ' ')"
kill -INT $REC; wait $REC
exit $STATUS
