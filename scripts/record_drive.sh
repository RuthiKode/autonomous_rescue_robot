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
# Do not start driving until the recorder is really subscribed to /cmd_vel,
# or the first commands of the drive are lost.
for _ in $(seq 30); do
  ros2 topic info /cmd_vel -v 2>/dev/null | grep -q "Node name: cmd_vel_recorder" && break
  sleep 1
done
if ! ros2 topic info /cmd_vel -v 2>/dev/null | grep -q "Node name: cmd_vel_recorder"; then
  echo "FAIL: the recorder never subscribed to /cmd_vel"; exit 1
fi
ros2 run rescue_eval auto_drive --world $WORLD --min-subscribers 2
STATUS=$?
echo "true end pose: $(gz model -m rescue_robot -p 2>/dev/null | tail -2 | tr '\n' ' ')"
pkill -TERM -f "rescue_eval/cmd_vel_record"; wait $REC
exit $STATUS
