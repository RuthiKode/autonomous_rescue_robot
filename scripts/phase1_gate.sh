#!/bin/bash
# Phase 1 gate checks for one world, run headless.
#
#   scripts/phase1_gate.sh <world> [--frames]
#
# Starts the simulation, then checks: bridged topics, sensor rates (sim time and
# wall clock), sensor noise, ground truth, keyboard teleop, and with --frames the
# TF tree. Everything printed is measured in this run.
WORLD=${1:?usage: phase1_gate.sh <rooms|collapsed|open_plan> [--frames]}
FRAMES=${2:-}
WS=$(cd "$(dirname "$0")/.." && pwd)
source "$WS/install/setup.bash"
set -u
LOG=$(mktemp /tmp/phase1_gate_${WORLD}_XXXX.log)

odom() { ros2 topic echo /odom_raw --once --field pose.pose.position 2>/dev/null | tr '\n' ' '; echo; }
true_pose() { gz model -m rescue_robot -p 2>/dev/null | grep -A1 -E 'XYZ|RPY' | tr '\n' ' '; echo; }

echo "##### world=$WORLD  log=$LOG"
setsid ros2 launch rescue_bringup sim.launch.py world:=$WORLD headless:=true > "$LOG" 2>&1 &
LAUNCH=$!
cleanup() {
  kill -INT -- -$LAUNCH 2>/dev/null
  for _ in $(seq 20); do kill -0 $LAUNCH 2>/dev/null || break; sleep 1; done
  kill -KILL -- -$LAUNCH 2>/dev/null
}
trap cleanup EXIT

echo "--- waiting for /clock"
READY=0
for _ in $(seq 60); do  # the topic does not exist until the bridge is up, so retry
  if timeout 5 ros2 topic echo /clock rosgraph_msgs/msg/Clock --once > /dev/null 2>&1; then
    READY=1; break
  fi
  sleep 1
done
if [ $READY -ne 1 ]; then
  echo "FAIL: /clock never arrived"; tail -30 "$LOG"; exit 1
fi
sleep 5

echo "--- renderer used by Gazebo sensors"
grep -m1 -iE 'd3d12|llvmpipe|GL_RENDERER|OpenGL' "$LOG" || echo "(not reported in the launch log)"

echo "--- ROS topics"
ros2 topic list | tr '\n' ' '; echo

echo "--- sensor_check (rates from sim-time stamps, noise on a stationary robot)"
ros2 run rescue_eval sensor_check --duration 10

echo "--- ros2 topic hz (wall clock)"
for t in /scan /imu /camera/image_raw; do
  echo "$t: $(timeout 10 ros2 topic hz $t --window 500 2>&1 | grep 'average rate' | tail -1)"
done

echo "--- /ground_truth/survivors"
ros2 topic echo /ground_truth/survivors --once --qos-durability transient_local 2>/dev/null \
  | grep -E 'frame_id|^- id|    x:|    y:|state' | grep -vE 'orientation' | tr '\n' ' '; echo

echo "--- teleop_twist_keyboard: forward (i) 2 s, stop (k), turn left (j) 2 s, stop (k)"
echo "odom before:  $(odom)"
( sleep 3; printf 'i'; sleep 2; printf 'k'; sleep 1; printf 'j'; sleep 2; printf 'k'; sleep 1; printf '\003' ) \
  | script -qfec "ros2 run teleop_twist_keyboard teleop_twist_keyboard" /dev/null > /dev/null 2>&1
sleep 1
echo "odom after:   $(odom)"
echo "true pose (Gazebo): $(true_pose)"

if [ "$FRAMES" = "--frames" ]; then
  echo "--- TF tree (tf2_tools view_frames)"
  (cd /tmp && ros2 run tf2_tools view_frames -o phase1_frames > /tmp/view_frames.out 2>&1)
  grep -E '\->|Broadcaster' /tmp/phase1_frames.gv | sed 's/\\n/ | /g'
  echo "--- publishers on /tf and /tf_static"
  ros2 topic info /tf -v | grep -E 'Node name|Publisher count'
  ros2 topic info /tf_static -v | grep -E 'Node name|Publisher count'
fi

echo "--- warnings and errors in the launch log (gz_frame_id notices filtered)"
grep -iE 'error|exception|traceback|\[warn' "$LOG" | grep -v gz_frame_id | head -10
echo "##### done: $WORLD"
