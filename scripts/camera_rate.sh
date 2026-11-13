#!/bin/bash
# Camera rate and dropped-frame measurement for one world, headless.
#
#   scripts/camera_rate.sh <world> [duration_sim_s] [arm ...]
#
# Arms (default: all): raw, deserialize, reliable, bag
#   raw          Python subscriber that does not decode the image
#   deserialize  Python subscriber that decodes every image (a normal rclpy node)
#   reliable     raw subscriber with reliable QoS and a deeper queue
#   bag          ros2 bag record (C++), then stamps are read back from the bag
WORLD=${1:?usage: camera_rate.sh <world> [duration] [arm ...]}
DURATION=${2:-60}
shift; shift 2>/dev/null
ARMS=${*:-raw deserialize reliable bag}
WS=$(cd "$(dirname "$0")/.." && pwd)
source "$WS/install/setup.bash"
LOG=$(mktemp /tmp/camera_rate_${WORLD}_XXXX.log)

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
echo "##### world=$WORLD duration=${DURATION}s sim"
echo "Gazebo $(timeout 5 gz topic -e -t /stats -n 1 2>/dev/null | grep real_time_factor)"
for ARM in $ARMS; do
  echo "--- arm: $ARM"
  case $ARM in
    raw)         ros2 run rescue_eval camera_rate_check --duration $DURATION --mode raw ;;
    deserialize) ros2 run rescue_eval camera_rate_check --duration $DURATION --mode deserialize ;;
    reliable)    ros2 run rescue_eval camera_rate_check --duration $DURATION --mode raw \
                   --reliability reliable --depth 50 ;;
    bag)
      BAG=$(mktemp -d /tmp/camera_bag_XXXX)/bag
      timeout -s INT $((DURATION + 15)) ros2 bag record -o "$BAG" /camera/image_raw \
        /camera/camera_info > /dev/null 2>&1
      python3 "$WS/scripts/bag_stamp_gaps.py" "$BAG" /camera/image_raw /camera/camera_info
      rm -rf "$(dirname "$BAG")"
      ;;
  esac
done
echo "Gazebo $(timeout 5 gz topic -e -t /stats -n 1 2>/dev/null | grep real_time_factor)"
echo "##### done: $WORLD"
