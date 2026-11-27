#!/bin/bash
# Fresh-clone check: build and test the committed files only, as the README quick start does.
# Usage: scripts/clone_check.sh [repo url or path]   (default: this checkout)
set -e
SRC=${1:-$HOME/autonomous_rescue_robot}
D=$(mktemp -d /tmp/clonecheck.XXXX)
git clone -q "$SRC" "$D/ws"
cd "$D/ws"
echo "cloned $(git log --oneline -1)"
bash scripts/fetch_third_party.sh >/dev/null
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=76 RESCUE_SKIP_SIM_TESTS=1
colcon build --event-handlers console_cohesion- 2>&1 | tail -1
source install/setup.bash
colcon test --packages-skip explore_lite explore_lite_msgs > /dev/null 2>&1 || true
colcon test-result --all 2>&1 | tail -1
cd /; rm -rf "$D"
