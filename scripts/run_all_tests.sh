#!/bin/bash
# Build everything and run colcon test (simulation tests skipped). Prints per-package results.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
export RESCUE_SKIP_SIM_TESTS=1
export ROS_DOMAIN_ID=77
colcon build --packages-skip explore_lite explore_lite_msgs 2>&1 | tail -3
source install/setup.bash
colcon test --packages-skip explore_lite explore_lite_msgs --return-code-on-test-failure > /tmp/colcon_test.log 2>&1
echo "colcon test exit: $?"
colcon test-result --all 2>&1 | tail -3
colcon test-result 2>&1 | grep -E "errors|failures" | head -20
