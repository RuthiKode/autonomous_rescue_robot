#!/bin/bash
# Build the week-7 packages and check that they import.
cd ~/autonomous_rescue_robot
for f in $(git ls-files -m -o --exclude-standard src scripts | grep -E '\.(py|xml|cfg|yaml|msg|srv|txt|sh)$'); do
  sed -i 's/\r$//' "$f"
done
source /opt/ros/jazzy/setup.bash
colcon build 2>&1 | tail -6
source install/setup.bash
ros2 pkg executables rescue_mission
ros2 pkg executables rescue_exploration
python3 -c "from rescue_mission import behaviours; print('behaviours import ok')"
python3 -c "from rescue_msgs.srv import SelectFrontier; from rescue_msgs.msg import MissionState; print('msgs ok')"
