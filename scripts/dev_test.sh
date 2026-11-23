#!/bin/bash
# Build the packages named in $@ (default: the mission-related ones) into a separate overlay
# (build_dev/, install_dev/) and run their unit tests there. Safe while a simulation batch is
# running from install/: nothing in install/ is touched.
cd ~/autonomous_rescue_robot
for f in $(git ls-files -m -o --exclude-standard src scripts | grep -E '\.(py|xml|cfg|yaml|msg|srv|txt|sh)$'); do
  sed -i 's/\r$//' "$f"
done
PKGS=${@:-rescue_msgs rescue_perception rescue_exploration rescue_mission}
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --build-base build_dev --install-base install_dev --packages-select $PKGS 2>&1 | tail -4
source install_dev/setup.bash
export ROS_DOMAIN_ID=77          # keep the tests off the simulation's DDS domain
for p in $PKGS; do
  [ -d src/$p/test ] && (cd src/$p && python3 -m pytest -q test 2>&1 | tail -4)
done
