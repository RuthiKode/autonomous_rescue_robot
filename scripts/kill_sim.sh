#!/bin/bash
# Force-kill every simulation, ROS launch and batch process (used when a batch is stopped mid-run).
for p in $(ps -eo pid,args | grep -E "gz sim|explore_eval|ros2 bag|ros2 launch|install/rescue_|slam_toolbox|ekf_node|phase1[0-9]_tests|nav2|controller_server|bt_navigator" | grep -v -E "grep|kill_sim" | awk '{print $1}'); do
  kill -9 "$p" 2>/dev/null
done
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "left: $(ps -eo args | grep -E 'gz sim|explore_eval|ros2 launch' | grep -v grep | wc -l)"
