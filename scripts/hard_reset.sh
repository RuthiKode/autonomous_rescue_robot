#!/bin/bash
# Kill every sweep/simulation/ROS process, including hung map savers, clear leaked shared memory,
# and show what is left.   scripts/hard_reset.sh            (also kills a running sweep)
#                          scripts/hard_reset.sh --keep-sweep   (what sweep.sh calls between runs)
pats=("map_saver_cli" "explore_eval" "ros2 bag" "install/rescue_" "gz sim")
[ "$1" != "--keep-sweep" ] && pats=("sweep.sh" "wait_for.sh" "${pats[@]}")
for pat in "${pats[@]}"; do
  pkill -KILL -f "$pat" 2>/dev/null
done
bash ~/cleanup.sh 2>&1 | tail -4
source /opt/ros/jazzy/setup.bash
ros2 daemon stop 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "--- left:"
pgrep -af "gz|ros2|python3|map_saver" | grep -v -E "pgrep|hard_reset|unattended" | cut -c1-100 | head
echo "shm entries: $(ls /dev/shm | wc -l)"
free -m | sed -n 2p
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader 2>&1 | head -1
