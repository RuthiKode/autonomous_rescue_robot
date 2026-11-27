#!/bin/bash
# Lost-mode go/no-go on every unrecovered teleport bag (docs/PHASE11.md).
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
mkdir -p results/lost_offline   # min_known 40, as lost_min_known in mission.yaml
python3 scripts/reloc_offline.py lost results/lost_offline/searches.csv \
  results/phase10/bags/collapsed_reloc_teleport_seed[2-5]_* \
  results/phase10_attempt1/bags/collapsed_reloc_teleport_seed1_* \
  results/failure_tests/bags/collapsed_fail_teleport_seed[1-5]_*
