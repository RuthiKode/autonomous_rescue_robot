#!/bin/bash
# Offline calibration of kidnap detection and global search (docs/PHASE10.md).
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
F=results/failure_tests/bags; S=results/sweep/bags
python3 scripts/reloc_offline.py calibrate results/reloc_offline \
  $F/collapsed_fail_teleport_seed[1-5]_* $F/collapsed_fail_block_seed[1245]_* \
  $S/rooms_own_ft_seed1_20261124T084623Z $S/rooms_own_ft_seed2_20261124T113130Z \
  $S/rooms_own_ft_seed3_20261124T161926Z $S/rooms_own_ft_seed4_20261124T182608Z \
  $S/open_plan_own_ft_seed1_20261124T100408Z $S/open_plan_own_ft_seed2_20261124T145622Z \
  $S/collapsed_own_ft_seed1_20261123T224003Z
