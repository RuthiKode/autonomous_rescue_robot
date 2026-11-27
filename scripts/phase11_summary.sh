#!/bin/bash
# Per teleport run of a phase-11 batch: landing mapped share, time spent lost, held candidates, verdict.
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
B=${1:-results/phase11}
python3 scripts/reloc_offline.py landing $B 2>&1 | sed -E 's/_2026[0-9T]+Z//'
for d in $B/runs/*teleport*/; do
  t0=$(grep -h "lost: no confident" $d/launch.log | head -1 | sed -E 's/.*\[([0-9]+\.[0-9]+)\].*/\1/')
  t1=$(grep -h "relocalized from lost mode" $d/launch.log | head -1 | sed -E 's/.*\[([0-9]+\.[0-9]+)\].*/\1/')
  n=$(grep -c "lost: candidate match" $d/launch.log)
  v=$(grep -ho '"verdict": "[^"]*"' $d/injection.json)
  echo "$(basename $d | sed -E 's/_2026[0-9T]+Z//'): lost ${t0:+for $(python3 -c "print(round($t1-$t0))") s (wall), $n held candidates}${t0:-no}; $v"
done
