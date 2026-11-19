#!/bin/bash
# Phase 3 checks that the random-goal baseline does not cover.
#
#   scripts/phase3_extra.sh [output_dir]
#
# 1. Gaps: collapsed world, goals across the barrier gaps (0.7 m wide), out and back.
#    The warm-up drive stays in the start zone, so the far zones are not in the map yet.
# 2. Unreachable goal: rooms world, a goal outside the building. Nav2 cannot plan to it;
#    its recovery behaviours must fire, be logged, and the goal must end aborted, not hang.
WS=$(cd "$(dirname "$0")/.." && pwd)
OUT=${1:-$WS/results/phase3_extra}
mkdir -p "$OUT"
echo "######## 1. collapsed world: crossing the gaps"
# east zone through the gap at (2.5, -1.7); north zone through the gap at (0.0, 2.0)
"$WS/scripts/nav_staged.sh" collapsed "$OUT/gaps.csv" --goal-timeout 150 \
  --goal=5.0,-1.5 --goal=1.0,-1.0 --goal=-0.5,5.0 --goal=1.0,-1.0 2>&1
echo "######## 2. rooms world: unreachable goal (outside the building)"
"$WS/scripts/nav_staged.sh" rooms "$OUT/unreachable.csv" --goal-timeout 120 \
  --goal=11.0,0.0 --goal=1.0,0.0 2>&1
echo EXTRA_DONE
