#!/bin/bash
# Stop a running phase test batch and everything it started.
pkill -f "scripts/phase1[0-9]_tests.sh" 2>/dev/null
pkill -f "rescue_eval/explore_eval" 2>/dev/null
sleep 2
bash ~/autonomous_rescue_robot/scripts/hard_reset.sh --keep-sweep >/dev/null 2>&1
ps -eo args | grep -E "explore_eval|gz sim|phase1[0-9]_tests" | grep -v grep | head -3
echo stopped
