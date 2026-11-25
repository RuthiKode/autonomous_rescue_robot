#!/bin/bash
# Usage: latest_run.sh <runs-dir> -- newest run dir: its last log lines and last coverage row
cd "$1" || exit 1
d=$(ls -dt */ | head -1)
echo "dir: $d  ($(date +%T))"
tail -c 1200 "$d/launch.log" | cut -c1-200
echo "--- coverage:"; tail -2 "$d/coverage.csv"
