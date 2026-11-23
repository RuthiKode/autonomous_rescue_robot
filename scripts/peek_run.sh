#!/bin/bash
# Usage: peek_run.sh <runs-dir> <glob> [egrep pattern]  -- list a run dir and grep its launch log
cd "$1" || exit 1
d=$(ls -d $2 | head -1)
echo "dir: $d"
ls "$d"
grep -a -E "${3:-mission state|Traceback|ERROR|localization jump}" "$d/launch.log" | cut -c1-220 | head -40
