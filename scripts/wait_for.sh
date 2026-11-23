#!/bin/bash
# Usage: wait_for.sh <logfile> <marker> [max_minutes]  -- wait until marker appears in logfile, then print its tail
f=$1; m=$2; max=${3:-9}
for ((i=0; i<max*6; i++)); do
  grep -q "$m" "$f" 2>/dev/null && break
  sleep 10
done
tail -40 "$f" | cut -c1-220
