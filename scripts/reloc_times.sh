#!/bin/bash
# Seconds from the kidnap detection to the first re-seed, per teleport run: reloc_times.sh RESULTS_DIR...
for out in "$@"; do
  for d in "$out"/runs/*teleport*; do
    k=$(grep -m1 -oP '\[\K[0-9.]+(?=\] \[mission_tree\]: kidnapped:)' "$d/launch.log")
    r=$(grep -m1 -oP '\[\K[0-9.]+(?=\] \[mission_tree\]: relocalized)' "$d/launch.log")
    w=$(grep -m1 -oP 'mission_tree\]: \Krelocalized[^:]*' "$d/launch.log")
    if [ -n "$k" ] && [ -n "$r" ]; then
      printf '%s: %.0f s to "%s"\n' "$(basename "$d")" "$(echo "$r - $k" | bc)" "$w"
    else
      echo "$(basename "$d"): no kidnap-to-re-seed pair"
    fi
  done
done
