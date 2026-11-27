#!/bin/bash
# Seconds spent in lost mode per episode: until the saved map was reloaded ("found"), until the
# lost-mode timeout gave it up ("gave up"), or the end of the log ("still lost").
#   lost_durations.sh RESULTS_DIR...
for out in "$@"; do
  for f in "$out"/runs/*/launch.log; do
    [ -f "$f" ] || continue
    grep -oP '\[\K[0-9.]+(?=\] \[mission_tree\]: (lost: no confident match|lost: no confirmed match|relocalized from lost mode))|mission_tree\]: \K(lost: no confident|lost: no confirmed|relocalized from lost)' "$f" |
      paste - - | awk -v run="$(basename "$(dirname "$f")")" '
        $4=="confident" {t=$1}
        $4=="confirmed" && t {printf "%s gave up after %.0f s\n", run, $1-t; t=0}
        $2=="relocalized" && t {printf "%s found after %.0f s\n", run, $1-t; t=0}
        END {if (t) printf "%s still lost at end of log (entered at %.0f)\n", run, t}'
  done
done
