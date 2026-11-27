#!/bin/bash
# Proof that every SLAM re-seed in a batch was motion-confirmed: each "relocalized: slam_toolbox
# re-seeded" line must follow a "candidate confirmed" line, and each "relocalized from lost mode" line
# a "lost: match ... confirmed" line, in the same run. Usage: scripts/reseed_proof.sh results/phase12
cd ~/autonomous_rescue_robot
bad=0; n=0
for f in ${1:-results/phase12}/runs/*/launch.log; do
  while IFS= read -r line; do
    case "$line" in
      *"candidate confirmed"*|*"lost: match"*"confirmed by an earlier match"*) ok=1 ;;
      *"relocalized: slam_toolbox re-seeded"*|*"relocalized from lost mode"*)
        n=$((n+1)); [ "$ok" = 1 ] || { bad=$((bad+1)); echo "UNCONFIRMED in $f: ${line:0:160}"; }; ok=0 ;;
    esac
  done < <(grep -hE "candidate confirmed|confirmed by an earlier match|relocalized" "$f")
  ok=0
done
echo "re-seeds: $n, unconfirmed: $bad"
