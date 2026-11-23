#!/bin/bash
# Last detector throughput line of every pipeline run that used YOLO (frames, inference time, skipped).
cd ~/autonomous_rescue_robot/results
for log in phase4/runs/*yolo*/launch.log phase5*/runs/*/launch.log; do
  line=$(grep -a "inference mean" "$log" | tail -1 | sed 's/.*yolo_detector\]: //')
  echo "$(basename "$(dirname "$log")"): $line"
done
