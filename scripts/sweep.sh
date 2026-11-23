#!/bin/bash
# Week 9: the full evaluation sweep. Same worlds and Gazebo seeds for every condition; one run at a
# time (the machine cannot run two simulations). Resumable: a (world, condition, seed) that already
# has a row with status ok in results/sweep/runs.csv is skipped.
#
#   scripts/sweep.sh                 everything (about 120 runs, 15 to 20 hours)
#   scripts/sweep.sh 1 3             only seeds 1 to 3
#   CONDITIONS="own_ft lite_ft" scripts/sweep.sh
#
# Conditions (docs/PLAN.md, evaluation):
#   random_ft       random reachable goals, fine-tuned YOLO                       (seeds 1-5)
#   lite_ft         explore_lite with its upstream default parameters, fine-tuned YOLO
#   own_ft          own frontier selector + mission tree, no verification, fine-tuned YOLO
#   ownverify_ft    own selector + mission tree + verification, fine-tuned YOLO
#   ownverify_pre   as ownverify_ft but with the pretrained COCO weights              (seeds 1-5)
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OUT=${OUT:-results/sweep}
FIRST=${1:-1}
LAST=${2:-10}
CONDITIONS=${CONDITIONS:-"lite_ft own_ft ownverify_ft random_ft ownverify_pre"}
MODEL=results/detector/runs/all_x/weights/best.pt
LITE_DEFAULTS=results/phase4_tuning/A_upstream_defaults.yaml

flags_for() {
  case $1 in
    random_ft)     echo "--explorer random --detector yolo --model $MODEL" ;;
    lite_ft)       echo "--explorer lite --detector yolo --model $MODEL --explore-params $LITE_DEFAULTS" ;;
    own_ft)        echo "--explorer own --detector yolo --model $MODEL --no-plateau" ;;
    ownverify_ft)  echo "--explorer own --verify --detector yolo --model $MODEL --no-plateau" ;;
    ownverify_pre) echo "--explorer own --verify --detector yolo --no-plateau" ;;
  esac
}
max_seed_for() {
  case $1 in random_ft|ownverify_pre) echo 5 ;; *) echo 10 ;; esac
}
done_already() {  # world condition seed
  [ -f $OUT/runs.csv ] && python3 - "$OUT/runs.csv" "$1" "$2" "$3" <<'PY'
import csv, sys
path, world, method, seed = sys.argv[1:]
ok = any(r['world'] == world and r['method'] == method and r['seed'] == seed and r['status'] == 'ok'
         for r in csv.DictReader(open(path)))
sys.exit(0 if ok else 1)
PY
}

for seed in $(seq $FIRST $LAST); do
  for world in collapsed rooms open_plan; do
    for cond in $CONDITIONS; do
      [ $seed -le $(max_seed_for $cond) ] || continue
      if done_already $world $cond $seed; then continue; fi
      bash ~/cleanup.sh >/dev/null 2>&1
      echo "=== $cond $world seed $seed $(date +%T)"
      ros2 run rescue_eval explore_eval --world $world --seed $seed --method $cond \
          --output-dir $OUT $(flags_for $cond) 2>&1 \
          | grep -E "coverage [0-9.]+ after|tracks:|pins:|status|injected|injection"
    done
  done
done
echo SWEEP_DONE
