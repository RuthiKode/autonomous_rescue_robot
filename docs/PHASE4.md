# Phase 4 (plan week 5): thin end-to-end slice

Goal (docs/PLAN.md, week 5 gate): the robot explores on its own and drops at least one survivor pin.
Slice: explore_lite frontier exploration + pretrained YOLO (person only) + a crude survivor pin on the map.

## What was built
- `src/rescue_exploration`: explore_lite config and launch (explore_lite itself is fetched, not committed:
  `scripts/fetch_third_party.sh`, pinned to commit 326cf8a of m-explore-ros2).
- `src/rescue_perception`: `yolo_detector` (YOLOv8n, person class, runs in its own venv), `oracle_detector`
  (ground-truth TEST DOUBLE, see deviations), `survivor_localizer` (detection + depth + TF at the detection
  stamp -> map pin; confirm after 5 hits from 2 viewpoints), pure-Python tracker and geometry with tests.
- `src/rescue_eval`: `explore_eval` (one seeded exploration run), coverage metric, one-to-one pin scoring.
- `scripts/phase4_runs.sh` (all runs below), `scripts/render_phase4.py` (figures), `scripts/detector_probe.py`.

## Reproduce
```bash
scripts/fetch_third_party.sh && colcon build
scripts/phase4_runs.sh            # about 1 h 10 min wall; results/phase4/runs.csv
python3 scripts/render_phase4.py  # docs/img/phase4/*
# single run:
ros2 run rescue_eval explore_eval --world collapsed --seed 1 --detector oracle --method demo
```
The YOLO venv is `~/venvs/yolo` with weights `~/venvs/yolo/weights/yolov8n.pt` (not committed; Ultralytics is AGPL-3.0).

## Results (seed 1 unless noted; every number is from `results/phase4/runs.csv`)

| world | detector | coverage | time to 90 % | pins matched | precision / recall | pin error | map IoU / tol. IoU | RTF |
|---|---|---|---|---|---|---|---|---|
| collapsed | oracle | 0.999 | 95 s | 4 of 4 | 1.00 / 1.00 | 0.23 m | 0.619 / 0.891 | 0.89 |
| rooms | oracle | 0.957 | 286 s | 3 of 3 | 1.00 / 1.00 | 0.27 m | 0.427 / 0.740 | 0.88 |
| open_plan | oracle | 0.992 | 314 s | 3 of 3 | 1.00 / 1.00 | 0.14 m | 0.595 / 0.890 | 0.88 |
| collapsed | **real YOLOv8n** | 1.000 | 166 s | **0 of 4** | n/a / 0.00 | n/a | 0.630 / 0.898 | 0.88 |

Figures: `docs/img/phase4/pins_vs_truth.png` (final map, path, pins vs truth), `coverage_curves.png`,
`collapsed_path.gif` (path replay over the final map; it is not a simulator recording, and the pins are not shown
in it because pins carry no per-frame timestamps in the saved output).

### Gate: "explores and drops at least one pin"
Met with the oracle detector in all three worlds (every true survivor pinned, no duplicates, no false pins).
**Not met with the real pretrained detector: it produced zero detections** through the live pipeline
(5,084 frames, localizer log "detections 0"). Inference itself is cheap (mean about 30 ms, max about 70 ms per
frame) and does not slow the sim (RTF 0.88 vs 0.89 with the oracle). The earlier probe (`results/detector_probe/`)
found the same: 1 of 27 views detected, confidence 0.08. COCO weights do not recognise the simulated survivor
models, so week 6 (fine-tuning on simulated data) is required; the localizer and pin logic are unaffected by that.

### explore_lite tuning (collapsed world, no detector, seeds 1 and 2)
| variant | planner_freq / min_frontier / potential / gain | time to 90 % (s) | total time (s) |
|---|---|---|---|
| A upstream defaults | 0.15 / 0.75 / 3 / 1 | 199, 140 | 344, 256 |
| B project | 0.3 / 0.5 / 3 / 1 | 139, 189 | 415, 347 |
| C gain-heavy | 0.3 / 0.5 / 1 / 3 | 171, 135 | 365, 368 |

All six runs reached coverage of at least 0.999. Seed-to-seed spread (about 50 s) is as large as the gaps between
variants, so two seeds do not rank them. I kept B as the default; a real comparison needs more seeds.

## Deviations and decisions to review
1. **Oracle detector.** Pins in the table come from a test double that projects ground-truth survivor shapes through
   the camera. It validates localization, tracking and scoring only; it says nothing about perception quality.
   It is labelled as such in code, config and CSV method names.
2. **explore_lite built from source** (no Jazzy binary); fetched at a pinned commit, ignored by git.
3. **Wake-up drive.** slam_toolbox's map stays a tiny seed until the robot translates, and explore_lite then stops
   at once, so `explore_eval` drives forward 3 s before Nav2 starts (`explore.wakeup_*` in eval.yaml).
4. **YOLO in a separate venv** to keep AGPL code out of the ROS workspace; weights are not committed.

## Known issues
- Rooms world: lowest coverage (0.957) and map IoU (0.427 / tolerance 0.740). I did not diagnose it; the known
  rooms SLAM pose jump from earlier phases is a candidate, not a confirmed cause.
- One seed per world for the oracle runs; the tuning comparison has two seeds.
- Plateau stop rule ends runs once coverage stalls, so "total time" is partly set by that rule.
- Nothing here was checked in a GUI (Gazebo/RViz); evidence is the saved maps, CSVs and figures. Please look at
  `pins_vs_truth.png`.

## Decision for you
Go to week 6 (generate simulated person data, fine-tune YOLO, rerun this pipeline with `--detector yolo`)?
