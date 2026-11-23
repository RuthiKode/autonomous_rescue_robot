# Phase 5 (plan week 6): synthetic dataset and fine-tuned YOLO

Plan gate for week 6: *held-out precision and recall table; one pin per survivor in 5 runs.*

**Status: the detector gate is met; the pin gate is met for recall and duplicates, not yet for precision.**
The first fine-tuned detector pinned every survivor but also confirmed 5 to 44 pins per run for 3 or 4
survivors (precision 0.07 to 0.27). "Round 4" (section below) mined hard negatives from exploration frames and
tightened the pin rule: on the committed worlds the pipeline now confirms 3 to 7 pins per run (mean precision
0.74, every survivor pinned, no duplicates), and on two generated worlds the detector had never seen, 19 of 25
survivors were pinned (19 of 20 when the one run whose exploration stalled is left out) with mean precision
0.84. Extra false pins remain in most runs, so a verification step is still needed.

## What was built
- **Labels without hand work.** `rescue_simulation` survivor models carry a Gazebo semantic label
  (`gz-sim-label-system`, label 1). `rescue_description` has an optional segmentation camera
  (`seg_camera:=true`, off by default, never used in evaluation runs). `boxes_from_mask`
  (`rescue_perception/dataset.py`) turns the mask into boxes; pieces of one person cut by debris are
  joined (6 px), boxes under 60 px area or 6 px side are dropped.
- **Collector.** `scripts/collect_dataset.py` generates a world, starts the simulator headless, teleports the
  robot to sampled poses (65 % looking at a survivor from 1.2 to 6 m with 0.25 rad jitter, the rest random free
  poses), changes the sun direction and strength every 10 frames, and saves an RGB frame, a YOLO label file and
  a `meta.csv` row (pose, light, and for each box the survivor's posture and occluded flag).
  `sim.launch.py` gained `seg_camera` and `worlds_dir`.
- **Dataset.** 3 layouts x 9 world seeds (42 is the committed world) x 120 frames = 3,240 frames, 640x480,
  with the robot's own camera noise. Survivor shirt, trousers and skin colours come from the world seed.
  Label sample: `docs/img/phase5/labels_sample.png`.
- **Training and scoring.** `scripts/train_detector.py`: YOLOv8n from the COCO weights, 40 epochs at 640 px
  (patience 15), splits by **layout**. Scoring is my own matcher (IoU >= 0.5, confidence >= 0.30, the
  localizer's threshold) so it can break results down by posture, occlusion and box height.
- **Pipeline hook.** `explore_eval --model PATH` and `perception.launch.py model:=PATH`.

## Reproduce
```bash
scripts/collect_all.sh 120                 # seeds 42,1-5; SEEDS="6 7 8" scripts/collect_all.sh 120 adds more (about 25 min each)
scripts/train_all.sh && scripts/eval_all.sh  # leave-one-layout-out, round 1 (about 40 min)
scripts/round2.sh                          # seeds 6-8, retrain as *_r2, pipeline runs
scripts/round3.sh                          # detector trained on all layouts except world seed 42, pipeline runs
python3 scripts/phase5_tables.py           # every table below, from the result files only
```
Tables are also saved in `docs/img/phase5/tables.md`.

## Held-out detector results

Split by layout: a model never sees the layout it is tested on; seed-5 worlds of the training layouts are the
validation set. Test = all frames of the held-out layout (world seeds 42 and 1 to 5, 720 frames, about 200 to
600 labelled boxes). All numbers at confidence >= 0.30, IoU >= 0.5.

| model | held-out | pretrained P / R | fine-tuned P / R | tp / fp / fn |
|---|---|---|---|---|
| round 1 | collapsed | n/a / 0.00 | 0.58 / 0.92 | 425 / 312 / 39 |
| round 1 | rooms | n/a / 0.00 | 0.93 / 0.96 | 194 / 14 / 9 |
| round 1 | open_plan | n/a / 0.00 | 0.93 / 0.94 | 559 / 40 / 36 |
| round 2 (+3 training worlds per layout) | collapsed | n/a / 0.00 | 0.56 / 0.94 | 437 / 340 / 27 |
| round 2 | rooms | n/a / 0.00 | 0.92 / 0.94 | 190 / 16 / 13 |
| round 2 | open_plan | n/a / 0.00 | 0.93 / 0.96 | 571 / 45 / 24 |
| all layouts, world seed 42 held out | all | n/a / 0.00 | 0.95 / 0.98 | 194 / 11 / 4 |

The pretrained COCO model finds nothing at this threshold (0 detections in every test set; precision undefined), so the
fine-tuning is what makes detection work at all. The training-validation mAP50 was 0.96 to 0.98.

Confidence sweep, round 1 (precision / recall):

| held-out | conf 0.30 | conf 0.50 | conf 0.70 |
|---|---|---|---|
| collapsed | 0.58 / 0.92 | 0.68 / 0.90 | 0.80 / 0.86 |
| rooms | 0.93 / 0.96 | 0.94 / 0.93 | 0.97 / 0.91 |
| open_plan | 0.93 / 0.94 | 0.95 / 0.92 | 0.96 / 0.86 |

Recall by group, round 1 fine-tuned, held-out layouts (n = labelled boxes):

| held-out | standing | sitting | lying | not occluded | occluded | box < 20 px high | 20 to 60 px | >= 60 px |
|---|---|---|---|---|---|---|---|---|
| collapsed | 0.88 (163) | 0.94 (109) | 0.95 (151) | 0.91 (267) | 0.93 (156) | 0.73 (33) | 0.92 (96) | 0.93 (335) |
| rooms | 0.89 (55) | 1.00 (49) | 0.99 (73) | 0.98 (112) | 0.92 (65) | 1.00 (8) | 0.95 (22) | 0.95 (173) |
| open_plan | 0.95 (189) | 0.95 (150) | 0.94 (203) | 0.95 (316) | 0.94 (226) | 0.86 (83) | 0.93 (154) | 0.96 (358) |

Posture and occlusion matter little; small, far boxes are the weak spot. "Occluded" here is the world
generator's flag; boxes already show only the visible part.

**Collapsed is the weak layout (precision 0.56 to 0.68).** `docs/img/phase5/fp_collapsed.png` shows the false
positives: grey debris slabs and thin pieces next to or on top of real survivors. More training worlds (round 2)
did not help (0.58 to 0.56), so this is not a data-volume problem.

## Pipeline runs with the fine-tuned detector (explore_eval, 2 Gazebo seeds per world)

Round 1 detectors, each trained without the layout it runs in (the honest generalization case):

| world | seed | coverage | true survivors pinned | confirmed pins | pin precision | duplicate rate | pin error (m) | real-time factor |
|---|---|---|---|---|---|---|---|---|
| collapsed | 1 | 1.000 | 4 of 4 | 40 | 0.10 | 0.05 | 0.18 | 0.84 |
| rooms | 1 | 0.825 | 3 of 3 | 14 | 0.21 | 0.00 | 0.30 | 0.90 |
| open_plan | 1 | 0.998 | 3 of 3 | 30 | 0.10 | 0.00 | 0.07 | 0.88 |
| collapsed | 2 | 0.999 | 4 of 4 | 42 | 0.10 | 0.10 | 0.25 | 0.90 |
| rooms | 2 | 0.984 | 3 of 3 | 11 | 0.27 | 0.09 | 0.28 | 0.89 |
| open_plan | 2 | 0.998 | 3 of 3 | 30 | 0.10 | 0.03 | 0.17 | 0.85 |

Round 2 and the all-layouts detector behave the same: 1 to 4 survivors pinned per run, 5 to 44 confirmed pins,
pin precision 0.07 to 0.25 (full tables: `docs/img/phase5/tables.md`). "Confirmed pins" is derived from pin
recall, precision and the number of true survivors (the run logs print the same counts, e.g. 40 confirmed).
Round-2 rooms seed 1 pinned only 1 of 3 survivors with coverage 0.547; I did not diagnose it (the rooms SLAM
pose jump from Phase 3 is a candidate, not a confirmed cause).

### Why the pins are wrong (diagnosis, not yet fixed)
- Frame-level precision of 0.93 to 0.95 is not enough: a run sees 5,000 to 8,500 frames, and obstacles that look
  like people are seen again and again from several viewpoints, so the localizer's rule (5 hits from 2
  viewpoints) confirms them. `scripts/false_pin_where.py` on open_plan seed 1: the false tracks sit within
  0.1 to 0.9 m of an obstacle centre, several with 200 to 350 hits and confidence 0.65 to 0.86.
- Real survivors do have more hits and higher confidence (median hits 104 to 426, confidence 0.70 to 0.90 vs a
  false-track median confidence of about 0.5), but the ranges overlap.
- `scripts/hit_rate_diagnostic.py` (hits per frame in which the track was in view, occlusion ignored): in
  open_plan the true tracks score 0.38 to 0.92 against a false median of 0.06, so a hit-rate rule would help
  there. In collapsed and rooms the true tracks reach down to 0.01 to 0.07 (heavy occlusion, partial boxes), so
  it does not separate cleanly.
- My teleport sampling (65 % of frames aim at a survivor) does not match what the robot sees while exploring,
  so the held-out frame precision is optimistic for the pipeline.

### Throughput (gate: inference keeps up with the camera)
Camera 15 Hz. Across all 18 runs the detector skipped 0 frames at its rate cap; mean inference 26 to 57 ms
(budget 67 ms), maximum 50 to 352 ms in single frames. Real-time factor 0.67 to 0.91 (the Phase 4 oracle runs
were 0.88 to 0.89); the lowest values came in the later runs of the session, which I did not investigate.
`scripts/detector_throughput.sh` lists every run.

## Round 4: fixing the false pins

Three changes, in the order they were tried. Tables: `docs/img/phase5/tables.md`.

**1. Hard negatives from exploration frames.** `explore_eval --collect-frames DIR` starts the segmentation
camera and `frame_logger` (one labelled frame per sim second) while the robot explores. `scripts/collect_explore.sh`
did this in generated worlds with seeds 11, 12 and 13 (3 layouts each, 4,101 frames); seeds 11 and 12 train,
seed 13 validates. `train_detector.py train --holdout all --explore --mine WEIGHTS` adds the frames and repeats
every frame on which the previous detector fired without a label (1,136 of 2,921) twice more. Result on the
exploration frames of the unseen seed-13 worlds (1,180 frames, 352 labelled boxes):

| detector | frames with a false positive | false boxes | recall |
|---|---|---|---|
| round 3 (teleport frames only) | 441 (37.4 %) | 475 | 0.87 |
| round 4 (+ exploration frames, mined) | 11 (0.9 %) | 11 | 0.97 |

So the earlier teleport test did hide the problem: along real exploration paths the old detector fired falsely in
more than a third of the frames. On the teleport test frames the new detector scores 0.92 / 0.96 (P / R) against
0.95 / 0.98 before, which I read as a distribution effect, not a regression. Training here uses the
exploration worlds, which differ from the committed worlds the runs below use.

**2. Hit-rate rule (tested, left off).** The localizer counts, for every track, the frames in which its point
was visible to the camera (inside the field of view and range, no nearer surface in the depth image) and can
require hits / views >= `min_hit_rate`. With 0.25 over 5 runs the pin precision was 0.38 to 0.75 and the
duplicate rate went up (0.22 to 0.38 against 0.00 to 0.20 without it), so there is no clear gain; the code and
tests stay, the parameter is 0 (off). One run of that batch failed at start-up (`startup_timeout`, a simulator
start-up flake, not a result).

**3. Pin rule: more hits, a confidence floor, and merging.** `scripts/confirm_sweep.py` replays the rule offline
on the final track sets of the six no-rule runs (the scorer only sees the final tracks, and tracks form the
same way whatever the thresholds). Keeping confirmed tracks with hits >= 20 and mean confidence >= 0.6, and
reporting only the strongest track within 1.5 m of another, raised mean pin precision from 0.51 to 0.73 with
recall 1.00 and no duplicates (hits >= 80 lost a survivor in one of six runs). The localizer now uses
`confirm_hits: 20`, `confirm_confidence: 0.6`, `merge_radius_m: 1.5` (`perception.yaml`); merged tracks are
published with state `merged` and do not count as pins. The thresholds were picked on those six runs, so the
committed-world results below are not independent of the choice; the unseen worlds are.

### Pipeline runs with the round-4 detector and the final rule

Committed worlds, 2 Gazebo seeds per world:

| world | seed | coverage | survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) |
|---|---|---|---|---|---|---|---|
| collapsed | 1 | 0.967 | 4 of 4 | 6 | 0.67 | 0.00 | 0.55 |
| rooms | 1 | 0.912 | 3 of 3 | 3 | 1.00 | 0.00 | 0.31 |
| open_plan | 1 | 0.998 | 3 of 3 | 5 | 0.60 | 0.00 | 0.12 |
| collapsed | 2 | 0.984 | 4 of 4 | 7 | 0.57 | 0.00 | 0.34 |
| rooms | 2 | 0.905 | 3 of 3 | 3 | 1.00 | 0.00 | 0.37 |
| open_plan | 2 | 0.999 | 3 of 3 | 5 | 0.60 | 0.00 | 0.19 |

Generated worlds with seeds 21 and 22 (never used for training, mining or choosing the rule), 1 Gazebo seed:

| world | coverage | survivors pinned | confirmed pins | pin precision | duplicates | pin error (m) |
|---|---|---|---|---|---|---|
| collapsed, world 21 | 0.965 | 3 of 4 | 3 | 1.00 | 0.00 | 0.41 |
| rooms, world 21 | 0.551 | 0 of 5 | 0 | n/a | n/a | n/a |
| open_plan, world 21 | 1.000 | 3 of 3 | 4 | 0.75 | 0.00 | 0.30 |
| collapsed, world 22 | 0.980 | 3 of 3 | 4 | 0.75 | 0.00 | 0.24 |
| rooms, world 22 | 0.944 | 5 of 5 | 5 | 1.00 | 0.00 | 0.39 |
| open_plan, world 22 | 1.000 | 5 of 5 | 7 | 0.71 | 0.00 | 0.29 |

Against the first fine-tuned detector in the same committed worlds (23 to 44 confirmed pins, precision 0.07 to
0.13), the pin set is about a seventh the size. Rooms world 21 is an exploration failure (coverage 0.551, the
run stopped on its plateau rule), not a detection failure; I did not diagnose it (the rooms SLAM pose jump
is a candidate). Collapsed world 21 missed one survivor; I did not check why. Remaining false pins sit on
obstacles; I did not examine them after the rule change.

Throughput in these runs: no skipped frames; real-time factor 0.31 to 0.91, lower than the 0.85 to 0.90 of
the earlier batches (the lowest was rooms world 21). I did not find the cause; the session was interrupted once
(WSL restarted) during this batch, and the five runs after it were resumed with the same settings
(`scripts/phase5y_resume.sh`).

## Gate check
| gate item | result |
|---|---|
| Held-out precision and recall table | done (above); collapsed precision 0.56 to 0.68 on teleport frames was the weak point |
| Sustained inference at the camera rate | met: 0 skipped frames in all runs |
| One pin per survivor in 5 runs | **met for recall and duplicates**: with the round-4 detector and rule, 10 of 12 runs pin every survivor with 0 duplicates (the other two: world 21 collapsed, 1 of 4 missed; world 21 rooms, exploration stalled, 0 of 5). **Not met for precision**: 7 of 11 runs with pins still confirm 1 to 3 extra pins (precision 0.57 to 0.75) |

## Deviations and decisions
1. Boxes come from a segmentation camera as the plan says, but frames use teleported poses rather than driving
   the robot through the worlds, to get 3,240 labelled frames in about 1.5 h of wall time.
2. The sun direction and strength are randomized; survivor colours differ by world seed. Textures, survivor model
   shapes and debris appearance are not randomized.
3. I changed the committed worlds' SDF (a label plugin per survivor). Geometry and truth files are unchanged.
4. Dataset frames and weights are not committed (`.gitignore`); the dataset is rebuilt by the scripts above.
5. Ultralytics is AGPL-3.0, as noted in the plan, so check the licence before publishing.

## Known issues
- 1 to 3 extra pins remain in most runs (obstacle look-alikes); pin precision 0.57 to 1.00.
- Rooms exploration is unreliable in some worlds (coverage 0.551 in world 21, 0.547 in one earlier run); the
  pipeline cannot pin what the robot never sees. Not diagnosed.
- Real-time factor dropped in the later batches (0.31 to 0.91); cause not found.
- Sim-to-real gap: the worlds are flat-shaded low-poly scenes; nothing here says how the detector behaves on real
  images. Report it as a limitation.
- Single training run per detector (seed 0), 1 to 2 runs per condition; the numbers carry no spread.
- The pin rule's thresholds were chosen on the committed worlds' earlier runs (see round 4).
- The leave-one-layout-out detectors were not retrained with exploration frames, so the cross-layout
  generalization numbers above are for the first (teleport-only) detectors.

## Decision for you
Move on to week 7 (own frontier selector, behaviour tree)? My recommendation: yes. The pin set is small enough
(3 to 7 pins for 3 to 5 survivors) for the week-8 Verify branch to handle the remaining extras by taking a
second look at each pin. Two things I would watch: the rooms exploration stalls (they will also affect the
week-7 coverage curves) and the real-time factor drop.
