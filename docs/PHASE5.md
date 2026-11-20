# Phase 5 (plan week 6): synthetic dataset and fine-tuned YOLO

Plan gate for week 6: *held-out precision and recall table; one pin per survivor in 5 runs.*

**Status: the detector gate is met, the pin gate is only half met.** Every true survivor got a pin in 17 of
18 pipeline runs, but the pipeline also confirms many false pins (5 to 44 confirmed pins per run for 3 or 4
survivors; pin precision 0.07 to 0.27).
Details and a proposed fix are at the end; I stopped here for your decision.

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

## Gate check
| gate item | result |
|---|---|
| Held-out precision and recall table | done (above); collapsed precision 0.56 to 0.68 is the weak point |
| Sustained inference at the camera rate | met: 0 skipped frames in 18 runs |
| One pin per survivor in 5 runs | **not met as stated**: every survivor is pinned in 17 of 18 runs (the exception is round-2 rooms seed 1), duplicates are rare (0.00 to 0.10), but 5 to 44 pins are confirmed per run for 3 or 4 survivors |

## Deviations and decisions
1. Boxes come from a segmentation camera as the plan says, but frames use teleported poses rather than driving
   the robot through the worlds, to get 3,240 labelled frames in about 1.5 h of wall time.
2. The sun direction and strength are randomized; survivor colours differ by world seed. Textures, survivor model
   shapes and debris appearance are not randomized.
3. I changed the committed worlds' SDF (a label plugin per survivor). Geometry and truth files are unchanged.
4. Dataset frames and weights are not committed (`.gitignore`); the dataset is rebuilt by the scripts above.
5. Ultralytics is AGPL-3.0, as noted in the plan, so check the licence before publishing.

## Known issues
- False pins as described; the pipeline currently reports an unusable pin set although recall is perfect.
- Sim-to-real gap: the worlds are flat-shaded low-poly scenes; nothing here says how the detector behaves on real
  images. Report it as a limitation.
- Single training run per fold (seed 0); the numbers carry no spread.
- Test frames are my sampled poses, not exploration views.

## Decision for you
Fix the false pins before week 7? My recommendation: yes, in two steps that stay inside this week's work:
1. **Mine hard negatives from exploration runs.** Run the segmentation camera during explore_eval, label frames
   along the real exploration path, add the frames where the detector fires without a label, retrain. This also
   makes the held-out frame test match what the pipeline sees.
2. **Confirm by hit rate in the localizer** (needs a per-track "in view" count), tuned on these bags.
Expect about half a day. The alternative is to leave it and rely on the week-8 Verify branch, but verifying 30
to 40 pins per run would dominate mission time.
