# Final report

Simulation-first autonomous search-and-rescue robot, 12-week plan (`docs/PLAN.md`). This page summarises
each phase and links the detailed write-ups, which hold the reproduction commands, the full tables,
deviations and known issues.

## Gates
| week | gate | status | where |
|---|---|---|---|
| 1 | Day-1 gate, TF tree matches contract | met | `docs/PHASE1.md` |
| 2 | seeded worlds, one scored CSV row | met | `docs/PHASE1.md` |
| 3 | mapping baseline per world | met; strict IoU 0.53-0.62 (below 0.8 target), 1-cell tolerance 0.79-0.93 | `docs/PHASE2.md` |
| 4 | navigation baseline, recoveries logged | met; goal success 100 % in collapsed/open_plan, 55 % in rooms | `docs/PHASE3.md` |
| 5 | thin end-to-end slice with a pin | met with an oracle detector; not with pretrained YOLO (0 detections), which led to week 6 | `docs/PHASE4.md` |
| 6 | held-out P/R table; one pin per survivor in 5 runs | detector met; pins met for recall and duplicates, not for precision | `docs/PHASE5.md` |
| 7 | coverage curves for both selectors | met (3 seeds; not separated) | `docs/PHASE6.md` |
| 8 | scripted failures pass in 5 runs; full launch | block 5/5, **teleport 0/5**; full launch works | `docs/PHASE7.md` |
| 9 | sweep, tables from CSV only | met with 3-5 seeds per cell instead of 10 | `docs/PHASE8.md`, `docs/results/` |
| 10 | Foxglove layout, plots, demo recording | layout and offline replay video; no Foxglove screen recording | `docs/PHASE9.md`, `docs/FOXGLOVE.md` |
| 11 | ablations, README, tests, CI | README; 211 tests pass locally; CI passes on GitHub (first run 2026-11-25); ablations limited to the sweep's two (verification, fine-tuning) | below |
| 12 | write-up, resume claims traced | this file, `docs/RESUME_CLAIMS.md` | |
| after | kidnap recovery (week-8 gap) | live: detection 5/5, no false alarm in 5 kidnap-free missions; **teleport 1/5 recovered** (the one landing in mapped space; 4 landed in unmapped space and were correctly not re-seeded); block 3/3 | `docs/PHASE10.md` |
| after | lost mode (unmapped kidnaps) | **teleport 5/5 recovered** (≤ 0.13 m, 4 via lost mode); block 2/2; normal mission 1.00 coverage; offline 8/8 motion-confirmed matches correct at the live setting (9/9 at a looser one) | `docs/PHASE11.md` |
| after | every re-seed confirmed by motion | **teleport 5/5 recovered** (≤ 0.11 m; one start-up failure rerun); 6/6 re-seeds motion-confirmed; block 2/2; normal mission 0.999 coverage; direct path 20-24 s (was 3 s) | `docs/PHASE12.md` |
| after | lost-mode timeout (360 s) | forced 20 s: gave up and kept exploring 2/2, ending DONE with the reason; at 360 s, teleport recovered (224 s lost) and the normal mission was unaffected | `docs/PHASE13.md` |

## Main findings
1. **Detection quality comes from the domain-specific fine-tune.** Paired recall gain +0.61 (95 % CI +0.39 to
   +0.83) over COCO weights, which pinned survivors in only 1 of 13 runs.
2. **Pins are clean in count, not yet in precision.** 0 duplicates everywhere, 0.18-0.50 m error; 1-3 false pins
   on obstacle look-alikes remain in some runs (cell means 0.62-1.00 for the fine-tuned conditions).
3. **The own selector matches explore_lite but does not beat it** at this sample size, and unlike
   explore_lite it ends the mission itself.
4. **Close-range verification, as tuned, hurts recall** (-0.22, CI -0.41 to -0.03): too few close views before
   the mission ends. Lowering `verify_views` or allowing more visits is the next thing to test.
5. **Localization is the main weakness.**
   - **Kidnap:** recovery was 0/5 in week 8.
     - A scan-to-map check, a global search and a slam_toolbox re-seed recovered kidnaps into mapped
       space (phase 10: 1/5).
     - Lost mode then handled kidnaps into unmapped space: SLAM restarts and the robot keeps
       exploring, and a match counts only when two matches agree with odometry. Phase 11: **5/5**
       live, final error ≤ 0.13 m.
     - Getting there took three live attempts. One showed that a single confident match can be an
       alias.
   - **Rooms world:** large SLAM errors in its narrow doors (ATE up to 1.5 m mean for some
     conditions).

## Ablations (week 11)
Only those inside the sweep were run, because each new sweep costs many hours on this machine:
verification on/off and fine-tuned vs pretrained detector (`docs/results/paired.md`). Planned but not run:
hazard weight, gain radius, `min_gain_m2`, verification thresholds.

## Engineering lessons
- Long unattended simulation batches need a hard reset between runs: leaked ROS nodes, Fast DDS shared-memory
  segments and hung map savers cost most of the sweep's 26 hours (`docs/PHASE8.md`).
- slam_toolbox only grows the map after translation; Nav2 must start after the open-loop wake-up drive.
- A recovery trigger based on map->odom jumps cannot detect a kidnap: odometry does not jump.
- Develop against recorded bags first. The kidnap detector and search were calibrated on 16 bags with
  ground truth in minutes. The one live failure mode the bags could not show (slam_toolbox snapping
  to a wrong place on its own) was caught on the first live run and fixed.
- One confident answer is not evidence, two that agree with the robot's own motion are: an alias does
  not move with the robot. Offline, that rule turned 27 wrong single answers (7 at the live setting)
  into 0 wrong accepts.
- Smoke tests must exercise the thing that matters. The first reset test held the robot still, so it
  could not show that slam_toolbox stops mapping after a reset while a pause toggle is still set; the
  live run did.

## What next
- More teleport batches and a second world for kidnap recovery (one batch of 5 so far); a wider margin
  for the jump-verification threshold (0.7 against a 0.66 1st percentile in normal driving).
- Keeping the map explored while lost instead of discarding it, and merging it with the old map after a lost-mode timeout.
- Tune the verification thresholds.
- More seeds (resume `scripts/sweep.sh 1 10`).
- A hazard/doorway-aware SLAM configuration for rooms.
