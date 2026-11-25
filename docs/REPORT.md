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

## Main findings
1. **Detection quality comes from the domain-specific fine-tune.** Paired recall gain +0.61 (95 % CI +0.39 to
   +0.83) over COCO weights, which pinned survivors in only 1 of 13 runs.
2. **Pins are clean in count, not yet in precision.** 0 duplicates everywhere, 0.18-0.50 m error; 1-3 false pins
   on obstacle look-alikes remain in some runs (cell means 0.62-1.00 for the fine-tuned conditions).
3. **The own selector matches explore_lite but does not beat it** at this sample size, and unlike
   explore_lite it ends the mission itself.
4. **Close-range verification, as tuned, hurts recall** (-0.22, CI -0.41 to -0.03): too few close views before
   the mission ends. Lowering `verify_views` or allowing more visits is the next thing to test.
5. **Localization is the main weakness.** No recovery from a kidnap (0/5) and large SLAM errors in the narrow
   doors of the rooms world (ATE up to 1.5 m mean for some conditions).

## Ablations (week 11)
Only those inside the sweep were run, because each new sweep costs many hours on this machine:
verification on/off and fine-tuned vs pretrained detector (`docs/results/paired.md`). Planned but not run:
hazard weight, gain radius, `min_gain_m2`, verification thresholds.

## Engineering lessons
- Long unattended simulation batches need a hard reset between runs: leaked ROS nodes, Fast DDS shared-memory
  segments and hung map savers cost most of the sweep's 26 hours (`docs/PHASE8.md`).
- slam_toolbox only grows the map after translation; Nav2 must start after the open-loop wake-up drive.
- A recovery trigger based on map->odom jumps cannot detect a kidnap: odometry does not jump.

## What next
Global relocalization (scan-to-map search, then re-seed SLAM); verification thresholds; more seeds (resume
`scripts/sweep.sh 1 10`); a hazard/doorway-aware SLAM configuration for rooms.
