# Resume claims and where each number comes from

Rule (docs/PLAN.md, week 12): every claim traces to a measured result. Numbers are copied from the files named.

## Suggested bullets
- Built an autonomous search-and-rescue robot in ROS 2 Jazzy / Gazebo Harmonic (slam_toolbox, Nav2,
  py_trees behaviour tree, YOLOv8n) that explores unknown worlds, maps them and pins survivors on the map.
- Fine-tuned YOLOv8n on auto-labelled synthetic frames from the simulator; raised survivor recall by 0.61
  over COCO weights (paired over 12 runs, 95 % CI +0.39 to +0.83).
- Wrote a survivor localizer (depth back-projection, map-frame tracking, multi-hit confirmation) that produced
  0 duplicate pins across 60 evaluated missions, with 0.2-0.5 m pin error.
- Built an own frontier selector and mission behaviour tree with recovery branches; missions finish on their
  own in 75-100 % of runs and recover from a blocked corridor in 5 of 5 scripted tests.
- Built a seeded evaluation harness (coverage, map IoU, ATE, pin precision/recall, failure injection) and
  generated every results table from CSV; 211 automated tests.

## Sources
| claim | number | source |
|---|---|---|
| recall gain from fine-tuning | +0.611, CI [+0.393, +0.829], 12 pairs | `docs/results/paired.md` |
| pretrained pins | precision defined in 1 of 5 collapsed runs, n/a in rooms/open_plan (no pins) | `docs/results/summary.md` |
| duplicate pins | 0.00 in every cell with pins | `docs/results/summary.md` (duplicate rate) |
| 60 missions | "60 runs with status ok" | `docs/results/summary.md` header |
| pin error | 0.18-0.50 m (cell means) | `docs/results/summary.md` (pin error) |
| missions finish themselves | DONE share 0.75-1.00, own conditions | `docs/results/summary.md` (mission DONE) |
| blocked corridor | 5 of 5 recovered | `docs/PHASE7.md`, `results/failure_tests/runs/*/injection.json` |
| tests | 211 tests, 0 errors, 0 failures, 2 skipped | `colcon test-result` (`scripts/run_all_tests.sh`), 2026-11-25 |

## Do not claim
- That the own selector beats explore_lite (difference -35 s to 90 %, CI -113 to +42 s: not separated).
- That verification improves precision (-0.08, CI -0.28 to +0.12) — it lowered recall.
- Recovery from kidnapping/teleport (0 of 5).
- "10+ seeds" (3-5 per cell were completed).
- Real-robot results (simulation only).
