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
- Added kidnap detection and global relocalization (scan-to-map scoring, grid search over the SLAM map,
  slam_toolbox pose-graph re-seed) plus a "lost mode" for kidnaps into unmapped space. In lost mode,
  SLAM restarts and the robot keeps exploring; a match is accepted only when two confident matches
  agree with the odometry between them. Recovered 5 of 5 live kidnapped-robot tests (one batch, one
  simulated world; final error ≤ 0.13 m), up from 0 of 5.
- Built a seeded evaluation harness (coverage, map IoU, ATE, pin precision/recall, failure injection) and
  generated every results table from CSV; 221 automated tests.

## Sources
| claim | number | source |
|---|---|---|
| recall gain from fine-tuning | +0.611, CI [+0.393, +0.829], 12 pairs | `docs/results/paired.md` |
| pretrained pins | precision defined in 1 of 5 collapsed runs, n/a in rooms/open_plan (no pins) | `docs/results/summary.md` |
| duplicate pins | 0.00 in every cell with pins | `docs/results/summary.md` (duplicate rate) |
| 60 missions | "60 runs with status ok" | `docs/results/summary.md` header |
| pin error | 0.18-0.50 m (cell means) | `docs/results/summary.md` (pin error) |
| missions finish themselves | DONE share 0.75-1.00, own conditions | `docs/results/summary.md` (mission DONE) |
| kidnap detection, live (held out) | 5 of 5 teleports detected; 0 kidnap flags in 3 block + 2 normal missions (collapsed, rooms) | `docs/PHASE10.md`, `results/phase10/runs/*/launch.log` |
| kidnap detection, calibration set (not held out: thresholds chosen on it) | 5 of 5 detected, 0 flags in 8,072 normal scans (3 worlds) | `docs/PHASE10.md`, `results/reloc_offline/scores.csv` |
| kidnapped robot recovered | 5 of 5, final error 0.00-0.13 m, 4 via lost mode (final batch) | `docs/PHASE11.md`, `results/phase11/runs/*/injection.json` |
| progression | week 8: 0/5; phase 10: 1/5; phase 11: 5/5 | `docs/PHASE7.md`, `docs/PHASE10.md`, `docs/PHASE11.md` |
| motion check offline (live setting, min 40 known endpoints) | 8 of 8 accepted matches correct; 0 of 7 wrong single answers accepted (min 20: 9 of 9, 0 of 27) | `docs/PHASE11.md`, `results/lost_offline/searches.csv`, `searches_min20.csv` |
| blocked corridor | 5 of 5 recovered | `docs/PHASE7.md`, `results/failure_tests/runs/*/injection.json` |
| tests | 221 tests, 0 errors, 0 failures, 2 skipped | `colcon test-result` (`scripts/run_all_tests.sh`), 2026-11-27 |

## Do not claim
- That the own selector beats explore_lite (difference -35 s to 90 %, CI -113 to +42 s: not separated).
- That verification improves precision (-0.08, CI -0.28 to +0.12) — it lowered recall.
- That kidnap recovery never makes a wrong re-seed. Before the repeat guard, one live run re-seeded on a
  confident alias and looped (`docs/PHASE11.md`, attempt 3). The 5/5 is one batch of 5 in one world,
  after three attempts that each changed the code.
- "10+ seeds" (3-5 per cell were completed).
- Real-robot results (simulation only).
