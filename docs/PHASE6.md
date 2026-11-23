# Phase 6 (plan week 7): own frontier selector and the mission tree's Explore branch

Plan gate for week 7: *coverage-versus-time curves and completion times for both selectors, over the same
seeded runs.*

## What was built
- `rescue_msgs`: `SelectFrontier.srv` (blacklist in, best goal out) and `MissionState.msg`.
- `rescue_exploration`:
  - `frontiers.py` (pure numpy): frontier cells (free next to unknown), 8-connected clusters, a goal cell per
    cluster (nearest the centroid among cells 0.35 m from obstacles), unknown area within 3 m, hazard area near
    the goal, and the score `gain / max(path, 1 m) - 2 * hazard_area`.
  - `selector_node.py`: service `/select_frontier`. Plans with Nav2's `ComputePathToPose` to candidates in order
    of a straight-line estimate, scores up to 6 that have a path, and returns the best. The caller owns the
    blacklist. `/frontiers` shows the candidates. Candidates with under `min_gain_m2` of unknown area are
    dropped so that exploration can end.
- `rescue_mission` (new): `py_trees` / `py_trees_ros` (installed from the ROS apt repository, as the plan says).
  The tree is a priority selector: TimeLimit, Explore, Finish (Recover and Verify were added in week 8, see
  `docs/PHASE7.md`). `logic.py` holds the decisions without ROS: blacklist, "a goal chosen again after being
  reached is blacklisted", no frontier for 3 answers in a row means DONE, 8 failed goals in a row means FAILED.
  `/mission/state` is published at every change.
- `explore_eval --explorer own` runs the same stack with the own selector instead of `explore_lite`;
  `--no-plateau` lets the mission end the run itself.
- Unit tests for the frontier maths, the mission logic and the tree structure (all in colcon test).

## Reproduce
```bash
scripts/phase6_runs.sh        # 18 runs: explore_lite and own selector, 3 worlds x 3 seeds, no detector (about 2.5 h)
scripts/phase6_complete.sh    # 9 runs of the own selector to completion (about 1.5 h)
python3 scripts/phase6_curves.py results/phase6 docs/img/phase6
```

## Coverage versus time (3 seeds per cell, `docs/img/phase6/coverage_vs_time.png`)

Both selectors ran on the same worlds and Gazebo seeds with the same stop rule (coverage plateau or 900 s).
Times are medians with [min-max]; coverage is the median over seeds.

| world | selector | time to 50 % (s) | time to 90 % (s) | time to 95 % (s) | coverage at 120 s | coverage at 240 s | final coverage |
|---|---|---|---|---|---|---|---|
| collapsed | explore_lite | 51 [46-55] | 185 [173-305] | 192 [185-310] | 0.75 | 0.99 | 0.999 |
| collapsed | own selector | 54 [51-143] | 206 [120-228] | 215 [128-261] | 0.52 | 0.97 | 0.999 |
| rooms | explore_lite | 115 [112-137] | 283 [280-287] (2 of 3 runs) | 469 (1 of 3) | 0.51 | 0.75 | 0.896 |
| rooms | own selector | 190 [142-226] | 302 [267-400] | 358 [302-450] | 0.33 | 0.75 | 0.881 |
| open_plan | explore_lite | 69 [69-122] | 273 [249-359] | 341 [293-390] | 0.64 | 0.83 | 1.000 |
| open_plan | own selector | 53 [45-55] | 217 [159-256] | 261 [221-264] | 0.60 | 0.92 | 1.000 |

Reading it: with three seeds the two selectors are not separated. The own selector is slower in the first two
minutes in collapsed and rooms (it goes for large unknown areas first and pays longer paths) and faster to 90 %
in open_plan (217 s against 273 s); in collapsed the ranges overlap and in rooms the own selector reached 90 %
in all three runs while explore_lite reached it in two of three (one explore_lite run stalled at coverage 0.50,
see known issues). The week-9 sweep has 10 seeds and the statistics for this question.

## Completion times (own selector, `min_gain_m2` 3, no plateau stop)

`explore_lite` has no completion signal in this setup (its "Exploration stopped" message appears when it is shut
down), so completion times exist only for the own selector; the common measure for both is time to 90 / 95 %.

| world | seed | mission | time to DONE (s) | coverage at DONE | time to 90 % (s) |
|---|---|---|---|---|---|
| collapsed | 1 | DONE | 207 | 0.997 | 129 |
| collapsed | 2 | DONE | 266 | 0.999 | 222 |
| collapsed | 3 | DONE | 314 | 0.993 | 260 |
| rooms | 1 | DONE | 623 | 0.859 | 364 |
| rooms | 2 | DONE | 484 | 0.995 | 304 |
| rooms | 3 | DONE | 379 | 0.724 | n/a |
| open_plan | 1 | DONE | 97 | 0.510 | n/a |
| open_plan | 2 | DONE | 415 | 1.000 | 177 |
| open_plan | 3 | DONE | 241 | 0.998 | 192 |

All nine missions ended DONE by themselves. **Three of them were premature**: open_plan seed 1 declared DONE at
coverage 0.51 and rooms seeds 1 and 3 at 0.86 and 0.72, each with "no remaining frontier is reachable". Cause:
the selector sent only the best 6 candidates (by straight-line distance) to the planner, and all of them could be
behind walls or in a stale costmap while reachable frontiers existed; the tree then accepted 3 empty answers
6 s apart. Fix, made after these runs (so these runs predate it): the selector keeps planning to further
candidates until 6 have a path (at most 16 planner calls), and "frontiers exist but none has a path" must repeat
8 times (16 s), with the costmaps cleared once halfway. The fix is covered by unit tests and is what the later
phases ran with; this table was not regenerated.

## Deviations and decisions
1. `py_trees` and `py_trees_ros` were installed with apt (ros-jazzy-py-trees, ros-jazzy-py-trees-ros,
   ros-jazzy-py-trees-ros-interfaces) after asking; they were missing.
2. The curves table comes from runs without the `min_gain_m2` floor, the completion table from runs with it.
   The floor only affects the tail (goals worth under 3 m2 of unknown area).
3. Hazard cells enter the score only as a penalty near the goal (`hazard_weight` 2 per m2), not along the path.

## Known issues
- Three seeds per cell: no statistical claim. The sweep in week 9 adds 7 more.
- explore_lite on rooms seed 3 stalled at coverage 0.50 (pose-jump or exploration failure, not diagnosed); that
  run is in the table (median) and pulls the explore_lite mean curve down.
- The first-two-minutes deficit of the own selector in collapsed and rooms was not investigated; a smaller
  `gain_radius_m` or a distance term in the score are the obvious things to try.
