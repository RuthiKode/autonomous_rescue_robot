# Phase 7 (plan week 8): Verify, Recover, scripted failures, full launch

Plan gate for week 8: *scripted failure tests pass in 5 consecutive runs, and one launch file brings up the whole system.*

**Result: the gate is partly met.** Blocked corridor: 5 of 5 recovered (with one re-run, see below). Teleport
(kidnapped robot): **0 of 5 recovered**. The full launch file works. The teleport failure is a real limit of the
stack (slam_toolbox in mapping mode cannot relocalize after a kidnap) and was not fixed.

## What was built
- Mission tree (priority selector, `rescue_mission/behaviours.py`): TimeLimit, Recover, Verify, Explore, Finish.
  - **Recover** triggers: `stuck` (clear costmaps, back up 0.3 m), `localization_jump` (map->odom step over
    0.5 m or 0.4 rad within 0.5 s: stop, spin a full turn, clear costmaps), `no_path` (clear costmaps once on the
    way to "unreachable"). FAILED after 6 recoveries in a row without reaching a goal.
  - **Verify**: drive to the nearest confirmed pin (or a candidate with 12+ hits), pick a viewpoint 1.5-2 m away with
    line of sight, dwell 6 s, at most 2 visits per pin. The tracker turns the close-range views into `verified` or
    `rejected`.
- `explore_eval --inject block|teleport --inject-at T`: a wall is spawned ahead of the robot, or the robot is moved
  5+ m to a reachable spot; `injection.json` records the verdict (recovered = mission not FAILED, and DONE or
  coverage >= 0.85, and for teleport a final pose error <= 0.5 m).
- `rescue_bringup full_mission.launch.py`: simulation, SLAM, wake-up drive, then Nav2, perception, mission (or
  explore_lite), optional Foxglove bridge. Arguments: world, headless, seed, detector, model, explorer, verify,
  foxglove, rviz.

## Reproduce
```bash
scripts/failure_tests.sh collapsed          # 10 runs, about 1.5 h; results/failure_tests/runs.csv
scripts/week8_followup.sh                   # block seed 3 re-run + one full-launch run with verification
```

## Scripted failure tests (collapsed world, seeds 1-5, own selector, injection at 90 s, no plateau stop)

| test | seed | recovered | mission end | coverage | goals / recoveries after | final pose error (m) |
|---|---|---|---|---|---|---|
| block | 1 | yes | DONE | 0.99 | 3 / 1 | 0.05 |
| block | 2 | yes | DONE | 1.00 | 5 / 2 | 0.09 |
| block | 3 | yes (re-run) | DONE | 1.00 | 5 / 1 | 0.03 |
| block | 4 | yes | DONE | 0.99 | 2 / 1 | 0.14 |
| block | 5 | yes | DONE | 0.98 | 3 / 1 | 0.03 |
| teleport | 1 | **no** | FAILED (6 recoveries in a row) | 0.80 | 5 / 6 | 10.53 |
| teleport | 2 | **no** | DONE (no reachable frontier) | 0.83 | 11 / 8 | 7.45 |
| teleport | 3 | **no** | DONE (no reachable frontier) | 0.73 | 11 / 2 | 6.01 |
| teleport | 4 | **no** | DONE (no reachable frontier) | 0.67 | 2 / 1 | 8.44 |
| teleport | 5 | **no** | DONE (no reachable frontier) | 0.91 | 0 / 1 | 5.61 |

(Numbers are copied from `runs/*/injection.json` and from `results/failure_tests.log` and
`results/week8_followup.log`.)

**Block seed 3:** the first attempt did not test anything: the simulator crashed (gz exit code -11) while the
mission was still in STARTING, so the wall was injected into a dead stack and the run ended at the time limit with
coverage 0.14. It was re-run with the same arguments (the second run is the one in the table; the first row is
still in `runs.csv`, marked by its coverage of 0.14).

**Teleport:** why it fails. The simulator moves the robot but the wheel odometry does not jump, so the EKF and
slam_toolbox keep believing the robot is at the old place. map->odom does not step, so the `localization_jump`
trigger never fires; the next scans are matched against the wrong place of the map, the pose error stays at 5-10 m
and the map is corrupted from there on. The tree then sees goals fail or frontiers vanish and ends DONE or FAILED
with 0.67-0.91 coverage and a map that is not usable. Real recovery needs global relocalization (scan-to-map
search on the saved map, or AMCL / slam_toolbox localization mode with an initial pose), then re-seeding SLAM.
That was not built; a trigger that only reacts to the map->odom step cannot detect this failure.

## Full launch
`ros2 launch rescue_bringup full_mission.launch.py world:=collapsed seed:=1 headless:=true detector:=yolo model:=<best.pt> explorer:=own verify:=true`
ran headless. `/mission/state` went STARTING -> EXPLORING -> VERIFYING (pins 1, 3, 5) -> EXPLORING and ended DONE
("no frontier cluster above the minimum size is left") about 230 s after STARTING; `/explore/explored_area_m2`
grew from 71 to 172 m2 (the last value held after DONE). `foxglove:=true` was not run (foxglove_bridge is not
installed in this environment).

## Deviations and known issues
1. Teleport gate not met (0 of 5), see above. Reported as measured; not tuned.
2. One simulator crash (gz exit -11) cost one run; the simulator process dies now and then under load.
3. Block recovery was only tested with a wall 1.5 m ahead in collapsed; other worlds are not covered.
4. The pose error column of block runs is measured at the end of the run, after SLAM may have corrected itself.
