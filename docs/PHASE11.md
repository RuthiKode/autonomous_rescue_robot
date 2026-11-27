# Phase 11: lost mode (kidnap into unmapped space)

Phase 10 recovered a kidnap only when the robot landed somewhere it had already mapped. In the live
test, 4 of 5 teleports landed in unmapped space and none of those recovered. Two of the missions even
ended FAILED, because the kidnap check fired again and again until 6 recoveries were reached in a row
(seeds 3 and 5).

## Method
**Entering lost mode.** When the phase-10 search finds no confident pose (two attempts, one spin in
between), the robot does not keep going on the wrong pose:
1. **Save the old map.** slam_toolbox's pose graph was saved at the first low score
   (`/tmp/rescue_prekidnap_posegraph`), while SLAM was already paused, so the save comes before the
   wrong scans. A snapshot of the old map is kept for searching.
2. **Reset SLAM** (`/slam_toolbox/reset`) to a fresh map. The robot backs up 0.4 m and spins once so
   the new map gets its first nodes. Without that, slam_toolbox adds nothing and the explorer sees no
   frontiers.
3. **Keep exploring** on the fresh map. While lost, "no frontier" in the first 120 s triggers another
   wake-up move instead of ending the mission.
4. **Turn off the kidnap and jump checks.** They compare against the fresh map, which has an unknown
   offset to the old one, so they mean nothing. This also removes the re-trigger loop that made
   seeds 3 and 5 FAIL.

**Searching while lost.** Every 2 s the current scan is searched against the *saved* map.
- An answer counts only if it is confident: score ≥ 0.85, margin ≥ 0.15 (the phase-10 thresholds,
  unchanged) and ≥ 40 endpoints on known cells.
- **A single confident answer is not enough.** Two confident answers must be separated by real robot
  motion (≥ 1 m or ≥ 45° by odometry), and the change between the two matches must equal the change
  in odometry (within 0.3 m and 10°). A wrong place that merely looks like the scan (an alias) does
  not move with the robot, so it fails this check. Searching again from a stopped robot would not
  help: it sees the same view and gets the same alias.
  Code: `relocalize.motion_consistent`.
- **On a confirmed match:** stop, carry the match forward by the odometry since it was taken, reload
  the saved pose graph there (`deserialize_map`, `START_AT_GIVEN_POSE`), and leave lost mode. The area
  mapped while lost is dropped and mapped again correctly.

All limits are in `src/rescue_mission/config/mission.yaml` (`lost_*`); `lost_mode_enabled` switches the
feature.

## Go/no-go checks
**(a) Offline, on all 10 unrecovered teleport bags** (`scripts/lost_offline.sh`): phase-10 seeds 2-5,
the phase-10 first-attempt seed 1, and week-8 seeds 1-5. These robots all drove for minutes on a wrong
pose, which is exactly the condition lost mode runs in.
- After each teleport, a search every 2 s against the map from just before it: 1,002 searches in all.
- A search counts as *possible* if the true cell was known free in that map; otherwise a correct
  answer did not exist.
- The check was first run with the search's default `min_known=20`, then re-run with 40, the value
  the live system uses (`lost_min_known`). Both are reported.
- Data: `results/lost_offline/searches_min20.csv` and `searches.csv` (40), with summaries
  `summary_min20.log` and `summary_min40.log`.

| | searches | confident (min 20 / **min 40**) | correct when confident (20 / **40**) |
|---|---|---|---|
| a correct answer was possible | 619 | 520 / **468** | 518 / **466** |
| no correct answer existed | 383 | 25 / **5** (all wrong by definition) | - |

Wrong single confident answers: 27 at 20, **7 at 40**. Applying the motion check over the confident
answers of each bag (table below at 20):

| bag | match possible | motion-checked accept | correct |
|---|---|---|---|
| phase-10 seed 2 | from +99 s | +104 s | yes (agrees with odometry within 0.02 m) |
| phase-10 seed 3 | from +79 s | +82 s | yes (0.05 m) |
| phase-10 seed 4 | from +32 s | +36 s | yes (0.10 m) |
| phase-10 seed 5 | never (mission FAILED at 164 s) | none | - |
| phase-10 attempt-1 seed 1 | from +26 s | +30 s | yes (0.12 m) |
| week-8 seeds 1-5 | from +3 to +22 s | +5 to +24 s | yes, all 5 |

**At 20: 9 of 9 accepted matches were correct, and 0 wrong.** None of the 27 wrong single answers
passed the motion check.

**At 40 (live setting): 8 bags reached an accept, all correct, 0 wrong.**
- Phase-10 seed 3 got only one confident match in the 7 searches where a match was possible, so it
  was never confirmed.
- Phase-10 attempt-1 seed 1 was confirmed later, at +117 s instead of +30 s.
- The stricter setting gives fewer false candidates (7 instead of 27), and is a little slower and
  catches slightly fewer. The motion-check limits were set before this run and not tuned on it, and the 0.85/0.15
thresholds are unchanged from phase 10.

**(b) Live smoke test of save → reset → reload** (`scripts/lost_cycle_smoke.sh`, robot moved, then
held still):
- before: 30.1 m² known;
- after reset: 3.8 m², a fresh map;
- after reloading the saved graph at the earlier pose: 35.2 m² known, pose error 0.00 m.

The first try of this smoke test left the robot at the origin (4 m² before and after), so it could not
tell whether the reset worked. It was re-run with the robot driven first.

## Live tests (`scripts/phase11_tests.sh`, 2026-11-27)
The harness and pass rule are the same as in weeks 8 and 10, unchanged: collapsed world, own selector,
no detector, injection at 90 s, 600 s limit. A teleport passes if the mission did not FAIL, it ended
DONE or reached 0.85 coverage, and the final pose error is ≤ 0.5 m.
- Final teleports: `results/phase11/`, run with `TELEPORT_ONLY=1`.
- Block and normal runs: `results/phase11_attempt3/` (see "How we got here" for why that is valid).
- "Landing mapped" is the share of a 2 m disc around the true landing spot known just before the
  teleport (`scripts/phase11_summary.sh`).

| test | seed | landing mapped | path | time lost | recovered | mission, coverage | final pose error (peak) |
|---|---|---|---|---|---|---|---|
| teleport | 1 | 0.69 | direct re-seed (score 0.94 vs 0.67) | - | **yes** | DONE, 0.99 | **0.00 m** (5.75) |
| teleport | 2 | 0.12 (cell unknown) | lost mode, 2 candidates held, then confirmed (0.01 m, 2°) | 50 s | **yes** | DONE, 0.99 | **0.01 m** (11.44) |
| teleport | 3 | 0.37 | lost mode, 1 held, confirmed (0.03 m, 3°) | 31 s | **yes** | DONE, 0.99 | **0.05 m** (10.54) |
| teleport | 4 | 0.78 | lost mode, 2 held, confirmed (0.02 m, 0°) | 175 s | **yes** | DONE, 0.99 | **0.13 m** (9.62) |
| teleport | 5 | 0.00 (cell unknown) | lost mode, 2 held, confirmed (0.01 m, 2°) | 53 s | **yes** | DONE, 1.00 | **0.07 m** (9.36) |
| block | 1 | n/a | no kidnap path triggered | - | **yes** | DONE, 0.99 | ATE 0.03 m |
| block | 2 | n/a | no kidnap path triggered | - | **yes** | DONE, 1.00 | ATE 0.03 m |
| normal | 1 | n/a | no kidnap path triggered | - | - | DONE, 1.00 (90 % at 136 s) | ATE 0.03 m |

**Teleport: 5 of 5 recovered** (week 8: 0/5; phase 10: 1/5).
- Four went through lost mode. Each robot was lost for 31-175 s (wall time), and every final pose
  error was ≤ 0.13 m.
- **No wrong re-seed in this batch.** In lost mode, 7 confident candidates were held until a second
  match after real motion confirmed them.
- The landing spots differ from phase 10: the robot's route before the teleport differs from run to
  run, so the same seed does not land in the same place.

**Regression:**
- Block 2 of 2, and the normal mission finished DONE at 1.00 coverage. None of the three runs ever
  paused SLAM or entered the kidnap path.
- **Rooms normal mission, final build** (`scripts/phase11_extra.sh`; a false kidnap is now more
  expensive, since it ends in a SLAM reset, and rooms is where it is most likely). Result: DONE, no
  pause, kidnap or lost line, so none of the new code ran. Coverage was only 0.79, against 0.99 for
  phase 10's rooms run. That is within the rooms spread from before any relocalization code existed:
  0.50-0.99 over the 12 own-mission rooms runs of the sweep.
- **Full-mission command** with lost mode on (`scripts/full_mission_check.sh`): DONE, 0 tracebacks,
  0 crashed processes.

## How we got here: three live attempts before the final batch (all kept)
Each attempt found a real problem that the offline replay could not show. The final batch above runs
the code with all three fixes; the earlier data stays in `results/phase11_attempt{1,2,3}/`.
1. **Attempt 1 — an empty fresh map ended the mission.**
   - Seed 4 entered lost mode and the mission ended DONE at once: the fresh map had no frontiers yet.
   - Fix: after the reset, back up 0.4 m and spin; while lost, an empty frontier answer within 120 s
     triggers another wake-up move instead of DONE.
   - Seed 3 of this attempt never started: Gazebo segfaulted at startup (exit -11), the same simulator
     flake as week-8 block seed 3.
2. **Attempt 2 — slam_toolbox stopped mapping after the reset.**
   - The map stayed at 3.5 m² for 2 minutes while the robot drove (`scripts/map_growth.py`).
   - Cause, tested live with `scripts/reset_growth_smoke.sh`: the reset's `pause_new_measurements:
     false` does not undo the earlier pause toggle. The map stayed at 4.0 m² after driving, and grew
     to 31.5 m² after one more toggle.
   - Fix: once the reset completes, toggle the pause off.
3. **Attempt 3 — a confident wrong re-seed through the phase-10 direct path.**
   - Lost mode recovered 1 of 1 (seed 2, 0.05 m) and the direct path 2 of 3 (seeds 1 and 3); seed 4
     did not start.
   - In seed 5, the direct path first re-seeded correctly, at (4.25, -1.65) against a true
     (4.2, -1.7). About 9 s later the score dropped again. The re-seed had reloaded the graph as it
     was at accept time, which still held a node or two SLAM added at the wrong pose right after the
     teleport, and SLAM then loop-closed onto them.
   - The second search then accepted a single confident alias (score 0.90, margin 0.21; truth 10 m
     away) and looped: 38 re-seeds, final error 10.75 m.
   - Fixes:
     - the direct re-seed now reloads the graph saved at the first low score;
     - a kidnap within 90 s of a re-seed no longer trusts a single match and goes to lost mode,
       which needs the motion-confirmed pair.
   - Seed 4 of this attempt hit the Gazebo startup segfault again.
   - The block and normal runs of attempt 3 are the regression results above. The fixes only change
     code that runs after a low scan score, and none of those three runs ever had one (0 pause,
     kidnap or lost lines in their logs). So they are valid for the final build.

## Reproduce
```bash
bash scripts/lost_offline.sh          # go/no-go (a): searches on the unrecovered teleport bags
bash scripts/lost_cycle_smoke.sh      # go/no-go (b): save, reset, reload on a live slam_toolbox
bash scripts/reset_growth_smoke.sh    # does SLAM map after a reset? (attempt-2 finding)
bash scripts/phase11_tests.sh         # live tests (TELEPORT_ONLY=1: teleports only)
bash scripts/phase11_summary.sh       # per-run landing share, time lost, verdict
```

## Known issues
- **Small sample.** 5 teleports in one world (collapsed). The 5/5 is one batch, after three attempts
  that each changed the code. The fixes respond to failures seen in those attempts, so this batch is
  the first clean test of the final code, not a long track record.
- **The direct path still trusts a single match on the first kidnap of an episode.** Attempt 3 showed
  that an alias can pass 0.85 / 0.15. Only the repeat guard (a second kidnap within 90 s) catches
  that case. Routing every re-seed through the motion check would be safer, at the cost of slower
  recovery. *Done in phase 12 (`docs/PHASE12.md`).*
- **The motion check accepted rotation-only pairs.** With the turn alternative on (45°), 3 of the 8
  offline accepts had 0.00 m of travel between the two matches. A 360° lidar sees the same scan,
  rotated, after a turn on the spot, so those accepts tested nothing (all 8 were correct anyway).
  *Phase 12 requires translation.*
- **The repeat guard was never triggered live** in this batch. *It fired, correctly, in phase 12
  (teleport 2).* No final-batch run had a second kidnap soon after a
  re-seed. It is covered by unit tests only (`test/test_reloc_guard.py`): a kidnap 30 s after a
  re-seed skips the single-match search, one 120 s after searches as usual, and the guard is off
  without lost mode.
- **The map explored while lost is thrown away** when the saved map is reloaded, and is explored
  again (lost 31-175 s here).
- **No timeout on lost mode.** A robot that never sees mapped space again explores the fresh map
  until the mission ends, with its pose in a different frame from the saved map.
- **Gazebo segfaults at startup** sometimes (2 of 20 phase-11 mission launches). These runs are reported as
  `startup_timeout`, and the run is repeated in the next batch.
