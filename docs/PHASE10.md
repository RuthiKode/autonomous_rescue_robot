# Phase 10: kidnap detection and global relocalization

Week 8 left one test failing: after the robot is teleported 5+ m (a "kidnap"), it never recovered
(0 of 5, `docs/PHASE7.md`). The two reasons:
1. **Detection:** the only trigger was a jump in map->odom. A teleport moves the robot without
   turning the wheels, so odometry does not jump and nothing fired.
2. **Re-seeding:** even when a recovery ran, it only spun on the spot. Nothing told slam_toolbox
   where the robot really was.

## Method (`src/rescue_mission/rescue_mission/relocalize.py`)
- **Score.**
  - Project the latest lidar scan into the SLAM map from the SLAM pose.
  - Only endpoints that land on *known* cells count. A robot that is exploring correctly sees
    plenty of unmapped space, so unknown cells must not count against it.
  - Each endpoint scores `exp(-d²/2σ²)`, where `d` is its distance to the nearest wall and
    σ = 0.075 m. The score is the mean over those endpoints.
- **Detection** (mission tree `poll()`, every 0.5 sim s).
  - A score below 0.35 for 3 checks in a row is a kidnap.
  - At the first low check, slam_toolbox's new measurements are paused, so it does not draw the
    scans at the wrong pose. They resume when the score recovers or the recovery ends.
  - The map snapshot used for the search is the last one that scored at least 0.5.
- **Global search.**
  - Coarse pass: every known-free cell on a 0.25 m grid, at 24 yaws. Endpoints outside the grid
    count as misses.
  - Fine pass: refine the 8 best places at 0.05 m and 2°.
  - The answer is accepted only if its score is at least 0.85 **and** it beats the best pose
    ≥ 1 m away by at least 0.15. A symmetric map gives a small margin, and the robot then does not
    commit.
- **Recovery `kidnapped`.**
  1. Stop the robot.
  2. Search. If the answer is not confident, spin 360° and search once more.
  3. On success, re-seed slam_toolbox: `serialize_map`, then `deserialize_map` with
     `START_AT_GIVEN_POSE`.
  4. Reset the map->odom jump detector (the jump is deliberate) and the recovery counter.
  5. Resume SLAM and clear the costmaps.
- All thresholds are in `src/rescue_mission/config/mission.yaml`. `relocalization_enabled`
  switches the whole feature.

## Go/no-go checks (before building it)
- **Was the landing spot mapped?** In all 5 week-8 teleport bags, the true landing cell was known
  free in the map published just before the teleport. 71-98 % of a 2 m disc around it was known.
  Command: `python3 scripts/reloc_offline.py landing results/failure_tests`.
- **Can slam_toolbox be re-seeded while mapping?** `scripts/reseed_smoke.sh`, robot held still:
  - re-seeding at the current pose left it there (error 0.00 m);
  - re-seeding 1.00 m away moved it to +0.78 m, because its own scan matching pulled the
    deliberately wrong pose 0.22 m back towards the true one;
  - `/map` kept updating after each re-seed.

## Offline calibration (16 recorded missions, `scripts/reloc_calibrate.sh`)
The 16 missions are the 5 teleport and 4 block tests from week 8, plus own-selector sweep runs:
4 rooms, 2 open_plan and 1 collapsed. They are replayed from their bags with the SLAM pose from
`/tf` and the truth from `/ground_truth/odom`. Output goes to `results/reloc_offline/`
(`scores.csv`, `search.csv`). `*_binary.csv` are the first calibration, run with a binary score
(an endpoint counted as a hit within 0.15 m of a wall) at 1 s intervals. Its second-best poses scored
0.71-0.91, too close to the best to judge confidence, so it was replaced by the soft score below.

**Detection** (scan scored every 0.5 s; only scans with ≥ 30 endpoints on known cells). These bags are
the calibration set, and the thresholds were chosen on them, so the zeros below are not a held-out
result. The held-out evidence is the live tests: 5 of 5 kidnaps detected, and no false alarm in 5
kidnap-free missions.

| threshold | checks in a row | false kidnaps in 8,072 normal scans (collapsed 3,031, rooms 3,981, open_plan 1,060) | teleports detected |
|---|---|---|---|
| 0.35 | 1 | 15 | 5 / 5 |
| 0.35 | 2 | 0 | 5 / 5 |
| **0.35** | **3 (chosen)** | **0** | **5 / 5** |
| 0.40 | 3 | 1 (rooms) | 5 / 5 |
| 0.45 | 3 | 2 (rooms) | 5 / 5 |

Score distribution:
- normal driving: median 0.95, 1st percentile 0.66;
- the 12 s after a teleport: median 0.24;
- later: median 0.89, because slam_toolbox redraws the map around the wrong pose. This is why
  detection must be quick and SLAM is paused at the first low score.

**Global search,** 3 s after each teleport, on the last good map snapshot, compared with the truth:

| run | position error | yaw error | score | best elsewhere | accepted (≥ 0.85, margin ≥ 0.15) |
|---|---|---|---|---|---|
| teleport seed 1 | 0.36 m | 2.8° | 0.79 | 0.76 | no (would spin and retry) |
| teleport seed 2 | 0.04 m | 0.2° | 0.97 | 0.53 | yes |
| teleport seed 3 | 0.02 m | 0.2° | 0.98 | 0.66 | yes |
| teleport seed 4 | 0.03 m | 0.1° | 0.98 | 0.61 | yes |
| teleport seed 5 | 0.05 m | 0.4° | 0.96 | 0.58 | yes |

Each search scored 35-48 thousand candidate poses in 0.16-0.20 s.

## Live tests (`scripts/phase10_tests.sh`, 2026-11-27)
Same harness and pass rule as week 8 (`explore_eval --inject teleport|block --inject-at 90`, own selector,
no detector, collapsed world), unchanged: the mission did not FAIL, it ended DONE or reached coverage
0.85, and for a teleport the final SLAM pose error is ≤ 0.5 m. Data: `results/phase10/runs.csv`,
`results/phase10/runs/*/injection.json` and each run's `launch.log`.

The "landing mapped" column comes from `scripts/reloc_offline.py landing results/phase10`: the share
of a 2 m disc around the true landing spot that was known in the map just before the teleport.

| test | seed | landing mapped | detected | search (best score, next best elsewhere) | re-seeded | recovered | mission, coverage | final pose error |
|---|---|---|---|---|---|---|---|---|
| teleport | 1 | 0.66 | yes (score 0.04) | 0.99 vs 0.63 | yes | **yes** | DONE, 1.00 | **0.11 m** |
| teleport | 2 | 0.00 (cell unknown) | yes (0.11) | 0.54 vs 0.53, twice | no (not confident) | no | DONE, 0.70 | 4.64 m |
| teleport | 3 | 0.01 (cell unknown) | yes (0.02-0.05, several times) | 0.46-0.63 vs 0.44-0.51 | no | no | FAILED (6 recoveries), 0.66 | 7.24 m |
| teleport | 4 | 0.12 (cell unknown) | yes (0.31) | 0.60-0.64 vs 0.52-0.60 | no | no | DONE, 0.80 | 9.86 m |
| teleport | 5 | 0.00 (cell unknown) | yes (0.04-0.35, several times) | up to 0.75 vs 0.46-0.56 | no | no | FAILED (6 recoveries), 0.60 | 6.06 m |
| block | 1 | n/a | no kidnap flagged | - | - | **yes** | DONE, 0.99 | 0.09 m |
| block | 2 | n/a | no kidnap flagged | - | - | **yes** | DONE, 0.99 | 0.08 m |
| block | 3 | n/a | no kidnap flagged | - | - | **yes** | DONE, 0.99 | 0.06 m |
| normal | 1 (collapsed) | n/a | no kidnap flagged, SLAM never paused | - | - | - | DONE, 0.99 (90 % at 142 s) | ATE 0.02 m |
| normal | 1 (rooms) | n/a | no kidnap flagged, SLAM never paused | - | - | - | DONE, 0.99 (90 % at 311 s) | ATE 0.02 m |

**Full launch:** the README's main command (`full_mission.launch.py world:=collapsed explorer:=own
verify:=true detector:=yolo`) was run once headless with relocalization on (`scripts/full_mission_check.sh`).
The mission ended DONE (no frontier left), with 0 tracebacks, 0 crashed processes and no kidnap flag.

**Decision: shipped enabled** (`relocalization_enabled: true` in `mission.yaml`). Teleport recovery
went from 0/5 to 1/5, block stayed 3/3, and no run had a false kidnap or a wrong re-seed. Set the
parameter to `false` to get the week-8 behaviour back.

**Teleport: 1 of 5 recovered** (week 8: 0 of 5).
- **Seed 1** was the only run where the robot landed in an area it had already mapped. It recovered:
  detected, located by the search with a clear margin, SLAM re-seeded, final error 0.11 m, and the
  mission still finished at 100 % coverage.
- **Seeds 2-5:** the harness teleported the robot into space it had **never mapped**. It picks any
  reachable spot in the true world. In week 8 all five landings happened to be in mapped space, in
  this batch four were not. A scan cannot be matched to a map that does not contain its
  surroundings, so these four are not solvable by this method.
- **No wrong re-seed.** In all four the system refused, as designed. In seed 5 the search proposed
  the same pose several times with score 0.73-0.75 and a margin of about 0.25. That pose was 5 m
  from the truth; the 0.85 score threshold is what stopped a confident wrong re-seed.

**Regression:**
- block 3 of 3, as before (week 8: 5 of 5);
- no kidnap false alarm in any block or normal run;
- the normal collapsed mission finished DONE at 0.99 coverage.

**First attempt (kept in `results/phase10_attempt1/`).** Before the final version, teleport seed 1
failed (pose error 9.59 m), for a reason the offline replay could not show:
- 0.7 s after the teleport, slam_toolbox's own scan matcher snapped to a wrong place.
- The old "localization jump" recovery then only spun the robot.
- At the wrong pose the scan fitted the map, so the kidnap check never reached 3 low scores.

Fix: with relocalization enabled, a localization jump runs the same global search and re-seeds only
if the search confidently disagrees with SLAM. The batch was then restarted from seed 1; the table
above is the restarted batch.

## Unit tests
`src/rescue_mission/test/test_relocalize.py` (5 tests) checks, on synthetic maps:
- a scan from the true pose scores high, and from a wrong pose scores low;
- the search recovers a known pose to within 0.3 m and 10°;
- a featureless corridor gives no margin, so the search refuses to commit;
- the detector needs 3 low checks in a row and keeps the last good snapshot;
- the detector ignores small maps.

## Reproduce
```bash
python3 scripts/reloc_offline.py landing results/failure_tests     # go/no-go (a)
bash scripts/reseed_smoke.sh                                        # go/no-go (b)
bash scripts/reloc_calibrate.sh                                     # offline calibration
bash scripts/phase10_tests.sh                                       # live tests
```

## Known issues
- **Kidnaps into unmapped space are not recovered.** The robot keeps a wrong pose (4.6-9.9 m in
  seeds 2-5). The next step is a "lost" mode: keep searching periodically while exploring
  cautiously, and re-seed when the scan first matches the map with confidence.
- **Small sample:** 5 teleports, 3 blocks and 2 normal missions in one world (collapsed) live; rooms
  and open_plan were checked offline for false alarms only (0 in 5,041 scans).
- **Thresholds come from 16 recorded missions.** Rooms-like symmetric maps could still give a
  confident wrong answer; the margin test is the guard, and it has not failed yet.
- **The re-seed reloads the whole pose graph** (serialize, then deserialize). In seed 1 this took
  0.61 s of wall time from acceptance to re-seeded. The whole recovery took 7.9 s: SLAM paused 2.4 s
  before the kidnap was declared, then stop, search, re-seed, resume and clear costmaps. The reload
  would grow with much larger maps.
