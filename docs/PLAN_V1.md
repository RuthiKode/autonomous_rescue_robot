# 6-hour plan: fix the kidnap weakness and close every loose end, then ship v1.0 to GitHub

## Context
The project is finished against the 12-week plan, but four things are open:
- the robot cannot recover after it is kidnapped (moved by teleport): 0 of 5 tests;
- the repository is messy and the last demo is unexplained:
  - `git status` lists more than 80 untracked result files;
  - the three `gui_*.sh` scripts are not committed;
  - the Gazebo window demo ended with exit code 4 and the reason is unknown;
- there is no LICENSE file;
- CI shows a Node 20 deprecation warning;
- nobody has built the repo from a fresh clone.

The user chose: **both** the relocalization fix and full polish, **6 hours**, and the **MIT** licence.
"Fool proof" means:
- every claim is measured;
- a fresh clone builds and its tests pass;
- `git status` is clean;
- CI is green;
- a tagged release exists.

### Why the kidnap fails (from `behaviours.py:131-153` and PHASE7)
- **Detection:** recovery is triggered by a jump in the map->odom transform. A teleport moves the robot without turning its wheels, so odometry never jumps and nothing triggers.
- **Re-seeding:** even when a recovery does run, the only step is "spin and re-scan". Nothing tells slam_toolbox where the robot really is.

### Key finding
The 11 failure-test bags (`results/failure_tests/bags/`) contain the data needed to build both parts offline, against ground truth, before any slow simulation run:
- `/scan`, `/map`, `/tf`, `/ground_truth/odom`;
- 5 of them include a real teleport.

## Timeline (6 h, each block has a hard stop)

### Block 1 (0:00-0:45): hygiene
1. **`.gitignore`:** ignore the run leftovers so they stop appearing as untracked:
   - `results/**/bags/`, `results/*.log`, `results/*.txt`;
   - the scratch experiment directories (`results/phase3_*`, `ab_*`, `repro_*`, `navtest*`, `depthcheck*`, `probe*`, `dataset_explore`, `explore_collect`, `unseen_worlds`, `smoke_verify`, …).

   Files already tracked stay tracked. Goal: `git status` shows only intended changes.
2. **Gazebo demo:** keep `scripts/gui_demo.sh`, delete `gui_check*.sh`.
   - Write the log into `results/gui_demo.log` instead of `/tmp`, because `/tmp` was wiped when WSL restarted.
   - Run it once, headless (same code path, much faster), with a 10-min limit, to find out what exit code 4 means: a mission ending normally, or a crash.
   - Document the answer in the script header.
3. **`LICENSE`:** MIT for this repo's code. Mention it in the README licence notes, keeping the AGPL note for Ultralytics.
4. **CI:** move `actions/checkout@v4` to `@v5` to remove the Node 20 deprecation warning.

### Block 2 (0:45-2:45): global relocalization, developed offline on the bags
**Go/no-go checks first, about 25 min:**
- **(a) Landing spot was mapped.** In each teleport bag, find the true landing spot and check that it lies in known-free cells of the `/map` taken just before the teleport. If it does not, scan-to-map search cannot work: stop and document why.
- **(b) The re-seed works on a running mapping node.** Smoke-test `serialize_map` then `deserialize_map(START_AT_GIVEN_POSE)` live on the running node. Check that map->odom moves and mapping continues.

**Score design:**
- Score only scan endpoints that land on *known* cells, and require a minimum number of them.
- Snapshot the map at the first low score, pause SLAM measurements, and search that snapshot.

**After a re-seed:**
- Reset the jump detector, so the deliberate map->odom jump does not trigger a recovery.
- Do not count the kidnap recovery toward `max_consecutive_recoveries`.

**Calibration:** use rooms-world sweep bags as well as collapsed ones, to measure false triggers.

Create a new module `src/rescue_mission/rescue_mission/relocalize.py` with pure numpy/scipy functions, so it can be unit-tested:
- `likelihood_field(map)`: distance transform of the occupied cells (scipy `distance_transform_edt`).
- `scan_match_score(field, scan, pose)`: the fraction of scan endpoints that land within `hit_dist_m` of a wall.
- `global_search(field, scan, free_cells)`:
  - coarse pass: every 0.25 m on free cells × 16 yaws;
  - fine pass: refine the top 5 at 0.05 m and 2°;
  - return the best pose, its score, and the margin over the second-best distinct pose.

**Detection** (in the mission's `poll()`, next to the existing jump check):
- Every `kidnap_check_s`, score the latest scan at the SLAM pose.
- Request a `kidnapped` recovery when the score stays below `kidnap_score` for `kidnap_checks` checks in a row.
- Only do this once the map has at least `kidnap_min_known_m2` of known area.

**Recovery `kidnapped`:**
1. Stop the robot and spin to get a full scan.
2. Run `global_search`.
3. If the score is at least `reloc_min_score` and the margin is at least `reloc_min_margin`:
   - call slam_toolbox `serialize_map`;
   - then call `deserialize_map` with `match_type=START_AT_GIVEN_POSE` at the found pose. The service is confirmed in `/opt/ros/jazzy/share/slam_toolbox/srv/DeserializePoseGraph.srv`.
4. Otherwise retry once after a short move. If that also fails, give up with the existing FAILED behaviour.

**Configuration:**
- All thresholds go in `src/rescue_mission/config/mission.yaml`.
- `relocalization_enabled` decides whether the feature ships on or off (see Block 3).

**Offline calibration** with a new script, `scripts/reloc_offline.py`, using rosbag2_py on the 11 bags:
- Score distribution on normal driving (all bags before the injection) vs right after the teleport. Pick `kidnap_score` so there are no false triggers on the normal data.
- Run `global_search` on post-teleport scans and compare the result with `/ground_truth/odom`, expressed in the map frame. Report how often it lands within 0.5 m.
- Write the results to `docs/img/phase10/` and a small table.

**Unit tests** (`src/rescue_mission/test/test_relocalize.py`), on a synthetic map:
- a perfect scan scores close to 1;
- a scan from a shifted pose scores low;
- `global_search` recovers a known pose;
- a symmetric map produces a low margin, so the search refuses to commit.

**Hard stop at 2:45.** If the offline search is not correct within 0.5 m in at least 3 of the 5 teleport bags, I stop working on it. In that case:
- the feature stays in the code with `relocalization_enabled: false`;
- the negative result is documented, and Block 3 runs only as a regression check.

### Block 3 (2:45-4:00): live tests
1. Rebuild and run all unit tests (`scripts/run_all_tests.sh`).
2. Run `scripts/failure_tests.sh` (existing, reused) for **teleport seeds 1-5**, with `hard_reset.sh` between runs. That is about 3-10 min per run, roughly 35 min in total.

   A run passes only by the existing rule in `explore_eval.py:85-101`, which is not changed: final pose error at most 0.5 m, and a mission that did not FAIL.
3. Regression check: **block seeds 1-3** plus **one normal mission**, about 20 min. They must not get worse (no false `kidnapped` triggers).
4. The feature ships on if teleport improves on 0/5 and nothing regresses; otherwise it ships off. Either way, the exact numbers are reported.

### Block 4 (4:00-5:00): docs
- **`docs/PHASE10.md`:** the method, offline calibration numbers, the live 5-run table (same columns as PHASE7), regressions, known issues and reproduction commands.
- **Update with the new result:** README (results, limitations), `docs/REPORT.md` (gate 8 row, findings, what next) and `docs/RESUME_CLAIMS.md` (a new bullet only if the result passes; the "do not claim" list updated either way).
- **README:**
  - add an animated GIF made from `docs/media/demo.mp4` (GitHub READMEs do not play mp4 files inline);
  - add a "Run with the Gazebo window" line pointing at `scripts/gui_demo.sh`.
- **`docs/PLAN.md`:** add a status line for this session.

### Block 5 (5:00-5:45): fresh-clone proof and release
1. Clone the **local repo** (committed files only) into `/tmp/clonecheck`, then:
   - run `scripts/fetch_third_party.sh`;
   - `colcon build`;
   - `colcon test` with simulation tests skipped.

   This proves the README quick start works from committed files only. Then delete `/tmp/clonecheck`.
2. Commit in logical pieces (hygiene / relocalization / docs), each with the trailer, and confirm no weights, bags or build directories are included (`git diff --cached --stat`).
3. Push `main`, wait for CI to pass, and read the result.
4. Tag `v1.0.0`, push the tag, and create a GitHub release (`gh release create`) with short notes. The notes list the measured results and the known limits.

### Block 6 (5:45-6:00): buffer and final check
- `git status` is clean.
- CI is green.
- The release page exists.
- Every number in the README and RESUME_CLAIMS can be traced to a file (spot-check 5).

## Critical files
- **New:**
  - `src/rescue_mission/rescue_mission/relocalize.py`
  - `src/rescue_mission/test/test_relocalize.py`
  - `scripts/reloc_offline.py`
  - `docs/PHASE10.md`
  - `LICENSE`
  - `docs/media/demo.gif`
- **Edited:**
  - `src/rescue_mission/rescue_mission/behaviours.py`: `poll()`, `plan_recovery()` and a new step that calls the SLAM services.
  - `src/rescue_mission/config/mission.yaml`
  - `.gitignore`
  - `.github/workflows/ci.yml`
  - `scripts/gui_demo.sh`
  - `README.md`, `docs/REPORT.md`, `docs/RESUME_CLAIMS.md`, `docs/PLAN.md`
- **Reused unchanged:**
  - `scripts/failure_tests.sh`, `scripts/hard_reset.sh`, `scripts/run_all_tests.sh`
  - the `explore_eval` pass rule
  - `scripts/render_demo.py`

## Verification
- Unit tests: counts as printed by `colcon test-result`.
- Offline: search accuracy against ground truth in the 5 teleport bags, and false-trigger count on the normal driving segments.
- Live: teleport 5 runs and block 3 runs, judged by the unchanged pass rule. The numbers come from `results/failure_tests/runs.csv` and the `injection.json` files.
- Fresh clone builds and its tests pass; CI is green on GitHub; `git status` is clean; the v1.0.0 release exists.
