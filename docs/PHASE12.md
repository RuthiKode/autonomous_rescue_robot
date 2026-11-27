# Phase 12: every re-seed confirmed by motion

In v1.1.0, only lost-mode re-seeds had to pass the motion check (two confident matches, after the robot
has moved, that agree with the odometry between them). The direct path still re-seeded SLAM on one
confident match. Phase 12 routes every re-seed through the check, and closes a second gap found while
testing it: SLAM sometimes snaps to a wrong place on its own, before the kidnap detector fires.

## What changed (`behaviours.py`, `relocalize.py`, `mission.yaml`)

**1. Confirm by motion on the direct path** (`reloc_confirm_by_motion: true`):
- A confident first match is stored as a candidate, with the odometry pose at that moment.
- `ConfirmDrive` turns toward open space and drives `reloc_confirm_drive_m` (1.2 m) at 0.15 m/s.
  - It uses Nav2's DriveOnHeading. Its collision check runs on the local costmap in the odom frame, so
    it is safe while SLAM is paused.
  - If the robot covered less than `lost_min_motion_m` (1.0 m), a sideways second leg follows.
- A second confident match is required. The two matches must agree with the odometry between them
  (`motion_consistent`, 0.3 m and 10° tolerance).
- If the confirmed pose already agrees with SLAM, nothing is re-seeded. Otherwise SLAM is re-seeded from
  the pose graph saved before the kidnap.

**2. Translation is required.**
- The turn-only alternative is disabled (`lost_min_turn_rad: 4.0`, more than any turn the robot makes).
- Reason: the lidar sees 360°, so turning on the spot gives the same scan rotated. A rotation-only
  "confirmation" tests nothing.
- Offline evidence (v1.1.0 rule, 10 unrecovered teleport recordings, `results/lost_offline/summary_min40.log`):
  - 3 of the 8 accepted matches had **0.00 m** of odometry travel between the two matches;
  - 2 more had 0.40-0.43 m.
  - All 8 happened to be correct, but the check did not earn that.
- Translation-only rule on the same data (`summary_min40_translation.log`):
  - **7 accepts, all correct**, after 1.44-6.32 m of travel;
  - one recording that the old rule recovered has no translation-only accept.

**3. Jump verification** (`reloc_jump_verify_score: 0.7`):
- A localization jump (map->odom step) runs the same search and confirmation.
- When neither can confirm a pose, the scan is scored at SLAM's new pose against the last good map
  snapshot:
  - below 0.7: the jump is treated as a kidnap, and lost mode starts;
  - at or above 0.7: SLAM resumes as before.
- The repeat guard from phase 11 is unchanged: a kidnap within 90 s of a re-seed goes to lost mode.

**4. `kidnap_min_known` 30 → 20:** attempt 1 missed a kidnap whose scan had only 24-28 endpoints on
known cells.

### Why 0.7
On normal driving the scan scores a median of 0.95 at the SLAM pose, with a 1st percentile of 0.66
(`docs/PHASE10.md`). The two wrong snaps recorded in attempt 2 scored:

| Wrong snap | Score at the SLAM pose | Window | Source |
|---|---|---|---|
| seed 4 | 0.40-0.55 | about 10 s | live log |
| seed 5 | 0.51-0.62 | 20 s, while the recovery ran | `scripts/reloc_offline.py trace` on the bag |

After SLAM resumed at the wrong pose, the seed-5 score climbed to 0.8+ within 15 s, because SLAM was
writing the new scans into the map at that wrong pose. The check therefore has to happen before SLAM
resumes, not afterwards.

The margin is thin: 0.7 sits just above the normal 1st percentile. The one live verification in the
final batch scored 0.74 (seed 5, a correct loop closure).

## Live tests (`scripts/phase12_tests.sh`, 2026-11-27)
Same harness and pass rule as phases 7-11:
- **Pass rule:** mission not FAILED, final pose error ≤ 0.5 m.
- **Setup:** collapsed world, teleport at 90 s, hard reset between runs.

Final batch, with every change above (`results/phase12/`):

| Run | Result | Final error | Path |
|---|---|---|---|
| teleport 1 | **recovered**, DONE 0.99 | 0.04 m | direct search not confident → lost mode, confirmed (0.13 m, 3°), reloaded after 65 s |
| teleport 2 | **recovered**, DONE 0.99 | 0.01 m | lost mode after 60 s. 65 s later the scan stopped fitting (score 0.31): the **repeat guard fired live** for the first time, so the robot went back to lost mode instead of trusting a single match; second reload correct |
| teleport 3 | simulator start-up failure (`map_save_timeout` after 6 s; controller heartbeat lost, gz crashed on shutdown) | n/a | not a test of the code; kept in `runs.csv` |
| teleport 3, rerun | **recovered**, DONE 0.99 | 0.01 m | first match (-6.26, 0.65) was correct but the match after the 1.23 m drive was not confident → not trusted → lost mode, reloaded correctly |
| teleport 4 | **recovered**, DONE 0.99 | 0.11 m | direct: candidate → drive 1.23 m → second match agrees within 0.06 m → re-seeded after 20 s |
| teleport 5 | **recovered**, DONE 0.99 | 0.03 m | direct: confirmed within 0.00 m, 2° → re-seeded after 24 s; later a 3.58 m loop-closure jump, verified at 0.74 |
| block 1 | recovered, DONE 1.00 | 0.09 m | no relocalization code triggered |
| block 2 | recovered, DONE 0.99 | 0.05 m | no relocalization code triggered |
| normal mission | DONE, coverage 0.999 | n/a | one 0.61 m loop-closure jump: confirmed SLAM's own pose (0.08 m), no re-seed; cost about 32 s |

- **Teleports: 5 of 5 recovered**, after one simulator start-up failure was rerun. Block: 2 of 2. Normal
  mission: done.
- **Re-seed proof** (`scripts/reseed_proof.sh results/phase12`): `re-seeds: 6, unconfirmed: 0`. Every
  re-seed line follows a confirmation line.
- **Recovery time** (`scripts/reloc_times.sh`, from the kidnap flag to the first re-seed):

  | | v1.1.0 (`results/phase11`) | phase 12 |
  |---|---|---|
  | direct path | 3 s (one run) | 20-24 s (two runs) |
  | lost mode | 46-182 s | 60-65 s |

  The direct-path increase is the price of the confirmation drive.

This is still one batch of 5 teleports in one world.

## How we got here: three attempts (all kept)
1. **`results/phase12_attempt1/`** (stopped after 2 runs):
   - Seed 1: the confirm drive stopped at 0.68 m, so the candidate was rejected. The lost-mode reload
     was correct, but the final error was 0.55 m (late SLAM drift), so the run failed.
   - Seed 2: the kidnap was never flagged, because the scan had 24-28 known endpoints, under the 30
     needed.
   - Fixes: the sideways second leg, and `kidnap_min_known` 20.
2. **`results/phase12_attempt2/`**: teleport 2 of 5, block and normal not run.
   - Seeds 1 and 3 recovered.
   - Seed 2: the teleport left the robot physically stuck. Its true position stayed at (-3.25, -3.33)
     for 250 s, so this is a harness problem and no localization can fix it.
   - Seeds 4 and 5: SLAM snapped to a wrong place before the detector fired (localization jumps of
     1.3-1.7 m). The jump path could not confirm a pose and resumed SLAM at the wrong pose, ending with
     final errors of 10.06 m and 15.93 m.
   - Fix: jump verification. The batch was stopped before its block runs so that the final batch tests
     the new code.
3. **`results/phase12/`**: the final batch above.

## Reproduce
```bash
bash scripts/phase12_tests.sh                    # 5 teleports, 2 blocks, 1 normal (about 45 min)
SEEDS=3 TELEPORT_ONLY=1 bash scripts/phase12_tests.sh   # rerun one teleport seed
bash scripts/reseed_proof.sh results/phase12     # every re-seed confirmed?
bash scripts/reloc_times.sh results/phase11 results/phase12
python3 scripts/reloc_offline.py trace results/phase12_attempt2/bags/collapsed_confirm_teleport_seed5_* 3 40
```
To restore the v1.1.0 behaviour, set these in `mission.yaml`:
- `reloc_confirm_by_motion: false`
- `reloc_jump_verify_score: 0.0`
- `kidnap_min_known: 30`
- `lost_min_turn_rad: 0.785`

## Known issues
- The jump-verification threshold has a thin margin: 0.7 against a 0.66 1st percentile in normal
  driving. A correct loop closure on a poor scan could start lost mode needlessly. That costs time, not
  correctness: lost mode finds the robot again on the saved map.
- A correct single match can be thrown away when the match after the drive is not confident (teleport
  3 rerun). The robot then takes the slower lost-mode route.
- The confirm drive can be blocked (0.00 m, teleport 5's jump). The candidate is then dropped, and the
  next attempt starts over.
- The "pre-kidnap" graph saved at a localization jump can already contain a second or so of scans from
  the wrong pose.
- Lost mode had no timeout here (*added in phase 13, `docs/PHASE13.md`*), and the map explored while
  lost is still discarded.
- One world, one batch of 5 teleports.
