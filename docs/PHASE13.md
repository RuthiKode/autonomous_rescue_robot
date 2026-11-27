# Phase 13: lost-mode timeout

Until v1.2.0, lost mode had no end. A robot that never saw mapped space again kept searching the saved
map until the mission ended, and its final state did not say it was lost.

## What changed
- **`lost_timeout_s`** (`behaviours.py` default 0 = search forever; `mission.yaml` **360 s**).
- **When it fires:** the robot has been lost longer than the timeout, and no recovery is running.
  The wait for a running recovery means a reload in progress always finishes, and the wake-up
  manoeuvre right after entering lost mode is not interrupted.
- **What the robot does then:**
  - stops searching the saved map;
  - makes the fresh SLAM map the mission map;
  - resumes kidnap and jump checks, now against the fresh map;
  - keeps exploring.

  Ending the mission as FAILED instead would throw away search time that a rescue robot can still
  use.
- **The final state says what happened.** It gets one of these suffixes:
  - `lost-mode timeout: the map from before the kidnap was abandoned`;
  - `still lost: the pose is on a fresh map, not on the map from before the kidnap` (a mission that
    ends while still lost, for example at the time limit).
- **Harness:** `explore_eval --explore-params FILE` with `--explorer own` now layers an extra
  parameter file over `mission.yaml` (`mission.launch.py mission_params:=FILE`). The live test uses
  this to force a short timeout without editing `mission.yaml`.

### Why 360 s
Every recorded lost-mode episode that ended in a reload was measured with
`scripts/lost_durations.sh`, across phases 11-13: **27-263 s**. The longest was phase-12 attempt-2
seed 1; the phase-13 regression run below took 224 s. 360 s is about 1.4 times the longest.

The one recorded episode that never ended was phase-12 attempt-2 seed 2: the teleport left the robot
physically stuck, so no search could succeed. That is the case the timeout exists for.

## Live tests (`scripts/phase13_tests.sh`, 2026-11-27, collapsed world, teleport at 90 s)

| Run | Timeout | Result | Lost for | Final state |
|---|---|---|---|---|
| teleport 1 | 20 s (forced, `scripts/lost_timeout_20s.yaml`) | gave up, kept exploring (15 goals, 1 recovery after) | gave up after 24 s | DONE, "...; lost-mode timeout: the map from before the kidnap was abandoned" |
| teleport 2 | 20 s (forced) | gave up, kept exploring (14 goals, 1 recovery after) | gave up after 27 s | DONE, same suffix |
| teleport 1 | 360 s (shipped) | **recovered**, 0.03 m, coverage 0.99 | found after 224 s | DONE, no suffix |
| normal mission | 360 s (shipped) | coverage 0.991; no relocalization code triggered | n/a | DONE, no suffix |

- **The give-up path works live.** In both forced runs:
  - the saved map was given up;
  - the robot explored the fresh map for about 6 more minutes, with no false kidnap or jump alarm
    on it;
  - the mission ended DONE with the reason.

  The harness scores these runs as not recovered (pose error 11.65 m and 8.85 m, coverage 0.19 and
  0.26). That is expected: after giving up, the map and pose are on the fresh map, whose frame does
  not match the world's. This is the cost the timeout accepts.
- **Timing:** the give-ups came 4-7 s after the 20 s mark, because the wake-up manoeuvre (back up,
  spin) was still running and the timeout waits for it.
- **The shipped timeout changed nothing in the regression run.** The 360 s teleport run recovered
  after 224 s in lost mode. A 20 s timeout would have thrown that recovery away.

## Reproduce
```bash
bash scripts/phase13_tests.sh                     # 2 forced-timeout teleports, 1 teleport + 1 normal at 360 s
bash scripts/lost_durations.sh results/phase11 results/phase12 results/phase13
```

## Known issues
- **After a timeout, the mission's map is the fresh one.** Pins found before the kidnap are in the
  old map's frame. Nothing merges the two maps.
- **Choosing the timeout is a trade-off.** Too short throws away slow recoveries (224 s and 263 s
  were measured); too long leaves a stuck robot searching. 360 s rests on 13 successful episodes in
  one world.
- **Small sample:** 2 forced-timeout runs and 2 regression runs.
- The map explored while lost is still discarded when the saved map is reloaded.
