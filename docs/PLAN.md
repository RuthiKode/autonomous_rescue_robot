# Autonomous Search & Rescue Robot: Pipeline and Stage Specs

Nov 12, 2026 · @Ruthik

This is a 12-week, simulation-first plan on ROS 2 Jazzy and Gazebo Harmonic under WSL2, built around one end-to-end slice by week 5 and an automated evaluation harness from week 3.

## Pipeline at a glance

&#91;embedded content: pipeline · sensors, 3 tracks, 1 behavior tree\]

Sensors feed three tracks: mapping and localization, perception, and exploration. The mission behavior tree joins them and sends goals to Nav2, whose /cmd\_vel output drives the robot back in Gazebo.

## Gaps closed

Every gap from the review has one design decision that fixes it, built in the stage named in the last column.

| Gap | Fix | Stage |
| --- | --- | --- |
| Integration deferred to week 8 | A thin end-to-end slice by week 5, and a full-system launch test every week from week 3 | Schedule |
| Nav2 on AMCL cannot explore an unknown map | Nav2 runs on slam\_toolbox's live map; AMCL is used only for replays on a saved map | Nav2 |
| No time to build the disaster worlds | Weeks 1-2 produce 3+ worlds with scripted, known survivor poses | Robot and worlds |
| Perfect simulated odometry makes the EKF pointless | Inject sensor noise and wheel slip so the EKF has real error to correct | Robot and worlds |
| COCO YOLO misses low-poly, lying, partly hidden people | Fine-tune on synthetic frames from your own worlds and report the domain gap | Perception |
| A 2D LiDAR plane misses a person lying below it | RGB-D depth, back-projected to 3D, transformed to the map frame with time-synced TF | Survivor localizer |
| One survivor becomes dozens of detections | Cluster in the map frame and confirm after N hits across M frames | Survivor localizer |
| "Hazardous areas" has no definition | Hazard = costmap cells in passages narrower than robot width + margin, shown as a map layer | Mapping layer |
| Frontier exploration is not part of Nav2 | explore\_lite is the baseline; your own survivor-aware selector is the contribution | Exploration |
| "Semantic mission planning" is vague | One concrete behavior: approach a detection for a verification view, then resume exploring | Mission tree |
| State machine has no failure handling | Behavior tree with stuck, lost-localization and unreachable-goal branches | Mission tree |
| Metrics and baselines undefined | Formal definitions, four baselines, at least 10 seeded runs per world | Evaluation |
| C++ listed but no C++ node planned | Python first; list C++ only if a node actually ships in it | Stack |
| Dashboard could eat a week | Foxglove first; a custom web page only if time remains | Dashboard |

## Environment (WSL2)

WSL2 with Ubuntu 24.04 is enough for this project, but the Gazebo GUI is the part to test first, before any project code.

**Day-1 gate**

1. In PowerShell: `wsl --update`, then `wsl --install -d Ubuntu-24.04`.
2. Inside WSL: `nvidia-smi` must list the RTX 4050. Use the Windows NVIDIA driver and install no Linux driver.
3. Install the stack:

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
  ros-jazzy-foxglove-bridge python3-colcon-common-extensions
```

4. Run `gz sim shapes.sdf` and `rviz2`. Both must open and hold a usable frame rate.
5. If the Gazebo GUI is black, crashes or stutters, try `export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`. If that fails, run Gazebo headless (`gz sim -s -r world.sdf`) and watch everything in RViz2 and Foxglove.

**Settings that matter**

- Limit WSL memory in `C:\Users\<you>\.wslconfig`: `[wsl2]` with `memory=10GB`, `processors=8`, `swap=8GB`.
- Keep the workspace in `~/` inside Linux, never under `/mnt/c/`; builds on the Windows filesystem are very slow.
- Budget 30-50 GB on C:. If space runs short, move the distro with `wsl --export` and `wsl --import`.
- Foxglove in a Windows browser reaches the bridge in WSL through localhost.

## Frames, topics and interfaces

One TF tree and one topic contract keep the packages independent: each node reads and writes only the names below.

**TF tree**

```text
map  ->  odom  ->  base_link  ->  laser_link
                              ->  camera_link
                              ->  imu_link
map -> odom       published by slam_toolbox
odom -> base_link published by the EKF (turn off the Gazebo diff-drive TF to avoid a duplicate)
```

Every node runs with `use_sim_time:=true`, and `/clock` is bridged from Gazebo. A missing `/clock` or a mixed time source causes most TF extrapolation errors.

**Topic contract**

| Topic | Type | From | To |
| --- | --- | --- | --- |
| /clock | rosgraph\_msgs/Clock | ros\_gz\_bridge | all nodes |
| /scan | sensor\_msgs/LaserScan | ros\_gz\_bridge | slam\_toolbox, Nav2 costmaps |
| /imu | sensor\_msgs/Imu | ros\_gz\_bridge | EKF |
| /odom\_raw | nav\_msgs/Odometry | ros\_gz\_bridge | EKF |
| /odometry/filtered | nav\_msgs/Odometry | EKF | Nav2 |
| /camera/image\_raw, /camera/depth, /camera/camera\_info | sensor\_msgs Image, Image (32FC1), CameraInfo | ros\_gz\_bridge | YOLO node, survivor localizer |
| /map | nav\_msgs/OccupancyGrid | slam\_toolbox | Nav2, exploration, dashboard |
| /cmd\_vel | geometry\_msgs/Twist | Nav2 | ros\_gz\_bridge, then Gazebo |
| /detections | vision\_msgs/Detection2DArray | YOLO node | survivor localizer |
| /survivors | rescue\_msgs/SurvivorArray | survivor localizer | mission tree, dashboard, evaluation |
| /frontiers | visualization\_msgs/MarkerArray | exploration | dashboard (debug) |
| /hazard\_map | nav\_msgs/OccupancyGrid | hazard layer node | dashboard |
| /mission/state | rescue\_msgs/MissionState | mission tree | dashboard, evaluation |
| /ground\_truth/survivors | rescue\_msgs/SurvivorArray | world publisher (simulation only) | evaluation only |

Two extra packages are needed beyond the original tree: `rescue_msgs` for the custom interfaces (a survivor has an id, a pose in the map frame, a confidence, a hit count, a first-seen time and a state) and `rescue_eval` for the scorer. Publish `/map` and `/survivors` with transient-local QoS so a dashboard that starts late still receives the current state.

## Stage specs

Eight stages, each with what to build and a pass/fail gate; week numbers match the schedule further down. Numeric thresholds are starting values to adjust once you have measured baselines.

### 1. Robot and worlds (weeks 1-2)

- **Robot:** a differential-drive Xacro model with every dimension (footprint, wheel radius, sensor mounts) as a parameter in one file. Sensors: 360-degree 2D LiDAR at about 10 Hz, an RGB-D camera at 15 Hz and an IMU at about 50 Hz.
- **Realism:** turn on Gazebo sensor noise for the LiDAR, camera depth and IMU, and lower wheel friction so the wheels slip. This gives the EKF real error to correct.
- **Bridge:** one `ros_gz_bridge` YAML file for every topic in the contract, including `/clock`. Disable the diff-drive plugin's odom TF so the EKF is the only `odom -> base_link` publisher.
- **Worlds:** at least three layouts generated by a seeded Python script that writes SDF: rooms and corridors, a collapsed floor with debris clusters and narrow gaps, and a larger open plan with clutter. Each holds 3-5 survivors in mixed poses (standing, sitting, lying, partly occluded). Survivor poses go in a YAML ground-truth file that a small world-publisher node republishes on `/ground_truth/survivors`.
- **Gate:** the robot can be teleoperated in all three worlds, `ros2 topic hz` shows the intended sensor rates, and `view_frames` matches the TF tree above.

### 2. SLAM (week 3)

- **Build:** `slam_toolbox` in online async mode with simulation time, loop closure on, and a range limit matching the LiDAR. Drive the robot by teleop and save the map with `nav2_map_server`'s `map_saver_cli`.
- **Scoring tools (in `rescue_eval`):** a script that rasterizes each world's SDF walls into a ground-truth occupancy grid, an IoU score between the built map and that grid, and an absolute trajectory error computed with `evo` against Gazebo's true robot pose.
- **Gate:** a loop closure is visible in RViz2, and IoU and trajectory error are recorded for every world as the mapping baseline. A starting target is IoU above 0.8 in the simplest world.

### 3. Nav2 on live SLAM (week 4)

- **Launch:** use Nav2's navigation launch only, with no map server and no AMCL. `slam_toolbox` already publishes `map -> odom`, so Nav2 plans on the map as it grows.
- **Costmaps:** a global costmap fed by `/map`, plus an obstacle layer from `/scan` and an inflation layer tuned to the robot radius plus a margin. Start from Nav2's default parameter file and change only the inflation radius and the controller limits for narrow gaps.
- **Recoveries:** keep spin, back-up, wait and costmap clearing enabled, and log each time one fires.
- **Hazard layer:** a small node (`rescue_mapping`) computes free-space clearance from `/map` and marks cells in passages narrower than robot width plus margin. It publishes `/hazard_map` for the dashboard and as a cost penalty for exploration.
- **Goal runner:** a script that sends 20 random reachable goals per world through `NavigateToPose` and logs success, time, path length and recovery count.
- **Gate:** the goal success rate is recorded for every world as the navigation baseline, and no run ends with the robot stuck without a recovery firing.

### 4. Perception (weeks 5-6)

- **Week 5, thin slice:** a YOLO node using pretrained COCO weights, class `person` only, publishing `/detections`. It exists only to prove the pipeline end to end.
- **Week 6, real detector:** collect labelled frames automatically. Add a Gazebo segmentation camera, drive the robot through randomized worlds (survivor poses, lighting, debris occlusion), and turn the segmentation masks into bounding boxes. This needs no hand labelling.
- **Split by world:** train on two worlds and test on the third, so the result measures generalization to a new layout and not memorization of one.
- **Compare:** pretrained against fine-tuned, with precision and recall per world and a breakdown for standing, lying and occluded survivors. Report the simulation-to-real gap as a limitation.
- **Runtime:** run inference on the GPU through the Windows NVIDIA driver, and cap the input size so it keeps up with the camera rate while Gazebo is running.
- **Licence:** the Ultralytics package is AGPL-3.0. That is fine for coursework, but check it before publishing the code.
- **Gate:** a precision and recall table on the held-out world, and sustained inference at the camera rate during a full run.

### 5. Survivor localizer (weeks 5-6)

This node turns a bounding box into a persistent pin on the map. It is the first of your two main contributions.

1. **Synchronize:** match each detection with the depth frame of the same time using an approximate time synchronizer, and look up the TF at the detection's own timestamp, never at "now".
2. **Measure depth:** take the median depth over the central part of the box. The median ignores background pixels that fall inside the box.
3. **Back-project:** convert the box centre and depth to a 3D point with the camera intrinsics from `/camera/camera_info`, then transform it to the `map` frame.
4. **Reject:** drop detections with invalid depth, depth beyond a maximum range, or confidence below the threshold.
5. **Associate:** match the point to the nearest existing track within a radius (start at 1 m). Update the track position with a confidence-weighted average and increment its hit count. If no track is near, start a new candidate.
6. **Confirm:** promote a candidate to `confirmed` only after N hits taken from at least K robot positions that are far enough apart. Viewpoint diversity stops a single bad angle from creating a false survivor. Unconfirmed candidates expire after a timeout.
7. **Publish:** `/survivors` carries id, map pose, confidence, hit count, first-seen time and state (`candidate`, `confirmed`, `verified`), with matching markers for the dashboard.

**Gate:** in a world with known survivors, each one ends as exactly one confirmed pin within 1 m of its true position, with no duplicates, in five consecutive seeded runs.

### 6. Exploration (week 5 baseline, week 7 own selector)

- **Baseline (week 5):** build `explore_lite` from the m-explore-ros2 repository, since it may not be released as a Jazzy package. Tune its frontier size and gain and potential scales. This gives the first autonomous exploration demo.
- **Own selector (week 7):** a `rescue_exploration` node that extracts frontiers from `/map` (free cells next to unknown cells), groups them into connected clusters, and drops clusters below a minimum size.
- **Score:** each frontier cluster gets estimated information gain (unknown area within sensor range of the cluster) divided by path cost from Nav2's `ComputePathToPose`, minus a penalty for hazard cells, with a blacklist for goals that already failed.
- **Survivor-aware behavior:** an unconfirmed candidate creates a verification goal at a viewpoint about 1.5-2 m from it with a clear line of sight. The mission tree visits it before returning to frontier search.
- **Stop rule:** exploration ends when no cluster above the minimum size remains, or when a time limit is reached.
- **Gate:** coverage-versus-time curves and completion times for the baseline and the own selector, over the same seeded runs.

### 7. Mission behavior tree (weeks 7-8)

The tree is the second main contribution and the place where the robot's behavior is defined. Use `py_trees` with `py_trees_ros`; check the Jazzy packages with `apt-cache search py-trees` first, and fall back to a plain state machine only if they are missing.

The root is a priority selector. Branches are tried in this order:

1. **Recover:** runs when a failure condition below is active.
2. **Verify:** runs when a survivor candidate is waiting for a closer view.
3. **Explore:** picks a frontier and sends it to Nav2 as a `NavigateToPose` goal.
4. **Finish:** runs when no frontiers remain, publishes the final report and sets the state to `DONE`.

| Failure | Detection | Response |
| --- | --- | --- |
| Robot stuck | No progress toward the goal for a set time | Clear costmaps, back up, blacklist the goal, pick another |
| Localization jump | Map-to-odom transform changes by more than a threshold in one step | Stop, rotate in place to re-scan, then resume |
| Goal unreachable | Nav2 returns an aborted result | Blacklist the goal and re-select |
| Repeated failures | More than N recoveries in a row | Set state to `FAILED` with a reason and stop |

Every state change is published on `/mission/state` with a reason string and is written to the evaluation log.

**Gate:** scripted failure tests (a blocked corridor, and the robot teleported to a new pose mid-run) recover automatically in five consecutive runs.

### 8. Dashboard (week 10)

- **Use Foxglove first.** Run `foxglove_bridge` in WSL and open Foxglove Studio in a Windows browser. The panels: a 3D view with the map, robot pose, survivor markers and hazard layer; an image panel showing detections; a state-transition view for `/mission/state`; and a plot of coverage over time.
- **Save the layout** as JSON in the repository so any run can be reopened with the same view.
- **Custom web page:** build one only if there is slack after the evaluation runs. It adds little to the evaluation.
- **Gate:** a screen recording of a full mission, using only the Foxglove layout from the repository.

## Evaluation harness

Every claim in the project should come from a seeded, automated run scored against Gazebo ground truth, and the harness exists from week 2 so each stage can be measured as it is built.

**Run protocol**

1. One script (`rescue_eval`) takes a world, a method and a seed. It launches Gazebo headless, starts a rosbag, runs the mission to completion or a time limit, shuts down and scores the bag.
2. Use sim time for every duration. Gazebo is not fully deterministic, so run at least 10 seeds per world and condition and report mean and standard deviation.
3. Match detections to truth one-to-one: a confirmed pin counts as correct if it is within 1 m of an unmatched true survivor.
4. Write one row per run to a CSV, and generate every table and plot in the report from those files only.

**Metrics**

| Metric | Definition | Computed from |
| --- | --- | --- |
| Navigation success | Goals reached within tolerance divided by goals sent | Goal runner log |
| Mapping quality | IoU between the final `/map` and the ground-truth grid | Map file and world SDF |
| Localization error | Absolute trajectory error (RMSE, metres) of the SLAM pose against Gazebo's true pose | Rosbag, `evo` |
| Exploration coverage | Explored reachable free cells divided by all reachable free cells | Map and ground-truth grid |
| Time to 90% coverage | Sim time until coverage first reaches 0.9 | Map stream |
| Detection precision | Correct confirmed pins divided by all confirmed pins | `/survivors`, ground truth |
| Detection recall | True survivors matched by a confirmed pin divided by all true survivors | `/survivors`, ground truth |
| Duplicate rate | Extra confirmed pins matching an already matched survivor | `/survivors`, ground truth |
| Pin position error | Mean distance between matched pins and true positions | `/survivors`, ground truth |
| Mission completion | Share of runs ending in `DONE`, and the time they took | `/mission/state` |
| Recoveries per run | Number of recovery behaviors fired | Mission log |

**Conditions to compare**

| Condition | What it shows |
| --- | --- |
| Random reachable goals | Lower bound for exploration |
| `explore_lite` defaults | The standard frontier baseline |
| Own selector without verification | Effect of your scoring function alone |
| Full system with verification | Effect of survivor-aware behavior |
| Pretrained against fine-tuned YOLO | Effect of the synthetic fine-tuning and the domain gap |

The claim you can defend is a measured difference between these rows on the same seeds, not that the pieces run.

## 12-week schedule

The schedule holds 10 weeks of planned work and 2 of slack. If you fall behind, cut dashboard polish and extra ablations first, and keep the evaluation.

| Week | Deliverable | Gate to move on |
| --- | --- | --- |
| 1 | WSL2 stack, robot Xacro, bridge config, teleop in an empty world | Day-1 gate passed; TF tree matches the contract |
| 2 | Three seeded worlds, sensor noise, ground-truth publisher, evaluation runner skeleton | Teleop works in all worlds; one scripted run writes a scored CSV row |
| 3 | `slam_toolbox`, map rasterizer, IoU and trajectory-error scoring | Mapping baseline recorded for every world |
| 4 | Nav2 on live SLAM, hazard layer, goal runner | Navigation baseline recorded; recoveries logged |
| 5 | Thin end-to-end slice: `explore_lite`, pretrained YOLO, crude pin on the map | Demo video: the robot explores and drops at least one pin |
| 6 | Synthetic dataset, fine-tuned YOLO, survivor localizer with deduplication | Held-out precision and recall table; one pin per survivor in 5 runs |
| 7 | Own frontier selector, behavior tree with the Explore branch | Coverage-versus-time curves for both selectors |
| 8 | Verify and Recover branches, failure tests, full launch file | Scripted failure tests pass in 5 runs |
| 9 | Full evaluation sweep: 10+ seeds, all worlds, all conditions | Every table generated from the CSV files only |
| 10 | Foxglove layout, plots, demo recording | Recording made with the layout stored in the repository |
| 11 | Ablations and fixes from the results, README, tests and CI | `colcon test` and CI pass |
| 12 | Slack, reruns, write-up, resume wording | Every resume claim traces to a measured result |

Start marking the project "in progress" on the resume when week 1's gate passes, and mark a feature as done only when its gate passes.

### Status at the end of the project (2026-11-25)
Per-week gate results are in docs/REPORT.md. Summary: weeks 1-4 met; week 5 met with an oracle detector only;
week 6 detector met, pin precision not; week 7 met; week 8 block 5/5 but teleport 0/5; week 9 met with 3-5
seeds per cell instead of 10; week 10 offline replay video, no Foxglove recording; week 11 README and 211
passing tests, CI passes on GitHub, ablations limited; week 12 REPORT.md and RESUME_CLAIMS.md.

## Risks and fallbacks

The riskiest items are the Gazebo GUI under WSL2, narrow-gap navigation and detector generalization, and each has a fallback that keeps the project on schedule.

| Risk | Fallback |
| --- | --- |
| Gazebo GUI is unusable under WSL2 | Run Gazebo headless and watch everything in RViz2 and Foxglove |
| Nav2 cannot pass the narrow gaps | Keep gap width a parameter in the world generator, widen it first, then tune inflation and the controller |
| Fine-tuned YOLO does not generalize to the held-out world | Add randomization (lighting, textures, survivor models, occlusion) and report the remaining gap as a result |
| Memory runs out with Gazebo, Nav2, SLAM and YOLO together | Set limits in `.wslconfig`, run headless, lower sensor rates and the YOLO input size |
| `explore_lite` does not build on Jazzy | Write a nearest-frontier baseline yourself; it takes a few days and is also a fair baseline |
| `py_trees_ros` is not available on Jazzy | Use a plain state machine with the same states and failure handling |
| slam_toolbox's pose jumps by 1-1.5 m in symmetric, corridor-like layouts and does not recover (measured in the rooms world, Phase 3: 2 of 3 runs) | Detect the jump in the mission tree (map-to-odom change in one step), stop and re-scan; check whether re-localization works before relying on it |
| Gazebo non-determinism blurs comparisons | Use more seeds and compare conditions pairwise on the same seeds |
| The schedule slips | Cut, in order: custom dashboard, extra ablations, extra world variants. Never cut the evaluation |

## Follow-up session (2026-11-27, `docs/PLAN_V1.md`)
Kidnap detection and global relocalization (`docs/PHASE10.md`): teleport 1/5 live (from 0/5; the other
four landed in unmapped space and were correctly not re-seeded), block 3/3, no false alarms.
Also: MIT licence, Gazebo window script, clean `git status`, 216 tests, v1.0.0 release.
Lost mode (`docs/PHASE11.md`): kidnaps into unmapped space. Teleport 5/5 live (final error ≤ 0.13 m),
block 2/2, 221 tests; three live attempts, each finding and fixing a failure, are kept.
Every re-seed confirmed by motion (`docs/PHASE12.md`): teleport 5/5 live with all 6 re-seeds
confirmed, block 2/2, normal mission done, 233 tests; two earlier attempts kept.
