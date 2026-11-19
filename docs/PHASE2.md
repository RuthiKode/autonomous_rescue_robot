# Phase 2 report: Phase 1 close-out and SLAM (plan week 3)

Run on 2026-11-13, same machine and stack as Phase 1 (Ubuntu 24.04 under WSL2, ROS 2 Jazzy, Gazebo Harmonic, RTX 4050 through Mesa D3D12).

**Result:** every gate item has output below. Three results look bad and are reported as such: map IoU is below the plan's 0.8 starting target in every world (0.53-0.62 against 0.8), with the original friction the EKF was slightly worse than raw wheel odometry (fixed by adding wheel slip, see 0d-2), and a blocked robot with spinning wheels drags both the EKF and SLAM away from the truth.

Nothing here was checked by eye. RViz2 loads both configs without crashing; whether they look right is yours to judge.

## Setup

```bash
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
rosdep check --from-paths src --ignore-src --rosdistro jazzy
pip install --user --break-system-packages --no-deps evo     # evo has no rosdep key
colcon build
source install/setup.bash
```

`rosdep check` printed `All system dependencies have been satisfied`. `evo` 1.38.1 is the one dependency rosdep cannot resolve; `--no-deps` keeps pip from replacing the system numpy that ROS uses. `python3-pip` was installed with apt for this.

## Gate

### Build and tests: pass

```bash
colcon build
colcon test            # set RESCUE_SKIP_SIM_TESTS=1 to skip the one test that starts Gazebo
colcon test-result --all
```

```text
Summary: 6 packages finished [31.8s]
build/rescue_description/test_results/rescue_description/test_xacro.xunit.xml: 4 tests, 0 errors, 0 failures, 0 skipped
build/rescue_eval/pytest.xml: 14 tests, 0 errors, 0 failures, 0 skipped
build/rescue_simulation/pytest.xml: 82 tests, 0 errors, 0 failures, 0 skipped
build/rescue_slam/pytest.xml: 3 tests, 0 errors, 0 failures, 0 skipped
Summary: 104 tests, 0 errors, 0 failures, 0 skipped
```

The single-publisher TF test (`rescue_eval/test/test_tf_single_publisher.py`) ran in 25.2 s and passed. It starts Gazebo headless with the bridge, EKF and slam_toolbox on a private ROS domain, waits for `map -> odom -> base_link -> laser_link / camera_link / imu_link`, then listens for 8 s.

### Saved map and rosbag per world: present

Under `results/phase2/` (not committed): `runs/<run_id>/map.pgm` and `map.yaml`, `runs/<run_id>/trajectory_slam.tum` and `trajectory_truth.tum`, and `bags/<run_id>/`. To look at a run in RViz2:

```bash
rviz2 -d install/rescue_bringup/share/rescue_bringup/rviz/phase2.rviz --ros-args -p use_sim_time:=true
# second terminal
ros2 bag play results/phase2/bags/<run_id> --clock
```

The bag holds `/scan`, `/tf`, `/tf_static`, `/map`, `/robot_description`, the pose graph markers (`/slam_toolbox/graph_visualization`), both odometry topics and the ground truth. Camera images are left out.

### IoU and trajectory error per world: recorded

See "Step 4" for the table.

## Step 0: Phase 1 close-out

### 0a. Camera rate

```bash
scripts/camera_rate.sh rooms 60      # also collapsed, open_plan
```

`/camera/image_raw`, 60 sim seconds per subscriber type. Full output: `results/phase2/camera_rate.txt`.

| World | Subscriber | Received | Dropped | Sim-time rate (Hz) | Wall rate (Hz) | Real-time factor |
| --- | --- | --- | --- | --- | --- | --- |
| rooms | Python, best effort, not decoding | 745 | 167 (18.3%) | 12.37 | 11.82 | 0.955 |
| rooms | Python, best effort, decoding | 750 | 161 (17.7%) | 12.47 | 11.70 | 0.938 |
| rooms | Python, reliable, depth 50 | 911 | 0 | 15.15 | 14.72 | 0.972 |
| rooms | `ros2 bag record` (C++) | 1095 | 0 | 15.15 | 14.38 | not measured |
| collapsed | Python, best effort, not decoding | 829 | 82 (9.0%) | 13.79 | 13.38 | 0.971 |
| collapsed | Python, best effort, decoding | 849 | 62 (6.8%) | 14.12 | 13.69 | 0.969 |
| collapsed | Python, reliable, depth 50 | 911 | 0 | 15.15 | 14.70 | 0.970 |
| collapsed | `ros2 bag record` (C++) | 1090 | 0 | 15.15 | 14.27 | not measured |
| open_plan | Python, best effort, not decoding | 851 | 60 (6.6%) | 14.15 | 13.76 | 0.972 |
| open_plan | Python, best effort, decoding | 855 | 56 (6.1%) | 14.22 | 13.81 | 0.971 |
| open_plan | Python, reliable, depth 50 | 911 | 0 | 15.15 | 14.73 | 0.972 |
| open_plan | `ros2 bag record` (C++) | 1096 | 0 | 15.15 | 14.18 | not measured |

What this shows:

- **The sensor renders every frame.** `/camera/camera_info` carries the same stamps in a tiny message and had 0 gaps in every arm, and a reliable subscriber or a rosbag receives every image.
- **Frames are lost only with best-effort QoS.** The bridge publishes reliable. A best-effort subscriber loses 6-18% of these 0.9 MB images, whether or not it decodes them, so the loss is in transport and not in Python decoding. Longest gap: 6 frames (rooms), 3 frames (others).
- **Wall rate equals sim rate times real-time factor.** 15.15 Hz x 0.972 = 14.73 Hz. Gazebo reports a factor of 1.00 with no image subscriber and 0.94-0.97 while images are being consumed.
- **The nominal rate is 15.15 Hz, not 15.** The physics step is 1 ms, so a 66.67 ms period becomes 66 ms.
- **Not the cause:** render cost (no sensor frame was skipped) and the sensor update rate.

Changes made: `sensor_check` subscribes to images with reliable QoS. Later image consumers (YOLO node, survivor localizer) should do the same.

Side finding, fixed: the depth camera was publishing a second copy of its camera info on `/camera/camera_info`, so that topic ran at 30 Hz with duplicate stamps. The depth sensor's Gazebo topic moved to `depth_camera/image`; `/camera/camera_info` is now 15.15 Hz with no duplicates.

### 0b. Diff-drive geometry

The plugin and the Xacro agree, because both come from the same two properties. From the SDF that Gazebo receives:

```text
<wheel_separation>0.34</wheel_separation>      robot_params.xacro: wheel_separation = 0.34
<wheel_radius>0.07</wheel_radius>              robot_params.xacro: wheel_radius = 0.07
left wheel joint at y = +0.17, right at y = -0.17, both at z = 0.07
```

So there was no parameter mismatch. The deficit came from contact geometry: the wheel collision was a 0.04 m wide cylinder, and it carried the robot on its inner edge. That puts the contact points 0.30 m apart instead of 0.34 m, and 0.30 / 0.34 = 0.88 matches the yaw deficit. Fix: a separate `wheel_collision_width` property (0.005 m); the wheel visual keeps its 0.04 m width.

```bash
ros2 run rescue_eval ekf_validation --worlds rooms --trials 5 --square-runs 0 --output results/phase2/geometry_after.csv
```

Raw wheel odometry against Gazebo truth, open-loop drive (forward 2 s at 0.5 m/s, turn 2 s at 1 rad/s), 5 fresh simulations each, rooms world:

| Wheel collision width | Raw yaw error (rad), mean +/- sd | Raw position drift (m) |
| --- | --- | --- |
| 0.04 m (before) | -0.2038 +/- 0.0001 | 0.0020 +/- 0.0000 |
| 0.005 m (after) | +0.0005 +/- 0.0000 | 0.0038 +/- 0.0000 |

### 0c. EKF covariances

Gazebo publishes IMU and odometry with all-zero covariances and robot_localization has no parameter to override them, so a small node, `covariance_stamper`, fills them in. Values are in `rescue_simulation/config/sensor_covariances.yaml`:

| Field | Value | Source |
| --- | --- | --- |
| IMU angular velocity variance | 4.0e-6 (rad/s)^2 | `imu_gyro_noise_stddev` = 0.002 in the Xacro, squared. Unit-tested against the Xacro. |
| IMU linear acceleration variance | 4.0e-4 (m/s^2)^2 | `imu_accel_noise_stddev` = 0.02, squared. Unit-tested. Not fused by the EKF. |
| IMU orientation | marked "not provided" (element 0 = -1) | Gazebo's IMU orientation has no noise model. |
| Odometry vx variance | 7.5e-5 (m/s)^2 | Measured: mean square of (odom - true) vx while moving, 7.27e-5 over 5 trials. Wheel odometry has no configured noise. |
| Odometry vyaw variance | 5.0e-4 (rad/s)^2 | Measured the same way: 4.97e-4. |
| Odometry vy variance | 1.0e-4 (m/s)^2 | Chosen. The robot cannot move sideways; this lets the EKF use vy = 0. |
| Odometry pose variances | 1.0e-2 m^2, 1.0e-1 rad^2 | Chosen placeholders. The EKF does not fuse odometry pose. |
| z, roll, pitch | 1.0e6 | Chosen. A planar robot has no information on them. |

Over the full validation (39 runs) the measured values were 6.23e-5 and 4.63e-4, close to the ones set.

The gyro bias (`imu_gyro_bias_mean` = 0.0005 rad/s) is not in the covariance. It is a constant offset, not white noise, and robot_localization has no bias state for it.

With real covariances in place the EKF now also fuses the wheel yaw rate (`ekf.yaml`). The gyro is weighted about 125 times more.

### 0d. EKF validation

```bash
ros2 run rescue_eval ekf_validation --trials 10 --square-runs 3 --output results/phase2/ekf_validation.csv
```

Every trial is a fresh headless simulation with its own Gazebo seed. Errors are the difference between estimated and true motion from the start to the end of the drive. Mean +/- standard deviation; yaw error is signed, position drift is a distance.

| World | Test | n | Raw yaw error (rad) | EKF yaw error (rad) | Raw position drift (m) | EKF position drift (m) |
| --- | --- | --- | --- | --- | --- | --- |
| rooms | drive | 10 | +0.0005 +/- 0.0000 | +0.0013 +/- 0.0034 | 0.0038 +/- 0.0000 | 0.0037 +/- 0.0005 |
| rooms | square, 3 laps | 3 | -0.0173 +/- 0.0001 | +0.0641 +/- 0.0309 | 0.0095 +/- 0.0000 | 0.0248 +/- 0.0122 |
| collapsed | drive | 10 | +0.0005 +/- 0.0000 | +0.0014 +/- 0.0034 | 0.0038 +/- 0.0000 | 0.0036 +/- 0.0005 |
| collapsed | square, 3 laps | 3 | -0.0178 +/- 0.0003 | +0.0732 +/- 0.0262 | 0.0097 +/- 0.0001 | 0.0289 +/- 0.0102 |
| open_plan | drive | 10 | +0.0005 +/- 0.0000 | +0.0013 +/- 0.0034 | 0.0038 +/- 0.0000 | 0.0037 +/- 0.0005 |
| open_plan | square, 3 laps | 3 | -0.0179 +/- 0.0001 | +0.0759 +/- 0.0284 | 0.0097 +/- 0.0000 | 0.0302 +/- 0.0112 |

The drive test is the Phase 1 one (forward 2 s, turn 2 s, open loop). The square loop is 0.7 m x 0.5 m, three laps, steered on Gazebo's true pose so the path is the same every run.

**This looks bad for the EKF, and it is a real result.** After the wheel fix, raw odometry is nearly exact on a clean floor: the wheels do not slip at `wheel_mu` = 0.35 with the configured acceleration limits. The EKF takes yaw from the gyro, which has a 0.0005 rad/s bias, so over the roughly 100 s square loop it drifts about 0.07 rad where raw odometry drifts 0.018 rad. The plan warned about exactly this ("perfect simulated odometry makes the EKF pointless"). Two ways to restore a real error for the EKF to correct, the first was done afterwards, see 0d-2:

- lower `wheel_mu` until the wheels slip under normal acceleration, or
- add noise to the wheel odometry itself (Gazebo's diff-drive has none).

The EKF does earn its place when the wheels really spin; see "Known issues".

During the collapsed and open_plan trials a second simulation was running on another ROS domain (recording a drive). The numbers match the rooms trials, which ran alone.

### 0d-2. Wheel slip follow-up (`wheel_mu` 0.35 to 0.12)

You chose to deal with the wheel-slip question first, and I took the plan's own fix, lower wheel friction. The decision I made for you is the value; say if you want a different severity.

**Sweep.** Rooms world, 3 open-loop drives and 1 square loop per value (`results/phase2/mu_sweep.txt`). The square loop is a single run per value, so those columns are indicative.

| `wheel_mu` | Open-loop drive | Square: raw yaw / position error | Square: EKF yaw / position error |
| --- | --- | --- | --- |
| 0.35 (before) | no slip | -0.017 rad / 0.010 m | +0.064 rad / 0.025 m |
| 0.20 | no slip | +0.012 rad / 0.005 m | +0.069 rad / 0.026 m |
| 0.12 | no slip | -0.149 rad / 0.071 m | +0.037 rad / 0.015 m |
| 0.08 | no slip | -2.56 rad / 0.98 m | -0.064 rad / 0.052 m |
| 0.05 | +0.17 rad / 0.020 m | -0.84 rad / 0.96 m | -0.232 rad / 0.163 m |

The open-loop drive (smooth, 0.5 m/s, 1 rad/s) does not slip until 0.05; the square loop (corner turns, stop-and-go) slips from about 0.15. 0.12 is the highest value where slip is clearly present, and odometry is still usable. At 0.08 and below, raw odometry is off by up to a metre.

**Validation at 0.12** (same protocol as 0d: 10 open-loop drives and 3 square loops per world, `results/phase2_slip/ekf_validation.txt`):

| World | Test | n | Raw yaw error (rad) | EKF yaw error (rad) | Raw position drift (m) | EKF position drift (m) |
| --- | --- | --- | --- | --- | --- | --- |
| rooms | drive | 10 | +0.0005 +/- 0.0000 | +0.0013 +/- 0.0033 | 0.0043 +/- 0.0000 | 0.0043 +/- 0.0004 |
| rooms | square, 3 laps | 3 | -0.1504 +/- 0.0038 | +0.0203 +/- 0.0296 | 0.0731 +/- 0.0056 | 0.0116 +/- 0.0043 |
| collapsed | drive | 10 | +0.0005 +/- 0.0000 | +0.0013 +/- 0.0033 | 0.0043 +/- 0.0000 | 0.0043 +/- 0.0004 |
| collapsed | square, 3 laps | 3 | -0.1480 +/- 0.0060 | +0.0224 +/- 0.0297 | 0.0703 +/- 0.0037 | 0.0126 +/- 0.0051 |
| open_plan | drive | 10 | +0.0005 +/- 0.0000 | +0.0013 +/- 0.0034 | 0.0043 +/- 0.0000 | 0.0042 +/- 0.0004 |
| open_plan | square, 3 laps | 3 | -0.1531 +/- 0.0121 | +0.0218 +/- 0.0298 | 0.0757 +/- 0.0070 | 0.0117 +/- 0.0040 |

With slip, the EKF cuts the square-loop yaw error from about -0.15 to +0.02 rad and the position error from about 0.07 m to 0.012 m, in every world. On the gentle open-loop drive nothing slips, so raw odometry and the EKF are equal. The EKF's yaw error across the three laps is not stable (+0.037, +0.038, -0.014 in rooms): the standard deviation (0.030) is larger than the mean.

**Covariances re-tested, and not changed.** Under slip the measured twist error of the wheel odometry is 5.8e-3 (m/s)^2 for vx and 1.6e-3 (rad/s)^2 for yaw over the full validation, far above the values in `sensor_covariances.yaml` (7.5e-5 and 5.0e-4, measured without slip). I tried 6.0e-3 and 1.7e-3 on the nine square loops (`results/phase2_slip/ekf_validation_newcov.txt`):

| Odometry vx / vyaw variance | EKF yaw error, mean over the 3 worlds | EKF position drift, mean |
| --- | --- | --- |
| 7.5e-5 / 5.0e-4 (kept) | +0.0215 rad | 0.0120 m |
| 6.0e-3 / 1.7e-3 | +0.0268 rad | 0.0125 m |

No improvement, so the file is unchanged. Two caveats: the "measured" value depends on the drive (squares alone give 2.0e-3 and 5.4e-3, the mixed set 5.8e-3 and 1.6e-3), so a single number is not well defined; and the EKF's error here is dominated by the gyro bias, which a covariance cannot remove.

**Re-recorded drives.** The old drives were open-loop commands recorded at 0.35. Replayed with slip, the robot would not follow the same path, so the three drives were recorded again at 0.12 with the tour steering on the true pose (`src/rescue_eval/drives/` now holds those; the old ones are in git history).

### 0e. Evaluation script rerun

```bash
ros2 run rescue_eval run_eval --world rooms --seed 1
```

```text
run_id,timestamp_utc,world,method,seed,status,drive,sim_duration_s,wall_duration_s,real_time_factor,n_true_survivors,bag_path,bag_messages,bag_topics_with_data,map_path,nav_success_rate,map_iou,map_accuracy,ate_rmse_m,ate_aligned_rmse_m,coverage,time_to_90_coverage_s,det_precision,det_recall,duplicate_rate,pin_error_m,mission_completed,mission_time_s,recoveries
rooms_none_seed1_20261113T083554Z,2026-11-13T08:35:54+00:00,rooms,none,1,ok,,20.000,20.585,0.972,3,bags/rooms_none_seed1_20261113T083554Z,4622,11,,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan
```

The CSV gained four columns (`drive`, `map_path`, `map_accuracy`, `ate_aligned_rmse_m`). The Phase 1 smoke-test file was moved to `results/phase1_runs.csv`; `run_eval` refuses to append to a file with a different header.

### 0f. RViz config

`rescue_bringup/rviz/phase1.rviz` has RobotModel, TF, LaserScan (`/scan`), camera image (`/camera/image_raw`) and the ground-truth survivor markers (`/ground_truth/survivor_markers`, new). Fixed frame: `odom`.

```bash
ros2 launch rescue_bringup sim.launch.py world:=rooms rviz:=true static_map_tf:=true
```

`static_map_tf:=true` publishes an identity `map -> odom` so the markers, which are in `map`, can be drawn without SLAM. Do not combine it with SLAM. `phase2.rviz` (fixed frame `map`, adds the SLAM map and pose graph) is the default for `mapping.launch.py rviz:=true`.

What I checked: `rviz2 -d <config>` started with each file, ran 20 s and exited on Ctrl-C with no crash. What I did not check: anything visible. One message needs your eyes: with `phase2.rviz` RViz logged

```text
[ERROR] [rviz2]: rviz/glsl120/indexed_8bit_image.vert ... GLSL link result : active samplers with a different type refer to the same texture image unit
```

when the Map display was created, under both the D3D12 and the software renderer. If the SLAM map does not appear in RViz2, this is the first thing to look at.

## Step 1: SLAM

```bash
ros2 launch rescue_bringup mapping.launch.py world:=rooms headless:=false rviz:=true
```

`rescue_slam` holds `config/slam_toolbox.yaml` and `launch/slam.launch.py`, which includes slam_toolbox's own `online_async_launch.py` (the node is a lifecycle node in Jazzy; that launch file configures and activates it). Resolution 0.05 m, laser range 0.15-12.0 m (unit-tested against the Xacro), loop closure on, sim time on. It differs from slam_toolbox's defaults in four places, each marked in the file: `base_frame` (base_link), `map_update_interval` (2 s), and `minimum_travel_distance` / `minimum_travel_heading` (0.2).

TF with the stack running (`ros2 run rescue_eval tf_audit --duration 5`):

```text
map -> odom            /tf         slam_toolbox
odom -> base_link      /tf         ekf_filter_node
base_link -> wheels    /tf         robot_state_publisher
base_link -> sensors   /tf_static  robot_state_publisher
RESULT: every transform has one publisher
```

**How the single-publisher test works, and a deviation from your instruction.** You asked for the tf2 buffer's frame YAML, which lists each transform's authority. In ROS 2 that field is always `default_authority` (it was in Phase 1's `view_frames` output too), because `/tf` messages do not carry the sender's name. So the frame YAML alone cannot tell two publishers apart. The test still reads it, for the tree shape, and adds the per-publisher sequence number that the middleware attaches to each message: one publisher's numbers strictly increase, two publishers' counters interleave. It also fails on a child with two parents and on a transform present on both `/tf` and `/tf_static`.

Negative controls, run against the live stack (`results/phase2/tf_negative_control.txt`):

```text
===== clean stack
RESULT: every transform has one publisher                                   exit=0
===== with a duplicate odom -> base_link on /tf   (scripts/duplicate_tf_publisher.py)
  - odom -> base_link on /tf: sequence number 51 after 444; more than one publisher sends this transform
                                                                             exit=1
===== with a duplicate map -> odom on /tf_static   (static_transform_publisher)
  - map -> odom is published on both /tf and /tf_static                     exit=1
```

My first attempt at a control used a second EKF node and "passed". That control was invalid: the second EKF never published (it was waiting for the clock). The controls above use publishers that do send.

## Step 2: Repeatable mapping runs

```bash
# terminal 1
ros2 launch rescue_bringup mapping.launch.py world:=rooms
# terminal 2: record while you drive, Ctrl-C to save
ros2 run rescue_eval cmd_vel_record --output src/rescue_eval/drives/rooms.csv
# terminal 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# later, against a fresh simulation
ros2 run rescue_eval cmd_vel_replay --input src/rescue_eval/drives/rooms.csv
```

A drive file is a CSV of sim-time offset, linear x and angular z, one row per received command. Replay publishes each row when the sim clock reaches the same offset.

**Replay is not bit-exact.** The commands are identical; the motion is not. Each command lands within about a millisecond of its offset and not on the same physics step, contact and slip amplify small differences, and sensor noise differs per seed. Measured spread of the true end pose across three replays of one file:

| World | Drive length | End-position spread | End-yaw spread |
| --- | --- | --- | --- |
| rooms | 80.9 m, 216 s | 0.030 m | 0.014 rad |
| collapsed | 37.9 m, 101 s | 0.004 m | 0.003 rad |
| open_plan | 92.1 m, 229 s | 0.107 m | 0.021 rad |

Replay has no feedback. If the robot touches anything, the rest of the drive is wrong (see "Known issues").

**The committed drives are not yours.** You were not at the keyboard, so `scripts/record_drive.sh <world>` made them with `auto_drive`: it plans a tour on the generator's geometry (goals by farthest-point sampling, A* with 0.40 m clearance, return to the start for a loop) and steers on Gazebo's true pose. It is test tooling that knows the map in advance, not part of the robot. Replace the files in `src/rescue_eval/drives/` with your own recordings whenever you like.

## Step 3: Scoring

### 3a. Ground-truth occupancy grid

```bash
ros2 run rescue_simulation generate_worlds --output-dir src/rescue_simulation/worlds
```

writes `<world>_truth_map.png` and `.yaml` (map_server format, 0.05 m, world frame) from the same geometry as the SDF. Choices:

- **It is a cut at the LiDAR height (0.20 m)**, not a floor plan. Debris lower than 0.20 m is free in the grid because the LiDAR cannot see it. `slice_z` is unit-tested against `lidar_z`.
- **Survivors are in it**, since the LiDAR sees them.
- **Solids are filled.** A cell is occupied when its centre is inside a shape.
- **Outside the outer walls is unknown.**
- `free_thresh` is 0.196, so grey 205 (unknown) is not read as free.

A unit test checks the committed grids against the generator.

### 3b. Map score

```bash
ros2 run rescue_eval map_score --truth src/rescue_simulation/worlds/rooms_truth_map.yaml --map results/phase2/runs/<run_id>/map.yaml
```

**Alignment.** Nothing is fitted. slam_toolbox's map frame starts at the robot's start pose, and the robot's spawn pose in the world is known from the ground-truth YAML, so `p_world = R(spawn_yaw) * p_map + (spawn_x, spawn_y)`. The generator spawns the robot at the world origin facing +x, so today this is the identity; the code does not assume it. Each truth cell centre is mapped into the SLAM map and takes the value of the cell it lands in. Map drift lowers the score; no best-fit hides it.

**Scores**, both over cells that are free or occupied in both maps:

- IoU = cells occupied in both / cells occupied in either.
- Accuracy = share of those cells where the two maps agree.

Saved maps mark unknown as grey 205 with `free_thresh` 0.25, which a plain threshold reads as free. The loader treats 205 as unknown.

### 3c. Trajectory error

Gazebo's true pose of `base_link` is bridged to `/ground_truth/odom` (an OdometryPublisher plugin on the robot, 50 Hz, world frame). During a run, `run_eval` samples it at 10 Hz and looks up `map -> base_link` at the same stamp. Both go to TUM files, and evo computes the translation APE:

- `ate_rmse_m`: no alignment. The map frame is tied to the world by the start pose, so drift counts.
- `ate_aligned_rmse_m`: after evo's rigid alignment, the figure SLAM papers usually call ATE.

```bash
evo_ape tum results/phase2/runs/<run_id>/trajectory_truth.tum results/phase2/runs/<run_id>/trajectory_slam.tum
```

(`evo_ape` needs evo's full dependencies; the harness uses evo as a library and does not.)

### 3d. In the CSV

`map_iou`, `map_accuracy`, `ate_rmse_m` and `ate_aligned_rmse_m` are filled for `--slam` runs. The other ten plan metrics stay `nan`.

## Step 4: Baseline

```bash
scripts/phase2_baseline.sh 3 results/phase2
ros2 run rescue_eval summarize --csv results/phase2/runs.csv --method slam_replay
```

Three runs per world, Gazebo seeds 1-3, each replaying that world's drive. Mean +/- standard deviation (sample, n = 3):

| World | Runs | IoU (occupied) | Accuracy | ATE RMSE (m) | ATE RMSE, aligned (m) |
| --- | --- | --- | --- | --- | --- |
| rooms | 3 | 0.5275 +/- 0.0504 | 0.9555 +/- 0.0063 | 0.0471 +/- 0.0048 | 0.0245 +/- 0.0029 |
| collapsed | 3 | 0.6167 +/- 0.0482 | 0.9776 +/- 0.0034 | 0.0446 +/- 0.0052 | 0.0149 +/- 0.0021 |
| open_plan | 3 | 0.5648 +/- 0.0203 | 0.9792 +/- 0.0015 | 0.1877 +/- 0.0377 | 0.1047 +/- 0.0169 |

Per-run values are in `results/phase2/runs.csv`. The table is regenerated with `summarize` above.

Reading the table:

- **The plan's starting target (IoU above 0.8 in the simplest world) is not met.** Best mean IoU is 0.62 (collapsed). Accuracy over cells known in both is 0.96-0.98.
- **Collapsed IoU covers the start zone only.** The scored cells are those known in both maps, and the tour never leaves that zone.
- **Trajectory error is small except in open_plan.** Rooms and collapsed: about 0.045 m RMSE. Open_plan: 0.19 m (0.10 m after rigid alignment), on the longest drive (92.1 m).
- **Open_plan error comes and goes.** In all three runs the error between SLAM and truth rises to 0.25-0.36 m at three quarters of the drive (peak 0.37-0.54 m) and is back to 0.004-0.013 m at the end, when the robot returns to its start. That pattern is what a loop closure produces. I did not confirm it in the pose graph; the bags are there for you to check in RViz2.
- **Three runs per world is a small sample.** The standard deviations are indicative only. The runs differ by Gazebo seed (sensor noise and slip), not by drive; the drive is one file per world.
- **Excluded from the table:** three open_plan runs made with a faulty drive file (method `slam_replay_truncated_drive`, see Known issues), and one rooms run that failed to save its map (`map_not_saved`, repeated successfully as seed 3).

### Baseline with wheel slip (`wheel_mu` 0.12)

Same protocol, drives re-recorded at 0.12, method `slam_replay_slip` (`results/phase2_slip/`, a separate CSV). Mean +/- standard deviation, n = 3:

| World | IoU (occupied) | Accuracy | ATE RMSE (m) | ATE RMSE, aligned (m) |
| --- | --- | --- | --- | --- |
| rooms | 0.6093 +/- 0.0310 | 0.9653 +/- 0.0034 | 0.0445 +/- 0.0121 | 0.0207 +/- 0.0021 |
| collapsed | 0.6680 +/- 0.0069 | 0.9813 +/- 0.0008 | 0.0290 +/- 0.0053 | 0.0167 +/- 0.0011 |
| open_plan | 0.5378 +/- 0.0348 | 0.9771 +/- 0.0026 | 0.1824 +/- 0.0239 | 0.1074 +/- 0.0071 |

Against the no-slip baseline above: rooms IoU 0.528 to 0.609, collapsed 0.617 to 0.668, open_plan 0.565 to 0.538; ATE is similar or lower. **Do not read this as "slip helps mapping".** The two baselines use different drive files (the slip tours were planned again and recorded under slip), and with three runs each, differences of 0.05 in IoU are within the spread of the rooms world (sd 0.03-0.05). What the table does show is that SLAM still works with realistic slip: the map quality did not collapse, and the target of IoU 0.8 is still not reached.

## Deviations from the plan and from the Phase 2 instructions

- **`/imu` and `/odom_raw` now come from `covariance_stamper`.** The bridge publishes `/gz/imu` and `/gz/odom_raw`. The plan's topic table lists the bridge as their source.
- **New simulation-only topics:** `/ground_truth/odom` and `/ground_truth/survivor_markers`.
- **The TF test does not rely on the frame YAML's authority field** (see Step 1).
- **The drives were recorded by a script, not by you** (see Step 2).
- **The ground-truth grid is generated by the world generator**, as you asked, instead of by rasterizing the SDF in `rescue_eval` as the plan says.
- **The EKF fuses wheel yaw rate again**, now that covariances weight it.
- **`evo` comes from pip**, the only dependency outside rosdep.
- **Wheel collision is 0.005 m wide**; the visual is unchanged.
- **`wheel_mu` is 0.12, not the Phase 1 value 0.35**, and the three drives were recorded again under that friction. The no-slip results stay in this report as the clean-floor reference.
- **Loop closure is on but was not confirmed.** The plan's gate says a loop closure is visible in RViz2. That is your check; every drive returns to its start to give SLAM the chance.

## Known issues

- **Map IoU is below the plan's 0.8 target.** Accuracy is 0.94-0.98, so the maps agree on almost every cell; the occupied-cell IoU is strict at 5 cm. Measured with  on the nine slip runs: SLAM has 1.2-1.5 times as many occupied cells as the truth, and the extra cells lie a median of 1 cell (5 cm) from a true wall. Allowing a 1-cell tolerance lifts IoU from 0.51-0.68 to 0.72-0.91, and a 2-cell tolerance to 0.79-0.95:

| World | Strict IoU | 1-cell tolerance | 2-cell tolerance | SLAM / truth occupied cells |
| --- | --- | --- | --- | --- |
| rooms (3 runs) | 0.58-0.64 | 0.77-0.88 | 0.83-0.93 | 1.17-1.24 |
| collapsed (3 runs) | 0.66-0.68 | 0.89-0.91 | 0.94-0.95 | 1.43-1.44 |
| open_plan (3 runs) | 0.51-0.58 | 0.72-0.78 | 0.79-0.85 | 1.40-1.49 |

So the maps are in the right place and the wall cells are thicker or offset by one or two cells; that is a measurement-resolution effect more than a mapping failure. The tolerance IoU is my own ad hoc diagnostic (not a plan metric), and I did not tune slam_toolbox. Part of the thickness may come from how the truth grid is cut (single height, centre-in-shape rule) and part from scan noise; I did not separate those.
- **A blocked robot corrupts the estimate.** In three open_plan runs made with a faulty drive file (kept in the CSV under method `slam_replay_truncated_drive`; IoU 0.22-0.28, ATE 0.50-0.71 m), the robot drove into a crate 12 s in and sat there with its wheels spinning. Raw odometry advanced 2 m while the robot stood still, the EKF followed it, and SLAM was pulled 0.3 m at once and 1.4 m by the end (ATE 0.50-0.71 m). The gyro kept EKF yaw right (0.16 against raw 0.43 at that moment), which is the one case here where the EKF clearly beat raw odometry. Stuck detection belongs to the mission tree later.
- **That faulty drive was my recording script's fault, and fixing it took two attempts.** The recorder had not subscribed when the tour started, so the opening turn was missing and the replayed robot ended up facing a crate. The first fix (wait for the subscription) was not enough: the simulator bridge could connect before the recorder, and the robot moved on commands the recording never got. `auto_drive` now waits for two `/cmd_vel` subscribers (bridge and recorder) and stands still for 1 s before the first command; the recording starts with that turn (`0.999,0.0000,1.0000`). The faulty file is kept at `results/phase2/open_plan_truncated_drive.csv`.
- **The rooms and collapsed drives were recorded before that fix.** Their first rows are present (`0.000,0.0000,1.0000` and `0.000,0.0000,-1.0000`) and their replays end where the tour ended (rooms and collapsed spread table above), so they are not affected; they have not been re-recorded with the final script.
- **The collapsed drive never passes a narrow gap.** The tour planner keeps 0.40 m clearance, which closes the 0.7 m gaps, so the collapsed baseline maps the start zone only.
- **`map_saver_cli` failed once in 13 SLAM runs** (`Failed to spin map subscription`, its 2 s default wait). The wait is now 10 s (`eval.yaml`). The failed run stays in the CSV as `map_not_saved` and was repeated; the 10 s wait has not hit the failure again in the four runs since.
- **Gazebo crashes on shutdown.** `gz sim` often ignores SIGINT for 5 s and then exits with code -11. Bags and maps are already written by then.
- **The EKF logs `Failed to meet update rate`** once at start-up.
- **Slip makes open-loop replay less repeatable.** Replay end-pose spread under slip was not re-measured; the figures in Step 2 are from the no-slip drives.
- **Below `wheel_mu` 0.08 odometry becomes unusable** (0.98 m error in one square loop); the severe end was only run once per value.
- **`evo` is not checked by rosdep**, and without it `run_eval` leaves the trajectory columns `nan` and says so.
- **Carried over from Phase 1:** nobody has looked at the Gazebo GUI or driven by hand; `results/` is not committed; reachability of survivors is not proven by the generator.

## Decisions for you

1. **Wheel slip: settled in 0d-2 at `wheel_mu` 0.12.** Say if you want less or more slip (0.08 is the severe end).
2. **IoU target.** The diagnostic above says a 1-cell tolerance already gives 0.72-0.91. Keep the strict 5 cm IoU and tune slam_toolbox toward 0.8, or report a 1-cell tolerance IoU alongside it?
3. **Your own drives.** The baseline should eventually use drives you recorded, including one through the collapsed world's gaps.
