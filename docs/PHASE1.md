# Phase 1 report (plan weeks 1-2)

Run on 2026-11-13: Ubuntu 24.04.5 under WSL2 (kernel 6.6.87.2), ROS 2 Jazzy, Gazebo Harmonic 8.15.0, RTX 4050 Laptop GPU with Windows driver 591.74.

**Result:** four of the five gate checks pass. The TF check passes only in part: there is no duplicate `odom -> base_link` publisher, but the `map -> odom -> base_link` part of the plan's tree does not exist yet, because nothing in Phase 1 publishes it (see "TF tree" below).

## Setup

```bash
cd ~/autonomous_rescue_robot
source /opt/ros/jazzy/setup.bash
rosdep check --from-paths src --ignore-src --rosdistro jazzy
colcon build
source install/setup.bash
```

`rosdep check` printed `All system dependencies have been satisfied`.

## Gate checks

### 1. Build and tests: pass

```bash
colcon build
colcon test
colcon test-result --all
```

```text
Summary: 5 packages finished [23.8s]

build/rescue_description/test_results/rescue_description/test_xacro.xunit.xml: 4 tests, 0 errors, 0 failures, 0 skipped
build/rescue_eval/pytest.xml: 2 tests, 0 errors, 0 failures, 0 skipped
build/rescue_simulation/pytest.xml: 72 tests, 0 errors, 0 failures, 0 skipped
Summary: 79 tests, 0 errors, 0 failures, 0 skipped
```

The tests cover: the Xacro expands with the required frames, sensor rates and noise; the bridge YAML covers the topic contract and does not bridge TF; the generator is deterministic, gives 3-5 survivors with all three postures and at least one occluded for 20 seeds per world, and takes gap width as a parameter; the committed worlds match the generator; the CSV writer keeps one header.

### 2. Teleop in all three worlds: pass (headless, scripted key presses)

```bash
scripts/phase1_gate.sh rooms --frames
scripts/phase1_gate.sh collapsed
scripts/phase1_gate.sh open_plan
```

The script starts the sim headless and feeds real key presses (`i` for 2 s, `k`, `j` for 2 s, `k`) to `teleop_twist_keyboard` through a pseudo-terminal. Position along x after the forward drive:

| World | `/odom_raw` x (m) | Gazebo true x (m) |
| --- | --- | --- |
| rooms | 0.9865 | 0.9847 |
| collapsed | 0.9710 | 0.9692 |
| open_plan | 0.9510 | 0.9492 |

To drive by hand:

```bash
ros2 launch rescue_bringup sim.launch.py world:=rooms headless:=false
# second terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Not verified: nobody drove the robot by hand, and nobody looked at the Gazebo GUI. With `headless:=false` the `gz sim gui` process started and was still alive after 30 s with a real-time factor of 0.999 and no errors in the log, but whether the window is usable needs your eyes.

### 3. Sensor rates: pass

`ros2 run rescue_eval sensor_check --duration 10` computes the rate from message header stamps (sim time). `ros2 topic hz` measures arrivals by wall clock.

| World | Topic | Target Hz | Sim-time rate | `ros2 topic hz` |
| --- | --- | --- | --- | --- |
| rooms | /scan | 10 | 10.00 | 9.662 |
| rooms | /imu | 50 | 50.00 | 45.004 |
| rooms | /camera/image_raw | 15 | 15.15 | 14.339 |
| collapsed | /scan | 10 | 10.00 | 9.721 |
| collapsed | /imu | 50 | 50.00 | 48.478 |
| collapsed | /camera/image_raw | 15 | 15.15 | 13.168 |
| open_plan | /scan | 10 | 10.00 | 8.868 |
| open_plan | /imu | 50 | 50.00 | 48.745 |
| open_plan | /camera/image_raw | 15 | 15.15 | 13.339 |

`/camera/depth` runs at 14.93 Hz in sim time. The wall-clock figures are lower because Python subscribers drop some messages: in 10 sim seconds `sensor_check` received 134-146 of about 152 colour frames and 92-110 depth frames. The sensors themselves tick at the intended rate.

Noise is present on every sensor. Spread between consecutive readings with the robot standing still (zero for a noise-free sensor), rooms world:

```text
topic               frame_id               received  sim span s  rate Hz  frame-to-frame noise (std)
/scan               laser_link                  102       10.10    10.00  0.01410 m
/imu                imu_link                    508       10.14    50.00  0.01441 rad/s, m/s^2
/camera/image_raw   camera_optical_link         146       10.03    15.15  0.03075 grey levels
/camera/depth       camera_optical_link         110       10.03    14.93  0.01419 m
```

### 4. TF tree: partial

```bash
ros2 run tf2_tools view_frames
```

Observed tree (rooms world), every transform broadcast by `robot_state_publisher`:

```text
base_link -> laser_link
base_link -> camera_link -> camera_optical_link
base_link -> imu_link
base_link -> chassis_link, caster_front, caster_rear   (static)
base_link -> left_wheel, right_wheel                   (from /joint_states)
```

`ros2 topic info /tf -v` and `/tf_static -v` each show one publisher, `robot_state_publisher`. There are zero `odom -> base_link` publishers: the diff-drive plugin's transform goes to the Gazebo topic `diff_drive/tf_unbridged`, which the bridge does not carry.

What is missing against the plan's tree: `map -> odom` (slam_toolbox, Phase 2) and `odom -> base_link` (EKF). This session's instructions say the EKF comes later, so I did not add it. The plan's week-1 gate says "TF tree matches the contract", which reads as if the EKF should already run. This needs your decision.

### 5. Evaluation script: pass

```bash
ros2 run rescue_eval run_eval --world rooms --seed 1
cat results/runs.csv
```

```text
[run_eval] sim time is running; recording 20.0 sim seconds
[run_eval] shutting down
[run_eval] status=ok; row appended to /home/ruthik/autonomous_rescue_robot/results/runs.csv

run_id,timestamp_utc,world,method,seed,status,sim_duration_s,wall_duration_s,real_time_factor,n_true_survivors,bag_path,bag_messages,bag_topics_with_data,nav_success_rate,map_iou,ate_rmse_m,coverage,time_to_90_coverage_s,det_precision,det_recall,duplicate_rate,pin_error_m,mission_completed,mission_time_s,recoveries
rooms_none_seed1_20261113T034625Z,2026-11-13T03:46:25+00:00,rooms,none,1,ok,20.000,20.474,0.977,3,bags/rooms_none_seed1_20261113T034625Z,2295,7,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan
```

The run facts (durations, real-time factor, bag message count) are measured. The twelve plan metrics are `nan` placeholders. No Gazebo, bridge or rosbag process was left running afterwards.

## Other commands

```bash
# Regenerate the worlds (same seed gives byte-identical files)
ros2 run rescue_simulation generate_worlds --output-dir src/rescue_simulation/worlds
# Narrower or wider gaps in the collapsed world, or another layout seed
ros2 run rescue_simulation generate_worlds --output-dir src/rescue_simulation/worlds --gap-width 0.9 --seed 7
```

Generated with seed 42: rooms has 3 survivors, collapsed 4, open_plan 3. Each world has standing, sitting and lying survivors and at least one occluded.

## Deviations from the plan and the session prompt

- **RGB-D is two Gazebo sensors.** A `camera` and a `depth_camera` share one pose. I was not sure the combined `rgbd_camera` type applies noise to depth and did not test it; with two sensors, noise is measured on both streams (check 3). Both stamp images with `camera_optical_link` (z forward), a child of `camera_link`.
- **`/joint_states` is bridged.** It is not in the topic contract. `robot_state_publisher` needs it for the wheel transforms.
- **The diff-drive TF is parked, not switched off.** The Harmonic plugin has no off switch, so its transform goes to an unbridged Gazebo topic.
- **`seed` means the Gazebo random seed.** `run_eval --seed` and `seed:=` set `gz sim --seed` (sensor noise). The world layout seed is separate, in `world_generator.yaml`. The plan does not say which one a run seed is.
- **World frame equals map frame by construction.** The robot always spawns at the world origin facing +x, so ground-truth poses are valid in `map` once SLAM starts there.
- **GPU rendering needs two environment variables under WSL2.** Mesa picked software rendering (llvmpipe) by default. The launch file sets `GALLIUM_DRIVER=d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` from `rescue_bringup/config/sim.yaml`; the Ogre log then reports `D3D12 (NVIDIA GeForce RTX 4050 Laptop GPU)`. The adapter variable alone, as the plan suggests, was not enough.
- **Extras:** `rescue_eval sensor_check` and `scripts/phase1_gate.sh` were added to make the gate reproducible. `MissionState` is not defined yet; Phase 1 does not use it.
- **`.wslconfig` was left as found** (`memory=12GB`, `swap=4GB`, `processors=6`); the plan suggests 10 GB, 8 GB and 8.

## Known issues

- **Wheel odometry under-reports turning by about 11%.** After turning in place at 1 rad/s for 4 s, `/odom_raw` gave a yaw of 4.09 rad and Gazebo's true yaw was 4.58 rad. My reading is that the wheels pivot on a narrower track than `wheel_separation` (0.34 m); I did not test that. Straight-line odometry is within 2 mm over 1 m, so the low friction (`wheel_mu` 0.35) causes little slip when driving straight.
- **Images are dropped on the way to Python subscribers** (see check 3). Later nodes should use sensor-data QoS and expect gaps.
- **A lying survivor is barely visible to the LiDAR.** The scan plane is at 0.20 m and a lying body is at most 0.22 m high, so the LiDAR only grazes the torso and head. This is the case the plan expects the depth camera to cover. Not measured.
- **Reachability is not checked by the generator.** Keep-out circles hold doorways, gaps and the spawn clear, but no path search proves every survivor can be reached.
- **Rosbags leave out camera images** to keep them small (`rescue_eval/config/eval.yaml`).
- **`results/` is not committed.** The CSV row above is a smoke test; decide later whether result tables belong in git.
- **`package.xml` maintainer email is a placeholder** (`ruthik@todo.todo`).
- **Gazebo prints four `gz_frame_id ... not defined in SDF` warnings** at start-up. They are expected on Harmonic and the frame ids are applied.
- **A shutdown message** (`NodeShared::Publish() Error: Interrupted system call`) appeared once when stopping Gazebo with Ctrl-C. It had no effect.
