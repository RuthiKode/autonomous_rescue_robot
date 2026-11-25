# Foxglove layout

`foxglove/rescue_layout.json` is a Foxglove Studio layout for a live run or a replayed bag. **Status: the JSON is
valid and its topic names come from the running system, but it was not opened in Foxglove Studio by me** (no
display or Foxglove install on this machine). Expect to adjust panel sizes after importing it.

## Panels
| panel | shows | topics |
|---|---|---|
| 3D | SLAM map, hazard layer, laser scan, Nav2 plan, frontier candidates, survivor pins, robot model | `/map`, `/hazard_map`, `/scan`, `/plan`, `/frontiers`, `/survivors/markers`, `/robot_description` |
| Image | robot camera | `/camera/image_raw` |
| State transitions | mission state over time | `/mission/state.state` |
| Plot x2 | explored area; goals sent / failed / recoveries / verifications | `/explore/explored_area_m2.data`, `/mission/state.*` |
| Raw messages x2 | latest survivors; latest detections | `/survivors`, `/detections` |

`/ground_truth/survivor_markers` is in the 3D panel but switched off (turn it on to compare pins with the truth).

## Use
Live:
```bash
sudo apt install ros-jazzy-foxglove-bridge       # once
ros2 launch rescue_bringup full_mission.launch.py world:=rooms foxglove:=true
```
Open Foxglove Studio, connect to `ws://localhost:8765` (WSL2 forwards localhost), and import the layout
(Layout menu, Import from file).

Replay: `explore_eval` records a bag per run under `results/<sweep>/runs/<id>/bag`; open it with
Foxglove (Open local file, then the same layout).

## Demo video
A screen recording of Foxglove cannot be made from this environment. `scripts/render_demo.py` renders a bag
offline into an mp4 (map, trajectory, pins, mission state); see `docs/PHASE9.md` for what it produced.
