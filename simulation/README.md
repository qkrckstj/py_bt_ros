# LIMO factory simulation

Snapshot of the working Ubuntu 24.04 / ROS 2 Jazzy / Webots R2025a setup.
Includes the factory world, saved map, robot meshes, ROS launch files, Nav2
configuration, camera/YOLO overlay and regression tests. Logs, caches, build outputs
and downloaded Webots assets are excluded.

## Build once

Install ROS 2 Jazzy, Webots R2025a and rosdep first. Prepare the BT and trained YOLO
runtime using [the search scenario instructions](../scenarios/limo_factory_search/README.md).
The default YOLO interpreter is `~/.venvs/human-yolo/bin/python`.

```bash
source /opt/ros/jazzy/setup.bash
export PY_BT_ROS_ROOT=~/ros2_ws/src/py_bt_ros
rosdep install --from-paths "$PY_BT_ROS_ROOT/simulation/webots_ros2_limo" --ignore-src -r -y --rosdistro jazzy
mkdir -p ~/limo_sim_ws
cd ~/limo_sim_ws
colcon build --symlink-install --base-paths "$PY_BT_ROS_ROOT/simulation/webots_ros2_limo" --packages-select webots_ros2_limo
python3 "$PY_BT_ROS_ROOT/simulation/webots_ros2_limo/scripts/cache_factory_assets.py"
```

`simulation/COLCON_IGNORE` prevents automatic discovery beside an existing local
`webots_ros2_limo` checkout. The explicit child `--base-paths` above builds this copy.
Use only one simulator package installation per terminal; do not source the old
simulation workspace on top of this one. Change `PY_BT_ROS_ROOT` if cloned elsewhere.

## Run (three terminals)

In **each** terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/limo_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export PY_BT_ROS_ROOT=~/ros2_ws/src/py_bt_ros
```

1. Simulator:
   `ros2 launch webots_ros2_limo robot_launch.py world:=limo_factory_hall.wbt`
2. Navigation, YOLO, RViz and BT:
   `ros2 launch webots_ros2_limo factory_search_launch.py`
3. After the map, transforms and navigation are ready:
   `ros2 topic pub --once /bt/search/start std_msgs/msg/Empty '{}'`

Closing the BT or RViz stops its navigation/YOLO launch. Stop navigation before
resetting Webots, then restart navigation. A session lock prevents duplicate launches.
For fresh SLAM, add `slam:=true` to command 2; the default is the saved map plus AMCL.
The saved map is a simulated lidar survey, not a SLAM pose graph; its provenance
metadata retains the original survey hash, not a claim that later world edits were re-surveyed.

## Saved settings

- Start: x=-4.28 m, y=5.02 m, yaw=0.
- Maximum forward speed: 0.40 m/s.
- Global inflation: 0.30 m; local inflation: 0.05 m (obstacle cost radii, not body clearance).
- Goal tolerance: 0.30 m, no final-heading requirement.
- Rectangular robot footprint, padding and collision monitoring retained.
- Added obstacle boxes and manhole covers removed; person target inside the office.
- One visible Webots YOLO camera overlay; depth/IR sensors remain enabled for detection.

## Sources and licenses

Robot package adapted from [inmo-jang/webots_ros2_limo](https://github.com/inmo-jang/webots_ros2_limo),
base commit `68d4fc027a01478c7645eb17cda3ba5a5ba298ba`. Original
[Apache-2.0 license](webots_ros2_limo/LICENSE) and attribution are retained.
The factory world is adapted from [Cyberbotics Webots R2025a factory hall](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/environments/factory/worlds/hall.wbt).
External Webots PROTO/texture/mesh assets are fetched from their original URLs and
retain their respective upstream licenses; the download cache is not included.
