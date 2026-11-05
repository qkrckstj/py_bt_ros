# webots_ros2_limo

For this bundled Jazzy factory setup, start with [../README.md](../README.md).

AgileX **LIMO Pro** (four-wheel differential drive, or mecanum wheels) in [Webots](https://cyberbotics.com), driven by `webots_ros2_driver` + `ros2_control`, with cartographer SLAM and Nav2 navigation.

Tested on **Ubuntu 22.04 / ROS 2 Humble / Webots R2025a**.

---

## 1. Install Dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-webots-ros2 \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-teleop-twist-keyboard \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup
```

Webots R2025a must be installed at `/usr/local/webots` (or exported via `WEBOTS_HOME` / `ROS2_WEBOTS_HOME`).

## 2. Clone & Build

```bash
# in your ROS 2 workspace root (e.g. ~/my_ros2_ws)
git clone https://github.com/inmo-jang/webots_ros2_limo.git src/webots_ros2_limo
colcon build --packages-select webots_ros2_limo
source install/local_setup.bash
```

---

## 3. Run the Simulator

```bash
ros2 launch webots_ros2_limo robot_launch.py
```

This starts Webots with the LIMO robot, the `webots_ros2_driver`, the `ros2_control` spawners and `robot_state_publisher`.
The `limo_sensors` plugin publishes the sensors and `/odom` with the `odom -> base_link` TF, taking distance from the wheel encoders and heading from the IMU.

Drive it with the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` forward, `,` backward, `j`/`l` rotate, `u`/`o` forward + rotate.
`w`/`x` change linear speed, `e`/`c` angular.

### Launch arguments

| Argument | Default | Purpose |
|---|---|---|
| `drive` | `diff` | `diff` (four-wheel differential) or `mecanum` (omnidirectional, see step 6) |
| `world` | per `drive` | World file from `worlds/`: `limo_world.wbt` or `limo_world_mecanum.wbt` |
| `mode` | `realtime` | Webots startup mode (`realtime`, `fast`, `pause`) |
| `slam` | `false` | Also start SLAM + RViz in the same command |
| `nav` | `false` | Also start Nav2 + RViz in the same command |
| `map` | `resource/limo_world_map.yaml` | Map used when `nav:=true` |
| `use_sim_time` | `true` | Use the Webots `/clock` as ROS 2 time |

---

## 4. SLAM (cartographer)

Use **two terminals**.
Source your workspace in each one.

**Terminal 1** — wait until Webots is open and the robot is standing in the world before going on:

```bash
ros2 launch webots_ros2_limo robot_launch.py
```

**Terminal 2** — cartographer + RViz:

```bash
ros2 launch webots_ros2_limo cartographer_launch.py
```

Drive around with the keyboard teleop to build the map, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f limo_map
# Output in the current directory: limo_map.yaml, limo_map.pgm
```

![Cartographer SLAM in RViz while driving the LIMO with the keyboard (3x speed)](doc/cartographer.gif)

| Argument | Default | Purpose |
|---|---|---|
| `use_rviz` | `true` | Open RViz with the SLAM view |
| `configuration_basename` | `limo_lds_2d.lua` | Cartographer lua config in `resource/` |
| `resolution` | `0.05` | Occupancy grid cell size [m] |
| `use_sim_time` | `true` | Use the Webots `/clock` as ROS 2 time |

---

## 5. Navigation (Nav2 + AMCL)

Again **two terminals**.

**Terminal 1** — wait until Webots is open before going on:

```bash
ros2 launch webots_ros2_limo robot_launch.py
```

**Terminal 2** — Nav2 + AMCL + RViz:

```bash
ros2 launch webots_ros2_limo nav2_launch.py
```

In RViz, click **2D Pose Estimate** to set the robot's initial pose, then **Nav2 Goal** to send it somewhere.
Or from the command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.1, y: -3.6}, orientation: {w: 1.0}}}}"
```

![Nav2 driving the LIMO to a goal set in RViz (3x speed)](doc/nav2.gif)

| Argument | Default | Purpose |
|---|---|---|
| `drive` | `diff` | Selects the default `params_file`; use the same value as in terminal 1 |
| `map` | `resource/limo_world_map.yaml` | Map yaml for AMCL to localise against |
| `params_file` | `resource/<drive>/nav2_params.yaml` | Nav2 parameter file |
| `use_rviz` | `true` | Open RViz with the Nav2 view |
| `use_sim_time` | `true` | Use the Webots `/clock` as ROS 2 time |

To navigate with your own map from step 4, run this from the directory where you saved it:

```bash
ros2 launch webots_ros2_limo nav2_launch.py map:=limo_map.yaml
```

---

## 6. Mecanum Drive

With mecanum wheels the LIMO also moves sideways.
Add `drive:=mecanum` to the launch commands of steps 3-5; everything else stays the same.

```bash
ros2 launch webots_ros2_limo robot_launch.py drive:=mecanum
```

`/cmd_vel` then also takes `linear.y` (positive = left):

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.1}}"
```

In `teleop_twist_keyboard`, hold **Shift** for sideways motion: `J`/`L` strafe left/right, `U`/`O`/`M`/`>` move diagonally.

For navigation, start Nav2 with the same drive:

```bash
ros2 launch webots_ros2_limo nav2_launch.py drive:=mecanum
```

The robot keeps its heading while it drives and turns to the goal heading only after it has arrived:

![Nav2 driving the mecanum LIMO to goals set in RViz without turning on the way (3x speed)](doc/mecanum.gif)

| | `drive:=diff` | `drive:=mecanum` |
|---|---|---|
| Robot model | `LimoFourDiff.proto` | `LimoMecanum.proto` |
| Controller | `diff_drive_controller` | `mecanum_drive_controller` |
| Nav2 controller | DWB | DWB, holonomic (keeps its heading while driving) |
| AMCL motion model | differential | omnidirectional |

The mecanum rollers are not modelled geometrically: as in the Webots KUKA youBot, each wheel has an asymmetric contact friction rotated by ±45°, defined in `worlds/limo_world_mecanum.wbt`.

---

## Sensors

The default launch (no SLAM / Nav2, either `drive:=diff` or `drive:=mecanum`) publishes the same sensor interface as the real LIMO running `ros2 launch wego teleop_launch.py`: same topic names, message types, `frame_id`s, rates, QoS and TF frames, measured on the robot.
Code written against the simulator therefore runs on the robot unchanged.

| Topic | Type | Rate | frame_id | Notes |
|---|---|---|---|---|
| `/scan` | `LaserScan` (BEST_EFFORT) | 10 Hz | `laser_link` | YDLIDAR, ±110°, 248 beams, `range_min` 0.01 / `range_max` 64, no return = `0.0` |
| `/imu` | `Imu` | 100 Hz | `imu_link` | gravity included, same covariances as `limo_base` |
| `/odom` | `Odometry` | 50 Hz | `odom` → `base_link` | wheel encoders + IMU heading, with the `odom → base_link` TF |
| `/joint_states` | `JointState` | 25 Hz | `base_link` | real wheel angles (`joint_state_broadcaster`); the robot publishes zeros at 10 Hz |
| `/camera/color/image_raw` (+`/compressed`) | `Image` rgb8 640×480 | ~30 Hz | `camera_color_optical_frame` | Orbbec DaBai DC1 |
| `/camera/depth/image_raw` (+`/compressed`, `/compressedDepth`) | `Image` 16UC1 [mm] 640×480 | ~30 Hz | `camera_color_optical_frame` | aligned to color, 0.3–3 m, 0 = no data |
| `/camera/ir/image_raw` (+`/compressed`) | `Image` mono16 640×400 | ~30 Hz | `camera_color_optical_frame` | |
| `/camera/{color,depth,ir}/camera_info` | `CameraInfo` | ~30 Hz | `camera_color_optical_frame` | intrinsics copied from the robot |
| `/camera/depth/points` | `PointCloud2` xyz | ~30 Hz | `camera_color_optical_frame` | |
| `/camera/depth_registered/points` | `PointCloud2` xyz+rgb | ~30 Hz | `camera_color_optical_frame` | |

> **Note:** The main sensors (LiDAR, IMU, wheel odometry and the depth camera) are simulated as listed above, but some other topics of the real robot are not.

Robot control: `/cmd_vel` in (`linear.x`, `angular.z`; with `drive:=mecanum` also `linear.y`), `/odom` and `odom -> base_link` TF out.

---

## Package Layout

```
webots_ros2_limo/
├── launch/
│   ├── robot_launch.py         # Webots + driver + ros2_control spawners
│   ├── cartographer_launch.py  # cartographer_ros nodes + RViz
│   └── nav2_launch.py          # nav2_bringup + RViz, with this package's map
├── protos/
│   ├── LimoFourDiff.proto      # AgileX LIMO robot model (4-wheel diff)
│   ├── LimoMecanum.proto       # same robot with mecanum wheels
│   └── meshes/                 # limo_base.dae, limo_wheel.dae
├── resource/
│   ├── limo.urdf               # Webots driver plugins (limo_sensors + ros2_control)
│   ├── limo_lds_2d.lua         # cartographer config
│   ├── limo_world_map.{yaml,pgm}
│   ├── diff/
│   │   ├── ros2control.yaml    # diff_drive_controller config
│   │   └── nav2_params.yaml    # Nav2 parameters (DWB)
│   └── mecanum/
│       ├── ros2control.yaml    # mecanum_drive_controller config
│       └── nav2_params.yaml    # Nav2 parameters (DWB, holonomic)
├── webots_ros2_limo/
│   └── limo_sensors.py         # /scan, /imu, /odom, /camera/* like the real robot
├── doc/
│   └── cartographer.gif, nav2.gif, mecanum.gif
├── rviz/
│   └── limo_cartographer.rviz  # SLAM view
└── worlds/
    ├── limo_world.wbt          # apartment world, LimoFourDiff
    └── limo_world_mecanum.wbt  # same world, LimoMecanum
```


## LIMO Round-trip BT (py_bt_ros)

### Factory Hall world

`worlds/limo_factory_hall.wbt` adapts the [Cyberbotics R2025a factory hall](https://github.com/cyberbotics/webots/blob/R2025a/projects/samples/environments/factory/worlds/hall.wbt).
It adds the differential-drive LIMO at `(-4.28, 5.02, 0.145)`, heading +X, and removes
three visual-only roof panels so the interior is visible while editing. Industrial
objects and their interactions are preserved; the original sample is unchanged.

Scenario props are defined near the end of the world: `RESCUE_DUMMY` is a stationary
human model with collision enabled at `(5.55, 0.65, 1.27)` (hip height), inside the office.
The two added obstacle boxes and four square manhole covers are removed.
Internal workshop/office door leaves and the green shutter are removed; exterior
entrances remain. The dummy is a visual detection target with collision enabled.

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select webots_ros2_limo
source install/local_setup.bash
export ROS_DOMAIN_ID=42
python3 ~/ros2_ws/src/py_bt_ros/simulation/webots_ros2_limo/scripts/cache_factory_assets.py
ros2 launch webots_ros2_limo robot_launch.py world:=limo_factory_hall.wbt
```

The factory search BT uses `factory_search_launch.py` after starting the simulator.
It loads the saved factory map (not the apartment map), Nav2, RViz, and the py_bt_ros
continuous person detection with a three-region route. A detection cancels navigation
and records the person position before SUCCESS. The saved map is a Webots lidar survey, not a SLAM pose graph.
The launch starts the trained `human_yolo` detector by default, using
`~/.venvs/human-yolo/bin/python` (`yolo_python:=...` to override).
Prepare that environment with the factory-search README's installation commands.
`mock_detector:=true` selects the old distance mock instead; the two never run together.
The two added obstacle boxes have been removed from the factory world.
The four square manhole covers have also been removed.

```bash
ros2 launch webots_ros2_limo factory_search_launch.py
# In another sourced terminal, after Nav2/map/TF are ready:
ros2 topic pub --once /bt/search/start std_msgs/msg/Empty '{}'
```

Goals ignore final heading and accept 0.30 m XY error (`goal_tolerance:=0.40` to change).
Maximum forward speed is 0.40 m/s in both DWB and the velocity smoother.
Inflation radii are separate: 0.30 m for global planning and 0.05 m for local
trajectory scoring. These radii measure obstacle cost expansion, not body-to-wall
clearance. The body footprint, 0.03 m padding, and collision monitor are unchanged.
Live lidar obstacle layers detect unmapped boxes. Their minimum height is -0.2 m
because laser_link is below the planar base_link origin. A padded rectangular
footprint and footprint collision critic account for the robot body.

For fresh online mapping instead of saved-map localization, use `slam:=true` on
`factory_search_launch.py` (not on the simulator launch). This replaces AMCL with
SLAM Toolbox. Restart the simulator at its initial pose first. Unknown distant rooms
may be outside the new map bounds; map them with manual/nearby-goal driving first.
Full commands: `../../scenarios/limo_factory_search/README.md`.

The factory RViz view renders at 10 FPS. RViz subscribes to dynamic `/tf` with
best-effort QoS so a slow software renderer cannot hold up navigation's reliable
TF stream. Odometry TF retains five outgoing samples (100 ms at 50 Hz), while
fixed camera extrinsics are latched on `/tf_static`. The collision monitor and
its sensor freshness checks remain enabled. After changing TF publication,
restart both simulator and navigation to clear their old TF buffers.
Stop the factory-search launch before restarting the simulator. Closing either
its RViz or BT window now shuts down that launch's Nav2/AMCL/YOLO processes too.
Only one factory-search launch per user and ROS domain is allowed on this machine;
a second launch exits with an explanation instead of creating duplicate servers.
For a fresh saved-map session, the simulator spawn and AMCL initial pose are both
`x=-4.28, y=5.02, yaw=0`. Restart the factory-search launch after resetting Webots.

Edit the source `.wbt`; launch creates a temporary copy. Asset download helper:
`scripts/cache_factory_assets.py`. To regenerate the static reference map with Webots
closed, run `python3 scripts/generate_factory_map.py`. The survey excludes the person
and added boxes without modifying the source scene. Upstream Webots asset licenses apply.

### Round-trip scenario

The BT lives in the separate `py_bt_ros` repository, in `scenarios/limo_round_trip/`.
It uses two action node types: `MoveToGoal` and `WaitForSecond`:
**go to goal → wait 10 simulation seconds → return to the pose saved when the goal was received**.
Both repositories need these changes when sharing with teammates.
The `py_bt_ros` checkout uses the `dev_chanseo` branch and the original XML/Pygame
runner, with a scenario Agent selected through `agent.module`. Full Korean instructions:
`../../scenarios/limo_round_trip/README.md`.

On the current ROS 2 Jazzy machine:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select webots_ros2_limo
source install/local_setup.bash
export ROS_DOMAIN_ID=42
```

In terminal 1 (source ROS and the workspace, and set `ROS_DOMAIN_ID=42`, in every terminal):

```bash
ros2 launch webots_ros2_limo robot_launch.py
```

Wait for Webots to run and publish sensors, then in terminal 2:

```bash
ros2 launch webots_ros2_limo bt_round_trip_launch.py
```

This starts Nav2, AMCL, RViz and the external BT runner. Wait for Nav2 to be active
and the BT to report `IDLE`. Use RViz **2D Pose Estimate** if localization needs correcting.
The round-trip RViz configuration sends **2D Goal Pose** to `/bt/goal_pose`.
Alternatively, in terminal 3:

```bash
ros2 topic pub --once -w 1 /bt/goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}'

ros2 topic echo /bt/mission_status std_msgs/msg/String --qos-durability transient_local
```

Expected stages: `MOVING_TO_GOAL → WAITING → RETURNING_HOME → COMPLETE`.
The home pose is saved for each accepted mission and published on `/bt/home_pose`.
Arrival uses Nav2's goal tolerances. A new goal is ignored while a mission is running.
Ctrl+C in terminal 2 requests cancellation of active BT navigation; then stop terminal 1.
The Pygame window shows the executing XML tree. Cancel without closing it using
`ros2 topic pub --once -w 1 /bt/cancel std_msgs/msg/Empty '{}'` while the BT is ticking.

Optional arguments:

- `py_bt_ros_root:=/absolute/path/to/py_bt_ros` (default: `PY_BT_ROS_ROOT` or `~/ros2_ws/src/py_bt_ros`).
- `nav:=false` if Nav2 is already running.
- `use_rviz:=false` for command-line-only operation.
- `drive:=mecanum` on both launches for the mecanum model (round-trip runtime validation was on diff).

On Humble, source `/opt/ros/humble/setup.bash` instead. Jazzy automatically adapts the
bundled Nav2 parameters and stamps Twist commands for its diff-drive controller while
keeping the public `/cmd_vel` interface as `geometry_msgs/msg/Twist`.
Custom `params_file` arguments to `nav2_launch.py` are used without conversion.
The BT is executed from source; building the `py_bt_ros` repository is not required.
