# spot-isaac-lab-hospital

**Version 2.0**

Boston Dynamics **Spot** simulated in NVIDIA **Isaac Sim 5.1.0** inside a
custom hospital environment, with a full **ROS 2 Jazzy** interface for
navigation, perception and control.

Release notes are tracked in [`CHANGELOG.md`](CHANGELOG.md).

The package ships:

- a custom hospital USD world with extra lighting added (see [`docs/MODIFY_WORLD.md`](docs/MODIFY_WORLD.md))
- a character-enabled hospital USD with manually placed chair / wheelchair prims used by seated people scenarios
- a Spot robot with the sensors needed for ROS 2 navigation already attached in the initial stage
- a standalone Isaac Sim driver script that boots the scene, runs the locomotion policy and bridges sensors / `cmd_vel` to ROS 2
- a standalone Isaac Sim people-control test UI driven by YAML scenarios
- a combined Spot + people Isaac Sim bridge that uses the people-control scene as the base stage and adds Spot's ROS 2 bridge on top

The robot is driven by Isaac Sim's `SpotFlatTerrainPolicy` (RL locomotion
policy) and accepts `/cmd_vel` (Twist or TwistStamped) from any ROS 2
controller — Nav2, `teleop_twist_keyboard`, custom planners, etc.

<img src="docs/photos/main_readme/robot_in_scene.png" width="800" title="Spot robot inside the hospital scene with lighting"/>

## Why Isaac Sim instead of Gazebo?

Gazebo is easy to set up but its physics is approximate. Isaac Sim provides
photorealistic rendering, an accurate physics world, a larger library of
simulation-ready assets, and finer control over character movement in the
scene. The tradeoff is that Isaac Sim needs a more powerful machine (GPU
required) and has a smaller community, which can make troubleshooting harder.
This repo aims to lower that barrier by shipping a working hospital + Spot
setup out of the box.

---

## Prerequisites

- **Ubuntu 24.04 LTS**
- **NVIDIA GPU** with a recent driver (RTX series recommended)
- **Isaac Sim 5.1.0** — workstation install recommended
  <https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/index.html>
- **ROS 2 Jazzy**
  <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>
- ROS 2 dependencies:
  ```bash
  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-robot-state-publisher \
      ros-jazzy-teleop-twist-keyboard ros-jazzy-rqt-robot-steering
  ```

After installing Isaac Sim, verify it from the App Selector. From the Isaac
Sim install directory:

```bash
./isaac-sim.selector.sh
```

<img src="docs/photos/main_readme/isaac_selector_window.png" width="450" title="Isaac Sim App Selector — confirm Package Path and click START"/>

Pressing **START** should bring up an empty Isaac Sim stage:

<img src="docs/photos/main_readme/empty_isaac_scene.png" width="800" title="Empty Isaac Sim 5.1.0 stage — installation verified"/>

---

## Setup

```bash
# 1. Clone
git clone https://github.com/SJoyTheHawk/spot-isaac-lab-hospital.git
cd spot-isaac-lab-hospital

# 2. Configure for your machine
cp env/spot_isaac.env.template env/spot_isaac.env
$EDITOR env/spot_isaac.env       # set ISAAC_SIM_PATH if not ~/isaac-sim

# 3. Build the ROS 2 workspace
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
cd ..
```

The hospital USD path is resolved by `scripts/run_isaac.sh` in this order:

1. `HOSPITAL_USD` env var (set in `env/spot_isaac.env`)
2. `assets/isaac_hospital_scene_spot.usd` inside the repo (default)

The combined Spot + people bridge resolves its stage from `SPOT_PEOPLE_USD`,
then falls back to `assets/isaac_hospital_scene_spot_w_characters.usd`.

So a fresh clone runs without editing any absolute path.

---

## Operation

### Option A — Spot + People Bridge (recommended for v2.0)

```bash
./scripts/run_spot_bridge_with_people.sh
```

This launches one Isaac Sim app with the character-enabled hospital USD,
registers saved people under `/World/Characters`, opens the **People Control
Test** window, then enables the ROS 2 bridge for Spot. Use this mode when you
want robot sensors, `/cmd_vel`, `/clock`, `/odom`, `/tf`, and animated people
in the same simulation.

The combined bridge is implemented in
[`isaac_sim/spot_bridge_with_people.py`](isaac_sim/spot_bridge_with_people.py).
It is intentionally self-contained and does not import the standalone Spot or
people scripts at runtime.

People scenario captures:

- [Initialize people](docs/initialize_people.mp4)
- [People feature demo 1](docs/people_feature_1.mp4)
- [People feature demo 2](docs/people_feature_2.mp4)

### Option B — Spot only

```bash
./scripts/run_isaac.sh
```

Wait ~1 minute for assets to load. Use the scroll wheel to zoom into the
world to see the Spot robot.

<img src="docs/photos/main_readme/robot_with_sensors.png" width="800" title="Spot robot in initial stage — 3D LiDAR (top blue cylinder) + RGB camera, IMU, RealSense and fisheye sensors attached"/>

### Optional — `robot_state_publisher` (URDF → TF)

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch spot_hospital_bringup robot_state_publisher.launch.py
```

This publishes the dynamic leg-link TFs (`body → fl_hip → fl_uleg → ...`)
from the `/joint_states` topic emitted by Isaac.

### Drive the robot

Teleop (keyboard):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_unstamped \
    -p stamped:=true
```

Or send a one-shot command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{header: {frame_id: base_link}, twist: {linear: {x: 0.5}}}"
```

For a GUI velocity controller, run `rqt_robot_steering` and point it at the
same `/cmd_vel` topic:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_robot_steering rqt_robot_steering
```

### Option C — People only

The character test scene can also run without Spot or ROS 2:

- USD: `assets/isaac_hospital_scene_spot_w_characters.usd`
- character assets: `assets/characters/`
- UI / scenario file: `assets/people_initial_commands.yaml`
- runtime command scratch file: `/tmp/spot_isaac_people_runtime_commands.txt`

Launch it with:

```bash
./scripts/run_people_sim.sh
```

This opens the hospital scene with saved characters under `/World/Characters`,
bakes an Isaac Sim people navmesh, attaches the animation graph / behavior
scripts, and opens a small **People Control Test** window. The window lets you
select one character, enter `X`, `Y`, and `Yaw`, then trigger scenarios from
the YAML-defined buttons. Current examples include initialization, patrols,
seated people, talking pairs, look-around actions, and selected-character
`GoTo`.

The scene also contains manually added chair and wheelchair prims under
`/World/Chair`. These are intentional: seated character commands need stable
seat target prim paths such as `/World/Chair/SM_Chair_01a13`. Keep these
documented with the character scene because they are part of the scenario
authoring surface, not only visual decoration.

---

## Character Scenario Markup

`assets/people_initial_commands.yaml` is a small markup layer over
`omni.anim.people` command strings. It has two top-level keys:

- `buttons`: up to 9 UI buttons. Each button can provide `label`, `scenario`,
  and optional `action` (`reset` or `stop`).
- `scenarios`: named scenario definitions referenced by the buttons.

The underlying command-line format is:

```text
<character_name> <action> <arguments...> <duration_or_yaw>
```

Examples:

```text
Female_patient_05 Idle 9999
Female_visitor_04 LookAround 9999
Female_police_01 GoTo 19.2 30.89 0.0 0.0
Female_visitor_02 Sit /World/Chair/SM_Chair_02a7_01 9999
Male_nurse_04 Talk Male_nurse_02 9999
Female_nurse_05 TalkWith Male_patient_05 9999
```

Supported scenario nodes:

| Node type | YAML keys | Purpose |
| --------- | --------- | ------- |
| `command` | `command` | Run one people command for one character |
| `commands` | `commands` | Run a list of command strings, grouped by character |
| `wait` | `seconds` or `duration` | Pause a sequence before the next step |
| `sequence` | `steps` | Run child nodes in order |
| `parallel` | `children` | Run child scenario branches together |
| `repeat` | `count`, plus `steps`, `commands`, `command`, or `child` | Repeat a plan; `inf`, `infinite`, `forever`, or `0` mean endless |

Scenarios can target specific characters with a `characters` mapping:

```yaml
scenarios:
  set_patrol:
    label: "Set patrol"
    characters:
      Female_police_01:
        type: repeat
        count: inf
        steps:
          - type: command
            command: "Female_police_01 GoTo 19.2 30.89 0.0 0.0"
          - type: command
            command: "Female_police_01 GoTo 17.5 3.19 0.0 180.0"
```

The UI-selected character and target pose are available as template variables:
`{character}`, `{x}`, `{y}`, and `{r}`.

```yaml
selected_goto:
  label: "GoTo"
  characters:
    "{character}":
      type: command
      command: "{character} GoTo {x} {y} 0.0 {r}"
```

Use long durations such as `9999` for persistent idle, sit, talk, or
look-around states.

---

## ROS 2 Topics

Once `run_isaac.sh` or `run_spot_bridge_with_people.sh` is running, you can
see the ROS 2 topics and TF tree:

| Direction | Topic                          | Type                              |
| --------- | ------------------------------ | --------------------------------- |
| pub       | `/clock`                       | `rosgraph_msgs/Clock`             |
| pub       | `/odom`                        | `nav_msgs/Odometry`               |
| pub       | `/imu/data`                    | `sensor_msgs/Imu`                 |
| pub       | `/point_cloud`                 | `sensor_msgs/PointCloud2`         |
| pub       | `/front_camera/image`          | `sensor_msgs/Image`               |
| pub       | `/{left,right,back}_fisheye/image` | `sensor_msgs/Image`           |
| pub       | `/realsense/camera`            | `sensor_msgs/Image`               |
| pub       | `/realsense/depth/points`      | `sensor_msgs/PointCloud2`         |
| pub       | `/isaac_joint_states`          | `sensor_msgs/JointState` (raw)    |
| pub       | `/joint_states`                | `sensor_msgs/JointState` (URDF)   |
| pub       | `/tf`, `/tf_static`            | `tf2_msgs/TFMessage`              |
| sub       | `/cmd_vel`                     | `geometry_msgs/TwistStamped`      |

When RealSense is enabled, the color image and depth point cloud use the
actual discovered USD camera prim frame IDs. In the current scene, the depth
point cloud is typically published in `Camera_Pseudo_Depth`, not the generic
`realsense` mount frame. This avoids the 90-degree roll mismatch that happens
when a camera optical point cloud is visualized against a body-style mount TF.

### ROS / RViz captures

<img src="docs/photos/ros_topic/realsense_topic_rviz.png" width="800" title="RealSense color image and depth point cloud in RViz2"/>

<img src="docs/photos/ros_topic/costmap.png" width="800" title="RViz2 costmap for the hospital Nav2 setup"/>

[Point cloud RViz video](docs/photos/ros_topic/point_cloud_rviz.mp4)

### Camera streams

<table>
  <tr>
    <td><img src="docs/photos/ros_topic/front_camera.png" width="390" title="Front camera topic in RViz2"/></td>
    <td><img src="docs/photos/ros_topic/left_fisheye_camera.png" width="390" title="Left fisheye camera topic in RViz2"/></td>
  </tr>
  <tr>
    <td><img src="docs/photos/ros_topic/right_fisheye_camera.png" width="390" title="Right fisheye camera topic in RViz2"/></td>
    <td><img src="docs/photos/ros_topic/rear_fisheye_camera.png" width="390" title="Rear fisheye camera topic in RViz2"/></td>
  </tr>
</table>

### TF tree

<img src="docs/photos/main_readme/tf_frame.png" width="900" title="TF tree captured with `ros2 run tf2_tools view_frames` while the simulation was running"/>

---

## Sensors on Spot

The robot in `assets/isaac_hospital_scene_spot.usd` ships with a 3D LiDAR,
top RGB camera, IMU, odometry, RealSense D455i (toggleable) and three extra
fisheye RGB cameras (toggleable).

Mount poses are body-relative — `((x, y, z), (roll, pitch, yaw))` from the
`body` prim:

| Sensor          | Translation (m)         | RPY (rad)                                 |
| --------------- | ----------------------- | ----------------------------------------- |
| 3D LiDAR        | (0.223, 0.0, 0.1271)    | (0.0, 0.0, 0.0)                           |
| RealSense D455i | (0.45, 0.0, 0.07)       | (0.0, 0.872665, 0.0)  — 50° pitched down  |
| Top RGB camera  | (0.26, 0.0, 0.17)       | (0.0, 0.0, 0.0)                           |
| IMU             | (0.0, 0.0, 0.0)         | (0.0, 0.0, 0.0)                           |
| Left fisheye    | (-0.125, 0.12, 0.035)   | (0.0, 0.2, +π/2)                          |
| Right fisheye   | (-0.125, -0.12, 0.035)  | (0.0, 0.2, -π/2)                          |
| Back fisheye    | (-0.425, 0.0, 0.01)     | (0.0, 0.3, π)                             |

> The visual model and the virtual sensor pose can differ — you can run a
> sensor without a visible model, or have a model with no sensor.

### Modifying the world or sensor mounts

Open the USD in Isaac Sim:

```bash
./isaac-sim.selector.sh           # then File → Open
```

Pick `assets/isaac_hospital_scene_spot.usd` and save after editing. A short
walkthrough for editing sensor positions:
<https://www.youtube.com/watch?v=NV2hqw8wu3U>

The extra light setup is documented separately in
[`docs/MODIFY_WORLD.md`](docs/MODIFY_WORLD.md).

---

## Configuration

Most Spot-only parameters are at the top of
[`isaac_sim/spot_standalone.py`](isaac_sim/spot_standalone.py). The combined
Spot + people bridge mirrors the same Spot settings in
[`isaac_sim/spot_bridge_with_people.py`](isaac_sim/spot_bridge_with_people.py)
and adds people setup before the ROS 2 bridge is enabled:

| Variable                    | Type   | Purpose                                      |
| --------------------------- | ------ | -------------------------------------------- |
| `PHYSICS_NUM_THREADS`       | `int`  | PhysX worker threads. Keep ≥ 4; increase when enabling extra cameras |
| `CMD_SCALE`                 | `np.ndarray` | Maps `/cmd_vel` to RL policy command space |
| `CMD_VEL_STAMPED`           | `bool` | `True` for Nav2/ros2_control (TwistStamped)  |
| `ENABLE_FISHEYE_CAMERAS`    | `bool` | Add 3 body fisheye cameras (left/right/back). May slow simulation — increase `PHYSICS_NUM_THREADS` if needed |
| `ENABLE_REALSENSE`          | `bool` | Enable the RealSense D455 color/depth publishers. May slow simulation — increase `PHYSICS_NUM_THREADS` if needed |
| `ENABLE_LEG_TF`             | `bool` | Publish leg TFs from Isaac (off by default)  |
| `SENSOR_(SENSOR_NAME)_TRANS` / `_RPY` | `tuple` | TF mounts for laser, front cam, IMU, etc. |

For details about `scripts/run_isaac.sh` and `isaac_sim/spot_standalone.py`,
see [`docs/RUN_SIMULATION.md`](docs/RUN_SIMULATION.md).



---

## Repository layout

```
.
├── isaac_sim/                  # Isaac Sim standalone Python scripts
│   ├── spot_standalone.py      # main driver — loads scene + builds OmniGraph
│   ├── spot_bridge_with_people.py
│   │                              # combined Spot ROS 2 bridge + people simulation
│   ├── people_control_sim.py     # people-only scenario UI
│   ├── list_graphs.py          # debug: dump all OmniGraph nodes
│   ├── rtx_lidar.py            # debug: RTX lidar prim helper
│   └── export_tf_pose.py       # debug: dump TF tree for sanity-checking
├── assets/                     # USD scenes, character assets and command configs
│   ├── isaac_hospital_scene_spot.usd
│   ├── isaac_hospital_scene_spot_w_characters.usd
│   ├── people_initial_commands.yaml
│   ├── characters/
│   └── scene_positions.txt     # prim pose dump for world editing
├── ros2_ws/src/spot_hospital_bringup/
│   ├── urdf/spot.urdf          # Spot URDF (used by robot_state_publisher)
│   ├── maps/                   # Nav2 occupancy map (.yaml + .png)
│   └── launch/robot_state_publisher.launch.py
├── scripts/
│   ├── run_isaac.sh            # launch Isaac Sim with the standalone script
│   ├── run_spot_bridge_with_people.sh
│   │                              # launch combined v2.0 Spot + people bridge
│   ├── run_people_sim.sh       # launch the character-control scene and UI
│   ├── run_ros2.sh             # launch robot_state_publisher
│   └── dump_scene_positions.py # dump prim poses to /tmp/scene_positions.txt
├── docs/photos/                # screenshots used in this README
└── env/
    └── spot_isaac.env.template # copy → spot_isaac.env, edit for your machine
```

---

## Supporting Nav2 / RViz Demos

Nav2 is included as a validation and demo path rather than the core function
of this package. The package ships an occupancy map at
`ros2_ws/src/spot_hospital_bringup/maps/spot_hospital_map.{yaml,png}`. You can
launch a Nav2 stack against it with `use_sim_time:=true` and AMCL/SLAM
consuming `/point_cloud` (or via a `pointcloud_to_laserscan` adapter).

For simpler robot-control demos, use direct `/cmd_vel` input from
`teleop_twist_keyboard`, `rqt_robot_steering`, or a custom ROS 2 node. RViz2 is
useful for showing the costmap, TF tree, raw point cloud, RealSense color /
depth point cloud, and the camera streams while the robot moves through the
front desk area.

<img src="ros2_ws/src/spot_hospital_bringup/maps/spot_hospital_map.png" width="800" title="Nav2 occupancy map of the hospital environment"/>

---

## Troubleshooting

**Jittery sensor timestamps** — the physics step is falling behind real time.
Reduce `PHYSICS_NUM_THREADS` only if you have < 8 cores; otherwise try
increasing `physics_dt` from `1/500` toward `1/200`.

**`IsaacComputeOdometry` shape error** — extra `RigidBodyAPI`s are nested
under `/World/spot/body/`. The script strips these automatically; if you see
the error, check that the USD reference resolved correctly.

**Conflicting TF emitters** — Isaac and `robot_state_publisher` both publish
leg TFs by default. Keep `ENABLE_LEG_TF = False` and let
`robot_state_publisher` own the URDF kinematic tree.

**People are stuck in T-pose** — check the startup log from
`run_spot_bridge_with_people.sh`. You should see nonzero
`Registered people agents`. The combined bridge removes stale behavior scripts
from non-`SkelRoot` child prims and initializes only valid character roots.

**RealSense point cloud appears rolled or flipped** — use the camera frame from
the point cloud header, usually `Camera_Pseudo_Depth`, as the RViz fixed or
target frame. The generic `realsense` TF is a body-style mount frame kept for
compatibility, while the point cloud is emitted in the actual depth camera
frame.

**"Authoring to instance proxy not allowed"** — when adding lights or other
prims, write them under `/World/hospital/Inhouse_Light/` (outside the
instanced hospital reference), not inside `/World`.

---

## Media Still To Add

- [ ] GIF of `/cmd_vel` robot control with `rqt_robot_steering`
- [ ] GIF of Nav2 navigating through the front desk area

---

## License

Apache-2.0. The Spot URDF and Isaac Sim assets are subject to their
respective upstream licenses (NVIDIA Isaac Sim assets, Boston Dynamics Spot
description).

## Sponsor Me (With Thanks)

[![Sponsor SJoyTheHawk](https://img.shields.io/badge/Sponsor-%E2%9D%A4-pink?logo=github)](https://github.com/sponsors/SJoyTheHawk)
