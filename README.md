# spot-isaac-lab-hospital

Boston Dynamics **Spot** simulated in NVIDIA **Isaac Sim 4.5** inside a hospital
environment, exposing a full ROS 2 (Jazzy) interface for navigation, perception,
and control.

The robot is driven by Isaac Sim's `SpotFlatTerrainPolicy` (RL locomotion
policy) and accepts `/cmd_vel` (Twist or TwistStamped) from any ROS 2
controller — Nav2, `teleop_twist_keyboard`, custom planners, etc.

## Topics published / subscribed

| Direction | Topic                          | Type                              |
| --------- | ------------------------------ | --------------------------------- |
| pub       | `/clock`                       | `rosgraph_msgs/Clock`             |
| pub       | `/odom`                        | `nav_msgs/Odometry`               |
| pub       | `/imu/data`                    | `sensor_msgs/Imu`                 |
| pub       | `/point_cloud`                 | `sensor_msgs/PointCloud2`         |
| pub       | `/front_camera/image`          | `sensor_msgs/Image`               |
| pub       | `/isaac_joint_states`          | `sensor_msgs/JointState` (raw)    |
| pub       | `/joint_states`                | `sensor_msgs/JointState` (URDF)   |
| pub       | `/tf`, `/tf_static`            | `tf2_msgs/TFMessage`              |
| sub       | `/cmd_vel`                     | `geometry_msgs/TwistStamped`      |

Optional fisheye and RealSense camera publishers can be enabled via flags at
the top of [`isaac_sim/spot_standalone.py`](isaac_sim/spot_standalone.py).

## Repository layout

```
.
├── isaac_sim/                  # Isaac Sim standalone Python scripts
│   ├── spot_standalone.py      # main driver — loads scene + builds OmniGraph
│   ├── list_graphs.py          # debug: dump all OmniGraph nodes
│   ├── rtx_lidar.py            # debug: RTX lidar prim helper
│   └── export_tf_pose.py       # debug: dump TF tree for sanity-checking
├── assets/                     # USD scenes
│   ├── isaac_hospital_scene_spot.usd
│   └── spot_with_sensors.usd
├── ros2_ws/src/spot_hospital_bringup/
│   ├── urdf/spot.urdf          # Spot URDF (used by robot_state_publisher)
│   ├── maps/                   # Nav2 occupancy map (.yaml + .png)
│   └── launch/robot_state_publisher.launch.py
├── scripts/
│   ├── run_isaac.sh            # launch Isaac Sim with the standalone script
│   └── run_ros2.sh             # launch robot_state_publisher
└── env/
    └── spot_isaac.env.template # copy → spot_isaac.env, edit for your machine
```

## Prerequisites

- Ubuntu 24.04
- NVIDIA GPU with driver ≥ 535 (RTX series recommended)
- [Isaac Sim 4.5](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_workstation.html)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- `nav2` and `robot_state_publisher` ROS 2 packages:
  ```bash
  sudo apt install ros-jazzy-nav2-bringup ros-jazzy-robot-state-publisher
  ```

## Setup

```bash
# 1. Clone
git clone https://github.com/SJoyTheHawk/spot-isaac-lab-hospital.git
cd spot-isaac-lab-hospital

# 2. Configure for your machine
cp env/spot_isaac.env.template env/spot_isaac.env
$EDITOR env/spot_isaac.env       # set ISAAC_SIM_PATH if not ~/isaac-sim

# 3. Build the ROS 2 workspace
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select spot_hospital_bringup
cd ..
```

## Running

Open three terminals.

### Terminal 1 — Isaac Sim (the simulator + sensor publishers)

```bash
bash scripts/run_isaac.sh
```

This loads the hospital scene, spawns Spot, and starts publishing all sensor
topics. Wait for the viewport to render before launching the others.

### Terminal 2 — `robot_state_publisher` (URDF → TF)

```bash
bash scripts/run_ros2.sh
```

Publishes the dynamic leg-link TFs (`body → fl_hip → fl_uleg → ...`) from the
`/joint_states` topic emitted by Isaac.

### Terminal 3 — drive the robot

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

## Nav2

The package ships an occupancy map at
`ros2_ws/src/spot_hospital_bringup/maps/spot_hospital_map.{yaml,png}`. Launch a
Nav2 stack against it with `use_sim_time:=true` and AMCL/SLAM consuming
`/point_cloud` (or a `pointcloud_to_laserscan` adapter).

## Configuration

Most knobs live at the top of
[`isaac_sim/spot_standalone.py`](isaac_sim/spot_standalone.py):

| Variable                    | Purpose                                      |
| --------------------------- | -------------------------------------------- |
| `PHYSICS_NUM_THREADS`       | PhysX worker threads. Keep ≥ 4               |
| `CMD_SCALE`                 | Maps `/cmd_vel` to RL policy command space   |
| `CMD_VEL_STAMPED`           | `True` for Nav2/ros2_control (TwistStamped)  |
| `ENABLE_FISHEYE_CAMERAS`    | Add 3 body fisheye cameras (left/right/back) |
| `ENABLE_REALSENSE`          | Enable the RSD455 color/depth publishers     |
| `ENABLE_LEG_TF`             | Publish leg TFs from Isaac (off by default)  |
| `SENSOR_*_TRANS` / `_RPY`   | TF mounts for laser, front cam, IMU, etc.    |

The hospital USD path is resolved in this order:

1. `HOSPITAL_USD` env var (set in `env/spot_isaac.env`)
2. `<repo>/assets/isaac_hospital_scene_spot.usd` (default)

## Troubleshooting

**Jittery sensor timestamps** — the physics step is falling behind real time.
Reduce `PHYSICS_NUM_THREADS` only if you have < 8 cores; otherwise try
increasing `physics_dt` from `1/200` toward `1/100`.

**`IsaacComputeOdometry` shape error** — extra `RigidBodyAPI`s are nested under
`/World/spot/body/`. The script strips these automatically; if you see the
error, check that the USD reference resolved correctly.

**Conflicting TF emitters** — Isaac and `robot_state_publisher` both publish
leg TFs by default. Keep `ENABLE_LEG_TF = False` and let
`robot_state_publisher` own the URDF kinematic tree.

**"Authoring to instance proxy not allowed"** — when adding lights or other
prims, write them under `/World/Lights/` (outside the instanced hospital
reference), not inside `/World`.

## License

Apache-2.0. The Spot URDF and Isaac Sim assets are subject to their respective
upstream licenses (NVIDIA Isaac Sim assets, Boston Dynamics Spot description).
