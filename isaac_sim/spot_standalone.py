# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import math
import os

import carb
import carb.settings
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import define_prim
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom, UsdPhysics

# Enable ROS2 bridge before creating ROS2 OmniGraph nodes
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# rclpy must be imported AFTER the ROS2 bridge extension is enabled
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState

first_step = True
reset_needed = False
torch = import_module("torch")

# Scale factors mapping real-world cmd_vel (m/s, m/s, rad/s) to the policy's
# internal command space. SpotFlatTerrainPolicy was trained with vx/vy in
# roughly the 0.5-2.0 range and yaw_rate up to ~2.0 rad/s.
# Tune CMD_SCALE so that 1.0 m/s from ROS maps to a comfortable walking gait.
CMD_SCALE = np.array([1.44, 1.44, 1.480140971])  # [vx_scale, vy_scale, yaw_scale]

# Deadband threshold (m/s, m/s, rad/s). Below this, use CMD_IDLE instead.
CMD_DEADBAND = np.array([0.05, 0.05, 0.05])

# Minimum command injected when zero is received, keeps the policy in a stable
# walking gait rather than drifting/twisting from a dead-stop command.
CMD_IDLE = np.array([0.0, 0.0, 0.05])


# ---------------------------------------------------------------------------
# Sensor mount configuration (body-relative, URDF convention: +X fwd, +Y left,
# +Z up). All angles are in radians (URDF intrinsic XYZ rpy). These are the
# single source of truth for both ROS TF (body → <sensor>) and USD camera
# authoring of the new fisheye prims.
# ---------------------------------------------------------------------------
# Existing sensor mounts (TF only — the prims themselves already exist in USD).
SENSOR_LASER_TRANS = (0.223, 0.0, 0.1271)
SENSOR_LASER_RPY = (0.0, 0.0, 0.0)

SENSOR_FRONT_CAMERA_TRANS = (0.26, 0.0, 0.17)
SENSOR_FRONT_CAMERA_RPY = (0.0, 0.0, 0.0)

SENSOR_REALSENSE_TRANS = (0.45, 0.0, 0.07)
SENSOR_REALSENSE_RPY = (0.0, 0.872665, 0.0)

SENSOR_IMU_TRANS = (0.0, 0.0, 0.0)
SENSOR_IMU_RPY = (0.0, 0.0, 0.0)

# Feature toggles.
ENABLE_FISHEYE_CAMERAS = True
ENABLE_REALSENSE = False
FRONT_CAMERA_AS_FISHEYE = False
# Publish leg-link TFs from Isaac Sim. Set False when an external
# robot_state_publisher with the Spot URDF is running, otherwise the two will
# emit conflicting TFs (Isaac uses flat USD prim names, URDF nests them).
ENABLE_LEG_TF = False

# Number of PhysX CPU worker threads. Keep this >=4 to avoid the
# physics step falling behind real time, which would make /clock advance
# unevenly and produce jittery sensor timestamps downstream.
PHYSICS_NUM_THREADS = 12

PHYSICS_DT = 1 / 500
RENDERING_DT = 1 / 50

# Body-mounted fisheye cameras (left/right/back). Each tuple is:
#   (name, translation_xyz_m, rpy_rad, horizontal_fov_rad)
FISHEYE_CAMERA_SPECS = [
    ("left_fisheye", (-0.125, 0.12, 0.035), (0.0, 0.2, 1.5707963267948966), 1.78634),
    ("right_fisheye", (-0.125, -0.12, 0.035), (0.0, 0.2, -1.5707963267948966), 1.78634),
    ("back_fisheye", (-0.425, 0.0, 0.01), (0.0, 0.3, 3.1415926535897931), 1.78634),
]
FISHEYE_RES = (640, 480)
FISHEYE_FOCAL_LEN_MM = 1.4
FISHEYE_CLIP_RANGE = (0.05, 1000.0)
FRONT_CAMERA_RES = (1280, 720)
FRONT_CAMERA_FISHEYE_HFOV_RAD = 1.78634

# ---------------------------------------------------------------------------
# ROS2 topic names
# ---------------------------------------------------------------------------
TOPIC_CLOCK = "clock"
TOPIC_LIDAR_POINT_CLOUD = "point_cloud"
TOPIC_ODOM = "odom"
TOPIC_IMU = "imu/data"
TOPIC_TF_STATIC = "tf_static"
TOPIC_ISAAC_JOINT_STATES = "isaac_joint_states"
TOPIC_JOINT_STATES = "joint_states"
TOPIC_FRONT_CAMERA_IMAGE = "camera/rgb/image_raw"
TOPIC_REALSENSE_COLOR = "realsense/camera"
TOPIC_REALSENSE_DEPTH = "realsense/depth/points"
TOPIC_CMD_VEL = "cmd_vel"

# Set True to subscribe to geometry_msgs/TwistStamped instead of Twist.
# Nav2 / ros2_control publish TwistStamped by default in ROS 2 Jazzy.
CMD_VEL_STAMPED = True

# ---------------------------------------------------------------------------
# ROS2 / TF frame names
# ---------------------------------------------------------------------------
FRAME_ODOM = "odom"
FRAME_BASE_LINK = "base_link"
FRAME_BODY = "body"
FRAME_LASER = "laser"
FRAME_FRONT_CAMERA = "front_camera"
FRAME_IMU = "imu"
FRAME_REALSENSE = "realsense"

# USD prim paths.
SPOT_BODY_PRIM = "/World/spot/body"
LIDAR_CAMERA_PRIM = "/World/spot/body/XT_32/PandarXT_32_10hz"
FRONT_CAMERA_XFORM = "/World/spot/body/Camera_SG2_OX03CC_5200_GMSL2_H60YA"
REALSENSE_PRIM = "/World/spot/rsd455"
IMU_PRIM_PATH = SPOT_BODY_PRIM + "/imu_sensor"

# Discover the 16 leg link prims under /World/spot by name so TF_Articulation
# publishes only joint frames, not sensor prims like PandarXT_32_10hz.
LEG_PRIM_NAMES = {
    "fl_hip", "fl_uleg", "fl_lleg", "fl_foot",
    "fr_hip", "fr_uleg", "fr_lleg", "fr_foot",
    "hl_hip", "hl_uleg", "hl_lleg", "hl_foot",
    "hr_hip", "hr_uleg", "hr_lleg", "hr_foot",
}

# JointState remapper: Isaac publishes joint names like 'fl_hx', but the Spot
# URDF (used by robot_state_publisher) expects 'front_left_hip_x'.
LEG_PREFIX = {"fl": "front_left", "fr": "front_right", "hl": "rear_left", "hr": "rear_right"}
JOINT_SUFFIX = {"hx": "hip_x", "hy": "hip_y", "kn": "knee"}

# Optical-frame correction for USD cameras: maps URDF link frame (+X forward,
# +Y left, +Z up) to USD camera frame (-Z forward, +X right, +Y up).
# Quaternion derived from the basis change camera->link = [[0,0,-1],[-1,0,0],[0,1,0]].
OPTICAL_QUAT = (0.5, -0.5, -0.5, 0.5)


def _rpy_to_quat(rpy):
    """URDF rpy (radians) → quaternion (x, y, z, w). Order: R_z * R_y * R_x."""
    r, p, y = rpy
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def _quat_mul(q1, q2):
    """Hamilton product q1 ⊗ q2 (xyzw → xyzw)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def spot_policy_command(command):
    return torch.as_tensor(command, dtype=torch.float32, device=torch.device(str(spot.robot._device)))


# initialize robot on first step, run robot advance
def on_physics_step(step_size) -> None:
    global first_step
    global reset_needed
    if first_step:
        spot.initialize()
        first_step = False
    elif reset_needed:
        my_world.reset(True)
        reset_needed = False
        first_step = True
    else:
        if np.all(np.abs(base_command) < CMD_DEADBAND):
            # Hold a gentle idle command instead of pure zero to keep the policy stable
            spot.forward(step_size, spot_policy_command(CMD_IDLE))
        else:
            # Scale cmd_vel to the range the RL policy was trained on
            spot.forward(step_size, spot_policy_command(base_command * CMD_SCALE))


# spawn world
# Increase PhysX CPU thread count (default is usually 4). 0 = use all cores.
carb.settings.get_settings().set("/physics/numThreads", PHYSICS_NUM_THREADS)
print(f"[spot_standalone] PhysX threads: {carb.settings.get_settings().get('/physics/numThreads')}")
my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")

# spawn hospital scene
# Asset path resolution priority:
#   1. HOSPITAL_USD environment variable (set by scripts/run_isaac.sh)
#   2. Default: <repo>/assets/isaac_hospital_scene_spot.usd (relative to this file)
prim = define_prim("/World", "Xform")
_default_asset = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "assets", "isaac_hospital_scene_spot.usd",
)
asset_path = os.path.realpath(os.environ.get("HOSPITAL_USD", _default_asset))
print(f"[spot_standalone] Loading hospital scene from: {asset_path}")
prim.GetReferences().AddReference(asset_path)

# spawn robot
spot = SpotFlatTerrainPolicy(
    prim_path="/World/spot",
    position=np.array([0, 0, 0.8]),
)

# Strip extra RigidBodyAPI from any nested prim under Spot's body. Authoring of
# nested rigid bodies (e.g. an attached RSD455 camera) breaks PhysX tensors used
# by IsaacComputeOdometry — it returns shape (N, 6) instead of (1, 6).
# Removing the API turns those prims into plain xforms, keeping their poses but
# preventing PhysX from treating them as separate dynamic bodies.
_stage = omni.usd.get_context().get_stage()
for _p in _stage.Traverse():
    _path = str(_p.GetPath())
    if _path.startswith("/World/spot/body/") and _path != "/World/spot/body":
        if _p.HasAPI(UsdPhysics.RigidBodyAPI):
            print(f"[spot_standalone] Removing nested RigidBodyAPI from {_path}")
            _p.RemoveAPI(UsdPhysics.RigidBodyAPI)
            # Disable the rigid body flag too, in case the API removal isn't enough
            attr = _p.GetAttribute("physics:rigidBodyEnabled")
            if attr:
                attr.Set(False)

# Create three additional fisheye-style body cameras (left, right, back) to
# match Spot's real perception suite. The public spot_description URDF does not
# define these — Boston Dynamics ships them on hardware as monochrome 640x480
# fisheyes (~110° HFOV). We author them as USD Camera prims under /World/spot/body
# at body-relative offsets, then publish each via its own ROS2 camera graph.
#
# Author the additional fisheye camera prims (left/right/back) under
# /World/spot/body. Translation/rotation come from FISHEYE_CAMERA_SPECS at the
# top of the file. The USD camera is rotated by (link_rpy * optical_correction)
# so that the rendered view direction matches the URDF link's +X axis while the
# TF we publish (`body → <name>`) uses the link rotation alone.
_BODY_CAMERA_PRIMS = {}  # name → prim path
if ENABLE_FISHEYE_CAMERAS:
    for _name, _xyz, _rpy_rad, _hfov_rad in FISHEYE_CAMERA_SPECS:
        _cam_path = f"{SPOT_BODY_PRIM}/{_name}"
        _cam_prim = UsdGeom.Camera.Define(_stage, _cam_path)
        _xf = UsdGeom.Xformable(_cam_prim)
        _xf.ClearXformOpOrder()
        _xf.AddTranslateOp().Set(Gf.Vec3d(*_xyz))
        _link_q = _rpy_to_quat(_rpy_rad)
        _cam_q = _quat_mul(_link_q, OPTICAL_QUAT)
        _qx, _qy, _qz, _qw = _cam_q
        _xf.AddOrientOp().Set(Gf.Quatf(_qw, Gf.Vec3f(_qx, _qy, _qz)))
        # Aperture from HFOV: h_aperture = 2 * focal * tan(hfov/2)
        _h_ap = 2.0 * FISHEYE_FOCAL_LEN_MM * math.tan(_hfov_rad * 0.5)
        _v_ap = _h_ap * FISHEYE_RES[1] / FISHEYE_RES[0]
        _cam_prim.GetFocalLengthAttr().Set(FISHEYE_FOCAL_LEN_MM)
        _cam_prim.GetHorizontalApertureAttr().Set(_h_ap)
        _cam_prim.GetVerticalApertureAttr().Set(_v_ap)
        _cam_prim.GetClippingRangeAttr().Set(Gf.Vec2f(*FISHEYE_CLIP_RANGE))
        _BODY_CAMERA_PRIMS[_name] = _cam_path
        print(f"[spot_standalone] Created body camera {_cam_path}")
else:
    print("[spot_standalone] Fisheye cameras disabled (ENABLE_FISHEYE_CAMERAS=False)")

omni.kit.commands.execute(
    "IsaacSensorCreateImuSensor",
    path="/imu_sensor",
    parent=SPOT_BODY_PRIM,
    sensor_period=-1.0,  # -1 = every physics step
    translation=Gf.Vec3d(0, 0, 0),
    orientation=Gf.Quatd(1, 0, 0, 0),
)

# Build a ROS2 clock publisher action graph (publishes /clock from sim time)
try:
    og.Controller.edit(
        {"graph_path": "/World/ClockGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", TOPIC_CLOCK),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create clock graph: {e}")

def _find_camera_prims_under(root_path: str):
    """Find Camera prims under root_path.
    Accepts UsdGeom.Camera prims and any Isaac-specific type whose name contains 'camera'.
    """
    found = []
    for _p in _stage.Traverse():
        _pp = str(_p.GetPath())
        if not _pp.startswith(root_path + "/"):
            continue
        if UsdGeom.Camera(_p) or "camera" in _p.GetTypeName().lower():
            found.append(_pp)
    return found


def _frame_id_from_prim_path(prim_path: str, fallback: str) -> str:
    return prim_path.rsplit("/", 1)[-1] if prim_path else fallback


def _select_realsense_camera_prims(camera_prims: list[str]):
    if not camera_prims:
        carb.log_warn("[spot_standalone] No Camera prims found under rsd455 — RealSense graphs skipped")
        return None, None, FRAME_REALSENSE, FRAME_REALSENSE

    color_cams = [c for c in camera_prims if "color" in c.lower() or "rgb" in c.lower()]
    depth_cams = [c for c in camera_prims if "depth" in c.lower() or "ir" in c.lower()]
    color_cam = color_cams[0] if color_cams else camera_prims[0]
    depth_cam = depth_cams[0] if depth_cams else camera_prims[-1]
    color_frame = _frame_id_from_prim_path(color_cam, "realsense_color")
    depth_frame = _frame_id_from_prim_path(depth_cam, "realsense_depth")
    print(f"[spot_standalone] RealSense cameras: color={color_cam}, depth={depth_cam}")
    print(f"[spot_standalone] RealSense frames: color={color_frame}, depth={depth_frame}")
    return color_cam, depth_cam, color_frame, depth_frame


# Auto-discover the actual Camera prim inside the front camera xform.
# The outer xform and the inner Camera often share the same name.
_fc_cams = _find_camera_prims_under(FRONT_CAMERA_XFORM)
FRONT_CAMERA_PRIM = _fc_cams[0] if _fc_cams else FRONT_CAMERA_XFORM
print(f"[spot_standalone] Front camera prim: {FRONT_CAMERA_PRIM}")


def configure_front_camera_as_fisheye() -> None:
    prim = _stage.GetPrimAtPath(FRONT_CAMERA_PRIM)
    camera = UsdGeom.Camera(prim)
    if not camera:
        carb.log_warn(f"[spot_standalone] Cannot configure front camera as fisheye; not a USD Camera: {FRONT_CAMERA_PRIM}")
        return

    h_ap = 2.0 * FISHEYE_FOCAL_LEN_MM * math.tan(FRONT_CAMERA_FISHEYE_HFOV_RAD * 0.5)
    v_ap = h_ap * FRONT_CAMERA_RES[1] / FRONT_CAMERA_RES[0]
    camera.GetFocalLengthAttr().Set(FISHEYE_FOCAL_LEN_MM)
    camera.GetHorizontalApertureAttr().Set(h_ap)
    camera.GetVerticalApertureAttr().Set(v_ap)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*FISHEYE_CLIP_RANGE))
    print(f"[spot_standalone] Front camera configured as fisheye-style camera: {FRONT_CAMERA_PRIM}")


if FRONT_CAMERA_AS_FISHEYE:
    configure_front_camera_as_fisheye()

if ENABLE_REALSENSE:
    # Auto-discover RealSense Camera prims; try body-mounted path as fallback.
    _rs_root = REALSENSE_PRIM
    _rs_cams = _find_camera_prims_under(_rs_root)
    if not _rs_cams:
        _rs_root = "/World/spot/body/rsd455"
        _rs_cams = _find_camera_prims_under(_rs_root)
    if _rs_cams:
        print(f"[spot_standalone] RealSense camera prims found under {_rs_root}: {_rs_cams}")
    else:
        # Dump all descendant prim types so we can identify the right path/type
        for _candidate in [REALSENSE_PRIM, "/World/spot/body/rsd455"]:
            for _p in _stage.Traverse():
                _pp = str(_p.GetPath())
                if _pp.startswith(_candidate + "/"):
                    carb.log_warn(f"[spot_standalone] rsd455 child: {_pp}  type={_p.GetTypeName()}")
    REALSENSE_COLOR_CAM, REALSENSE_DEPTH_CAM, FRAME_REALSENSE_COLOR, FRAME_REALSENSE_DEPTH = _select_realsense_camera_prims(_rs_cams)
else:
    print("[spot_standalone] RealSense disabled (ENABLE_REALSENSE=False)")
    REALSENSE_COLOR_CAM, REALSENSE_DEPTH_CAM, FRAME_REALSENSE_COLOR, FRAME_REALSENSE_DEPTH = (
        None,
        None,
        FRAME_REALSENSE,
        FRAME_REALSENSE,
    )


LASER_TF_TRANS = list(SENSOR_LASER_TRANS)
LASER_TF_ROT = list(_rpy_to_quat(SENSOR_LASER_RPY))
FRONT_CAM_TF_TRANS = list(SENSOR_FRONT_CAMERA_TRANS)
FRONT_CAM_TF_ROT = list(_rpy_to_quat(SENSOR_FRONT_CAMERA_RPY))
REALSENSE_TF_TRANS = list(SENSOR_REALSENSE_TRANS)
REALSENSE_TF_ROT = list(_rpy_to_quat(SENSOR_REALSENSE_RPY))

IMU_TF_TRANS = list(SENSOR_IMU_TRANS)
IMU_TF_ROT = list(_rpy_to_quat(SENSOR_IMU_RPY))

# Body-relative TFs for the three additional fisheye cameras (link RPY only —
# no optical correction; the published TF describes the URDF-style link frame).
_BODY_CAMERA_TFS = {}
if ENABLE_FISHEYE_CAMERAS:
    _BODY_CAMERA_TFS = {
        _name: (list(_xyz), list(_rpy_to_quat(_rpy)))
        for _name, _xyz, _rpy, _hfov in FISHEYE_CAMERA_SPECS
    }

LEG_PRIMS = [
    str(_p.GetPath())
    for _p in _stage.Traverse()
    if str(_p.GetPath()).startswith("/World/spot/") and str(_p.GetPath()).split("/")[-1] in LEG_PRIM_NAMES
]
if not LEG_PRIMS:
    carb.log_warn("[spot_standalone] No leg prims found under /World/spot — check prim names in USD")
else:
    print(f"[spot_standalone] Found {len(LEG_PRIMS)} leg prims for TF: {LEG_PRIMS}")

try:
    og.Controller.edit(
        {"graph_path": "/World/LidarGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("PointCloudPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                # ("LaserScanPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [LIDAR_CAMERA_PRIM]),
                ("RenderProduct.inputs:enabled", True),
                ("RenderProduct.inputs:width", 1280),
                ("RenderProduct.inputs:height", 720),
                ("PointCloudPublish.inputs:topicName", TOPIC_LIDAR_POINT_CLOUD),
                ("PointCloudPublish.inputs:type", "point_cloud"),
                ("PointCloudPublish.inputs:frameId", FRAME_LASER),
                ("PointCloudPublish.inputs:fullScan", True),
                # Publish at 10 Hz: with rendering_dt=1/50 (50 Hz tick), skip 4 ticks between publishes
                ("PointCloudPublish.inputs:frameSkipCount", 4),
                # ("LaserScanPublish.inputs:topicName", "scan"),
                # ("LaserScanPublish.inputs:type", "laser_scan"),
                # ("LaserScanPublish.inputs:frameId", "laser"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "PointCloudPublish.inputs:execIn"),
                # ("RenderProduct.outputs:execOut", "LaserScanPublish.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "PointCloudPublish.inputs:renderProductPath"),
                # ("RenderProduct.outputs:renderProductPath", "LaserScanPublish.inputs:renderProductPath"),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create lidar graph: {e}")

# Odometry graph: computes body pose/velocity and publishes /odom topic + odom→base_link on /tf
# (ROS2PublishOdometry only emits the nav_msgs/Odometry message; the TF edge is published
# separately via ROS2PublishRawTransformTree fed from the same computed pose.)
try:
    og.Controller.edit(
        {"graph_path": "/World/OdomGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", [SPOT_BODY_PRIM]),
                ("PublishOdometry.inputs:topicName", TOPIC_ODOM),
                ("PublishOdometry.inputs:odomFrameId", FRAME_ODOM),
                ("PublishOdometry.inputs:chassisFrameId", FRAME_BASE_LINK),
                ("PublishOdomTF.inputs:parentFrameId", FRAME_ODOM),
                ("PublishOdomTF.inputs:childFrameId", FRAME_BASE_LINK),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create odometry graph: {e}")

# IMU graph: reads Isaac IMU sensor prim and publishes /imu
try:
    og.Controller.edit(
        {"graph_path": "/World/ImuGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadIMU.inputs:imuPrim", [IMU_PRIM_PATH]),
                ("PublishImu.inputs:topicName", TOPIC_IMU),
                ("PublishImu.inputs:frameId", FRAME_IMU),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishImu.inputs:execIn"),
                ("ReadIMU.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PublishImu.inputs:orientation"),
                ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create IMU graph: {e}")

# TF graph: publishes the full frame chain
#
#   odom → base_link      : from /odom topic published by the odometry node
#   base_link → body      : static identity
#   body → laser          : static transform read from USD at startup (direct, no intermediate lidar prim frame)
#   body → front_camera   : static transform read from USD at startup
#   body → realsense      : static transform read from USD at startup
#   body → fl_hip → ...   : dynamic articulation tree from TF_Articulation
#
try:
    _tf_create_nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        # Static: base_link → body (identity; frames are coincident)
        ("TF_BaseToBody", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        # Static: body → laser (lidar mount offset, no intermediate frame)
        ("TF_BodyToLaser", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        # Static: body → front_camera
        ("TF_BodyToFrontCamera", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        # Static: body → imu
        ("TF_BodyToImu", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
        # Dynamic: full Spot articulation tree (body → fl_hip/fr_hip/hl_hip/hr_hip → uleg → lleg → foot)
        ("TF_Articulation", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
    ]
    _tf_set_values = [
        ("TF_BaseToBody.inputs:parentFrameId", FRAME_BASE_LINK),
        ("TF_BaseToBody.inputs:childFrameId", FRAME_BODY),
        ("TF_BaseToBody.inputs:staticPublisher", True),
        ("TF_BaseToBody.inputs:topicName", TOPIC_TF_STATIC),
        ("TF_BodyToLaser.inputs:parentFrameId", FRAME_BODY),
        ("TF_BodyToLaser.inputs:childFrameId", FRAME_LASER),
        ("TF_BodyToLaser.inputs:translation", LASER_TF_TRANS),
        ("TF_BodyToLaser.inputs:rotation", LASER_TF_ROT),
        ("TF_BodyToLaser.inputs:staticPublisher", True),
        ("TF_BodyToLaser.inputs:topicName", TOPIC_TF_STATIC),
        ("TF_BodyToFrontCamera.inputs:parentFrameId", FRAME_BODY),
        ("TF_BodyToFrontCamera.inputs:childFrameId", FRAME_FRONT_CAMERA),
        ("TF_BodyToFrontCamera.inputs:translation", FRONT_CAM_TF_TRANS),
        ("TF_BodyToFrontCamera.inputs:rotation", FRONT_CAM_TF_ROT),
        ("TF_BodyToFrontCamera.inputs:staticPublisher", True),
        ("TF_BodyToFrontCamera.inputs:topicName", TOPIC_TF_STATIC),
        ("TF_BodyToImu.inputs:parentFrameId", FRAME_BODY),
        ("TF_BodyToImu.inputs:childFrameId", FRAME_IMU),
        ("TF_BodyToImu.inputs:translation", IMU_TF_TRANS),
        ("TF_BodyToImu.inputs:rotation", IMU_TF_ROT),
        ("TF_BodyToImu.inputs:staticPublisher", True),
        ("TF_BodyToImu.inputs:topicName", TOPIC_TF_STATIC),
        # Explicit leg prims only — avoids publishing sensor prim frames
        # (e.g. PandarXT_32_10hz) that are also children of body in USD.
        ("TF_Articulation.inputs:targetPrims", LEG_PRIMS),
        ("TF_Articulation.inputs:parentPrim", SPOT_BODY_PRIM),
    ]
    _tf_connect = [
        ("OnPlaybackTick.outputs:tick", "TF_BaseToBody.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "TF_BodyToLaser.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "TF_BodyToFrontCamera.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "TF_BodyToImu.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "TF_Articulation.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "TF_BaseToBody.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "TF_BodyToLaser.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "TF_BodyToFrontCamera.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "TF_BodyToImu.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "TF_Articulation.inputs:timeStamp"),
    ]
    if not ENABLE_LEG_TF:
        # Drop the articulation node entirely so this script doesn't conflict
        # with an external robot_state_publisher emitting joint TFs from URDF.
        _tf_create_nodes = [n for n in _tf_create_nodes if n[0] != "TF_Articulation"]
        _tf_set_values = [v for v in _tf_set_values if not v[0].startswith("TF_Articulation.")]
        _tf_connect = [c for c in _tf_connect if "TF_Articulation" not in c[0] and "TF_Articulation" not in c[1]]
        print("[spot_standalone] Leg TF disabled (ENABLE_LEG_TF=False) — relying on external robot_state_publisher")
    if ENABLE_REALSENSE:
        _tf_create_nodes.append(
            ("TF_BodyToRealsense", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")
        )
        _tf_set_values.extend([
            ("TF_BodyToRealsense.inputs:parentFrameId", FRAME_BODY),
            ("TF_BodyToRealsense.inputs:childFrameId", FRAME_REALSENSE),
            ("TF_BodyToRealsense.inputs:translation", REALSENSE_TF_TRANS),
            ("TF_BodyToRealsense.inputs:rotation", REALSENSE_TF_ROT),
            ("TF_BodyToRealsense.inputs:staticPublisher", True),
            ("TF_BodyToRealsense.inputs:topicName", TOPIC_TF_STATIC),
        ])
        _tf_connect.extend([
            ("OnPlaybackTick.outputs:tick", "TF_BodyToRealsense.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "TF_BodyToRealsense.inputs:timeStamp"),
        ])
        _realsense_camera_prims = []
        if REALSENSE_COLOR_CAM:
            _realsense_camera_prims.append(REALSENSE_COLOR_CAM)
        if REALSENSE_DEPTH_CAM and REALSENSE_DEPTH_CAM not in _realsense_camera_prims:
            _realsense_camera_prims.append(REALSENSE_DEPTH_CAM)
        if _realsense_camera_prims:
            _tf_create_nodes.append(
                ("TF_RealsenseCameras", "isaacsim.ros2.bridge.ROS2PublishTransformTree")
            )
            _tf_set_values.extend([
                ("TF_RealsenseCameras.inputs:targetPrims", _realsense_camera_prims),
                ("TF_RealsenseCameras.inputs:parentPrim", SPOT_BODY_PRIM),
                ("TF_RealsenseCameras.inputs:staticPublisher", True),
                ("TF_RealsenseCameras.inputs:topicName", TOPIC_TF_STATIC),
            ])
            _tf_connect.extend([
                ("OnPlaybackTick.outputs:tick", "TF_RealsenseCameras.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "TF_RealsenseCameras.inputs:timeStamp"),
            ])
            print(f"[spot_standalone] Publishing RealSense camera TFs: {_realsense_camera_prims}")
    og.Controller.edit(
        {"graph_path": "/World/TFGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: _tf_create_nodes,
            og.Controller.Keys.SET_VALUES: _tf_set_values,
            og.Controller.Keys.CONNECT: _tf_connect,
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create TF graph: {e}")


# Static TF graphs for the three additional fisheye cameras (left/right/back).
# One small graph per camera keeps the wiring uniform and easy to disable.
for _cam_name, (_trans, _rot) in _BODY_CAMERA_TFS.items():
    _node_id = f"TF_BodyTo{_cam_name.title().replace('_', '')}"
    _graph_path = f"/World/TF_{_cam_name}_Graph"
    try:
        og.Controller.edit(
            {"graph_path": _graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    (_node_id, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (f"{_node_id}.inputs:parentFrameId", FRAME_BODY),
                    (f"{_node_id}.inputs:childFrameId", _cam_name),
                    (f"{_node_id}.inputs:translation", _trans),
                    (f"{_node_id}.inputs:rotation", _rot),
                    (f"{_node_id}.inputs:staticPublisher", True),
                    (f"{_node_id}.inputs:topicName", TOPIC_TF_STATIC),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", f"{_node_id}.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", f"{_node_id}.inputs:timeStamp"),
                ],
            },
        )
    except Exception as e:
        carb.log_error(f"Failed to create TF graph for {_cam_name}: {e}")



# Joint state graph: publishes /joint_states (sensor_msgs/JointState) from
# the Spot articulation. Pair with `robot_state_publisher` + the Spot URDF
# externally to get the full kinematic TF tree (body → hip → uleg → lleg).
try:
    og.Controller.edit(
        {"graph_path": "/World/JointStateGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # Publish under /isaac_joint_states; an inline remapper below
                # republishes to /joint_states with URDF-matching names.
                ("PublishJointState.inputs:topicName", TOPIC_ISAAC_JOINT_STATES),
                ("PublishJointState.inputs:targetPrim", "/World/spot"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create JointState graph: {e}")


# Front camera graph: publishes /front_camera/image (RGB)
try:
    og.Controller.edit(
        {"graph_path": "/World/FrontCameraGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("RenderProduct.inputs:cameraPrim", [FRONT_CAMERA_PRIM]),
                ("RenderProduct.inputs:enabled", True),
                ("RenderProduct.inputs:width", FRONT_CAMERA_RES[0]),
                ("RenderProduct.inputs:height", FRONT_CAMERA_RES[1]),
                ("CameraHelper.inputs:topicName", TOPIC_FRONT_CAMERA_IMAGE),
                ("CameraHelper.inputs:type", "rgb"),
                ("CameraHelper.inputs:frameId", FRAME_FRONT_CAMERA),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut", "CameraHelper.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to create front camera graph: {e}")


# Side / back fisheye camera graphs (one per camera). Each publishes:
#   /<name>/image  (rgb)   — frameId = <name>
for _cam_name, _cam_path in _BODY_CAMERA_PRIMS.items():
    _graph_path = f"/World/{_cam_name}_Graph"
    try:
        og.Controller.edit(
            {"graph_path": _graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("CameraHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [_cam_path]),
                    ("RenderProduct.inputs:enabled", True),
                    ("RenderProduct.inputs:width", FISHEYE_RES[0]),
                    ("RenderProduct.inputs:height", FISHEYE_RES[1]),
                    ("CameraHelper.inputs:topicName", f"{_cam_name}/image"),
                    ("CameraHelper.inputs:type", "rgb"),
                    ("CameraHelper.inputs:frameId", _cam_name),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "CameraHelper.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "CameraHelper.inputs:renderProductPath"),
                ],
            },
        )
    except Exception as e:
        carb.log_error(f"Failed to create {_cam_name} graph: {e}")



# RealSense color graph: publishes /realsense/camera (RGB)
if REALSENSE_COLOR_CAM:
    try:
        og.Controller.edit(
            {"graph_path": "/World/RealsenseColorGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("ColorHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [REALSENSE_COLOR_CAM]),
                    ("RenderProduct.inputs:enabled", True),
                    ("RenderProduct.inputs:width", 640),
                    ("RenderProduct.inputs:height", 480),
                    ("ColorHelper.inputs:topicName", TOPIC_REALSENSE_COLOR),
                    ("ColorHelper.inputs:type", "rgb"),
                    ("ColorHelper.inputs:frameId", FRAME_REALSENSE_COLOR),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "ColorHelper.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "ColorHelper.inputs:renderProductPath"),
                ],
            },
        )
    except Exception as e:
        carb.log_error(f"Failed to create RealSense color graph: {e}")

# RealSense depth graph: publishes /realsense/depth (depth)
if REALSENSE_DEPTH_CAM:
    try:
        og.Controller.edit(
            {"graph_path": "/World/RealsenseDepthGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("DepthHelper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [REALSENSE_DEPTH_CAM]),
                    ("RenderProduct.inputs:enabled", True),
                    ("RenderProduct.inputs:width", 640),
                    ("RenderProduct.inputs:height", 480),
                    ("DepthHelper.inputs:topicName", TOPIC_REALSENSE_DEPTH),
                    ("DepthHelper.inputs:type", "depth_pcl"),
                    ("DepthHelper.inputs:frameId", FRAME_REALSENSE_DEPTH),
                    # 5 Hz: rendering_dt=1/50, skip 9 frames between publishes
                    ("DepthHelper.inputs:frameSkipCount", 9),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
                    ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "DepthHelper.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "DepthHelper.inputs:renderProductPath"),
                ],
            },
        )
    except Exception as e:
        carb.log_error(f"Failed to create RealSense depth graph: {e}")

my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# Initialize rclpy and subscribe to /cmd_vel to drive the robot
rclpy.init()
ros_node = rclpy.create_node("spot_cmd_vel_listener")

# robot command [vx, vy, yaw_rate]
base_command = np.zeros(3)


if CMD_VEL_STAMPED:
    def cmd_vel_callback(msg: TwistStamped) -> None:
        global base_command
        base_command = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z],
            dtype=np.float32,
        )
    _cmd_vel_msg_type = TwistStamped
else:
    def cmd_vel_callback(msg: Twist) -> None:
        global base_command
        base_command = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z],
            dtype=np.float32,
        )
    _cmd_vel_msg_type = Twist

cmd_vel_sub = ros_node.create_subscription(_cmd_vel_msg_type, TOPIC_CMD_VEL, cmd_vel_callback, 10)

def _remap_joint_name(name: str) -> str:
    parts = name.split("_")
    if len(parts) == 2 and parts[0] in LEG_PREFIX and parts[1] in JOINT_SUFFIX:
        return f"{LEG_PREFIX[parts[0]]}_{JOINT_SUFFIX[parts[1]]}"
    return name


joint_state_pub = ros_node.create_publisher(JointState, TOPIC_JOINT_STATES, 10)


def _isaac_joint_state_callback(msg: JointState) -> None:
    out = JointState()
    out.header = msg.header
    out.name = [_remap_joint_name(n) for n in msg.name]
    out.position = msg.position
    out.velocity = msg.velocity
    out.effort = msg.effort
    joint_state_pub.publish(out)


isaac_js_sub = ros_node.create_subscription(
    JointState, TOPIC_ISAAC_JOINT_STATES, _isaac_joint_state_callback, 10
)

while simulation_app.is_running():
    my_world.step(render=True)
    # Process incoming /cmd_vel messages every step (non-blocking)
    rclpy.spin_once(ros_node, timeout_sec=0.0)
    if my_world.is_stopped():
        reset_needed = True

# shutdown
ros_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
