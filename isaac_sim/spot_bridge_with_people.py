"""Standalone Spot ROS 2 bridge with Isaac people simulation.

This file is intentionally self-contained. It duplicates the required setup from
spot_standalone.py and people_control_sim.py so Isaac Sim starts one app, one
stage, one Spot ROS bridge, and one people simulation without importing either
standalone script.
"""

import math
import os
import random
import time

from isaacsim import SimulationApp


def _env_flag(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    try:
        return int(value)
    except ValueError:
        print(f"[spot_bridge_with_people] Ignoring invalid {name}={value!r}; using {default}.", flush=True)
        return default


def _env_optional_int(name: str) -> int | None:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return None
    try:
        return int(value)
    except ValueError:
        print(f"[spot_bridge_with_people] Ignoring invalid {name}={value!r}.", flush=True)
        return None


ISAAC_HEADLESS = _env_flag("SPOT_ISAAC_HEADLESS", False)
ISAAC_MULTI_GPU = _env_flag("SPOT_ISAAC_MULTI_GPU", False)
ISAAC_ANTI_ALIASING = _env_int("SPOT_ISAAC_ANTI_ALIASING", 1)
ISAAC_RENDERER = os.environ.get("SPOT_ISAAC_RENDERER", "").strip() or "RaytracedLighting"
ISAAC_CREATE_NEW_STAGE = _env_flag("SPOT_ISAAC_CREATE_NEW_STAGE", False)

ISAAC_LAUNCH_CONFIG = {
    "headless": ISAAC_HEADLESS,
    "multi_gpu": ISAAC_MULTI_GPU,
    "renderer": ISAAC_RENDERER,
    "anti_aliasing": ISAAC_ANTI_ALIASING,
    "create_new_stage": ISAAC_CREATE_NEW_STAGE,
    "width": _env_int("SPOT_ISAAC_WIDTH", 1280),
    "height": _env_int("SPOT_ISAAC_HEIGHT", 720),
    "window_width": _env_int("SPOT_ISAAC_WINDOW_WIDTH", 1440),
    "window_height": _env_int("SPOT_ISAAC_WINDOW_HEIGHT", 900),
    "extra_args": [
        f"--/rtx/post/aa/op={ISAAC_ANTI_ALIASING}",
        f"--/rtx-defaults/post/aa/op={ISAAC_ANTI_ALIASING}",
        f"--/rtx/rendermode={ISAAC_RENDERER}",
        f"--/rtx-defaults/rendermode={ISAAC_RENDERER}",
    ],
}

if _env_flag("SPOT_ISAAC_DISABLE_STARTUP_VIEWPORT", ISAAC_HEADLESS):
    ISAAC_LAUNCH_CONFIG["extra_args"].extend(
        [
            "--/exts/omni.kit.viewport.window/startup/disableWindowOnLoad=true",
            "--/exts/omni.kit.viewport.window/startup/showOnLaunch=[]",
            "--/exts/omni.kit.widget.viewport/autoAttach/mode=0",
        ]
    )

if not ISAAC_MULTI_GPU:
    ISAAC_LAUNCH_CONFIG["max_gpu_count"] = 1

ISAAC_MAX_GPU_COUNT = _env_optional_int("SPOT_ISAAC_MAX_GPU_COUNT")
if ISAAC_MAX_GPU_COUNT is not None:
    ISAAC_LAUNCH_CONFIG["max_gpu_count"] = ISAAC_MAX_GPU_COUNT

print(
    "[spot_bridge_with_people] Isaac launch: "
    f"headless={ISAAC_HEADLESS}, renderer={ISAAC_RENDERER}, "
    f"anti_aliasing={ISAAC_ANTI_ALIASING}, multi_gpu={ISAAC_MULTI_GPU}, "
    f"create_new_stage={ISAAC_CREATE_NEW_STAGE}, "
    f"max_gpu_count={ISAAC_LAUNCH_CONFIG.get('max_gpu_count', 'all')}",
    flush=True,
)

simulation_app = SimulationApp(ISAAC_LAUNCH_CONFIG)

import carb
import carb.settings
import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
import yaml
from isaacsim.core.api import World
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from omni.behavior.scripting.core.scripts.script_manager import ScriptManager
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel


REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_PEOPLE_USD = os.path.join(REPO_DIR, "assets", "isaac_hospital_scene_spot_w_characters_6.usd")
USD_PATH = os.path.realpath(os.environ.get("SPOT_PEOPLE_USD", DEFAULT_PEOPLE_USD))
COMMANDS_YAML_FILE = os.path.realpath(
    os.environ.get("PEOPLE_INITIAL_COMMANDS", os.path.join(REPO_DIR, "assets", "people_initial_commands.yaml"))
)
PEOPLE_COMMAND_FILE = os.path.realpath(
    os.environ.get("PEOPLE_COMMAND_FILE", os.path.join("/tmp", "spot_isaac_people_runtime_commands.txt"))
)
PEOPLE_COMMAND_FILE_IS_DEFAULT = PEOPLE_COMMAND_FILE == os.path.realpath(
    os.path.join("/tmp", "spot_isaac_people_runtime_commands.txt")
)
CHARACTER_ROOT = "/World/Characters"
MOTION_LIBRARY_PRIM_PATH = f"{CHARACTER_ROOT}/HumanMotionLibrary"
LOOK_AT_DEFAULT_DURATION = 8.0
LOOK_AT_DEFAULT_RADIUS = 4.0
LOOK_AT_DEFAULT_HEIGHT = 1.45

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
# single source of truth for both ROS TF (body -> <sensor>) and USD camera
# authoring of the new fisheye prims.
# ---------------------------------------------------------------------------
# Existing sensor mounts (TF only; the prims themselves already exist in USD).
SENSOR_LASER_TRANS = (0.223, 0.0, 0.1271)
SENSOR_LASER_RPY = (0.0, 0.0, 0.0)

SENSOR_FRONT_CAMERA_TRANS = (0.26, 0.0, 0.17)
SENSOR_FRONT_CAMERA_RPY = (0.0, 0.0, 0.0)

SENSOR_REALSENSE_TRANS = (0.45, 0.0, 0.07)
SENSOR_REALSENSE_RPY = (0.0, 0.872665, 0.0)

SENSOR_IMU_TRANS = (0.0, 0.0, 0.0)
SENSOR_IMU_RPY = (0.0, 0.0, 0.0)

# Feature toggles.
ENABLE_FISHEYE_CAMERAS = False
ENABLE_REALSENSE = False
FRONT_CAMERA_AS_FISHEYE = True
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
FRONT_CAMERA_RES = (640, 480)
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

STATUS_LABEL = None
LAST_STATUS_LOG_TIME = 0.0
SCENARIO_RUNNER = None
torch = import_module("torch")


def enable_people_extensions() -> None:
    extensions = [
        "omni.behavior.scripting.core",
        "omni.anim.behavior.core",
        "omni.anim.graph.core",
        "omni.anim.retarget.core",
        "omni.anim.navigation.core",
        "isaacsim.replicator.agent.core",
        "omni.kit.mesh.raycast",
    ]
    if _env_flag("SPOT_ISAAC_ENABLE_ANIM_TIMELINE", False):
        extensions.insert(1, "omni.anim.timeline")

    for ext in extensions:
        print(f"[spot_bridge_with_people] Enabling extension: {ext}", flush=True)
        enable_extension(ext)
    simulation_app.update()


def open_stage() -> None:
    print(f"[people_control_test] Loading: {USD_PATH}")
    if not omni.usd.get_context().open_stage(USD_PATH):
        raise RuntimeError(USD_PATH)
    for _ in range(8):
        simulation_app.update()


def strip_nested_rigid_bodies() -> None:
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith("/World/spot/body/") and path != "/World/spot/body":
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                prim.GetAttribute("physics:rigidBodyEnabled").Set(False)


def ensure_people_command_file() -> None:
    if not PEOPLE_COMMAND_FILE or "://" in PEOPLE_COMMAND_FILE:
        return
    os.makedirs(os.path.dirname(PEOPLE_COMMAND_FILE), exist_ok=True)
    mode = "w" if PEOPLE_COMMAND_FILE_IS_DEFAULT else "a"
    with open(PEOPLE_COMMAND_FILE, mode, encoding="utf-8"):
        pass


def configure_people() -> None:
    ensure_people_command_file()
    settings = carb.settings.get_settings()
    settings.set("/exts/isaacsim.replicator.agent/characters_parent_prim_path", CHARACTER_ROOT)


def bake_navmesh() -> None:
    import omni.anim.navigation.core as nav

    print("[people_control_test] Baking navmesh...")
    nav_interface = nav.acquire_interface()
    bake_fn = getattr(nav_interface, "start_navmesh_baking_and_wait", None)
    if bake_fn is None:
        bake_fn = getattr(nav_interface, "startNavMeshBakingAndWait", None)
    if bake_fn is None:
        print("[people_control_test] Navmesh bake API not found; using existing navmesh if available.")
        return

    if not bake_fn():
        print("[people_control_test] Navmesh bake failed. GoTo commands may be rejected.")
        return

    print("[people_control_test] Navmesh ready.")
    for _ in range(3):
        simulation_app.update()


def load_people() -> list[str]:
    root = omni.usd.get_context().get_stage().GetPrimAtPath(CHARACTER_ROOT)
    if not root.IsValid():
        return []

    skelroots = get_people_skelroots()
    if skelroots:
        return sorted(character_display_name(prim) for prim in skelroots)

    return [child.GetName() for child in root.GetAllChildren() if child.GetName() != "Biped_Setup"]


def get_people_skelroots() -> list:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return []

    root = stage.GetPrimAtPath(CHARACTER_ROOT)
    if not root.IsValid():
        return []

    skelroots = []
    for prim in Usd.PrimRange(root):
        if prim == root:
            continue
        prim_path = str(prim.GetPath())
        if "/Biped_Setup/" in prim_path:
            continue
        if prim.IsA(UsdSkel.Root) or prim.GetTypeName() == "SkelRoot":
            skelroots.append(prim)

    return skelroots


def get_people_skelroot_paths() -> set[str]:
    return {str(prim.GetPath()) for prim in get_people_skelroots()}


def character_display_name(skelroot_prim) -> str:
    try:
        parent = skelroot_prim.GetParent()
        if parent and parent.IsValid() and str(parent.GetPath()).startswith(f"{CHARACTER_ROOT}/"):
            return parent.GetName()
    except Exception:
        pass
    return skelroot_prim.GetName()


def destroy_script_manager_entry(script_manager, prim_path: str) -> int:
    removed = 0
    scripts = dict(script_manager._prim_to_scripts.get(prim_path, {}))
    for script_path, script_instance in scripts.items():
        if script_instance:
            script_manager._destroy_script_instance(prim_path, script_path, script_instance)

        current_scripts = script_manager._prim_to_scripts.get(prim_path)
        if current_scripts is not None:
            current_scripts.pop(script_path, None)
            if not current_scripts:
                script_manager._prim_to_scripts.pop(prim_path, None)

        prim_paths = script_manager._script_to_prims.get(script_path)
        if prim_paths is not None:
            prim_paths.discard(prim_path)
            if not prim_paths:
                script_manager._unload_script(script_path)
                script_manager._script_to_prims.pop(script_path, None)

        removed += 1
    return removed


def remove_stale_script_manager_people_instances(valid_paths: set[str]) -> int:
    script_manager = ScriptManager.get_instance()
    if script_manager is None:
        return 0

    stale_paths = [
        path
        for path in list(script_manager._prim_to_scripts.keys())
        if path.startswith(f"{CHARACTER_ROOT}/") and path not in valid_paths
    ]
    if not stale_paths:
        return 0

    removed = 0
    print(f"[spot_bridge_with_people] Stale ScriptManager people script instances: {len(stale_paths)}")
    print(f"[spot_bridge_with_people] First stale ScriptManager people script: {stale_paths[0]}")
    for path in stale_paths:
        removed += destroy_script_manager_entry(script_manager, path)
    return removed


def setup_saved_characters() -> None:
    try:
        from isaacsim.replicator.agent.core.settings import BehaviorScriptPaths
        from isaacsim.replicator.agent.core.stage_util import CharacterUtil
    except ModuleNotFoundError:
        print("[spot_bridge_with_people] Legacy character setup API is not available; using Isaac 6 authored characters.")
        return

    biped_prim = CharacterUtil.load_default_biped_to_stage()
    anim_graph = CharacterUtil.get_anim_graph_from_character(biped_prim)
    skelroots = get_people_skelroots()
    CharacterUtil.setup_animation_graph_to_character(skelroots, anim_graph)
    CharacterUtil.setup_python_scripts_to_character(skelroots, BehaviorScriptPaths.behavior_script_path())
    for _ in range(15):
        simulation_app.update()


def default_human_motion_library_asset() -> str:
    settings = carb.settings.get_settings()
    asset = settings.get("/exts/isaacsim.replicator.agent/default_human_motion_library_asset")
    if asset:
        return asset

    try:
        from omni.metropolis.utils.isaac_sim_util import resolve_asset_path
    except Exception:
        return "Isaac/People/MotionLibrary/HumanMotionLibrary.usd"

    return resolve_asset_path("Isaac/People/MotionLibrary/HumanMotionLibrary.usd")


def ensure_behavior_motion_library() -> Sdf.Path | None:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return None

    motion_library_path = Sdf.Path(MOTION_LIBRARY_PRIM_PATH)
    prim = stage.GetPrimAtPath(motion_library_path)
    if prim and prim.IsValid():
        return motion_library_path

    asset = default_human_motion_library_asset()
    print(f"[spot_bridge_with_people] Creating Isaac 6 behavior motion library at {MOTION_LIBRARY_PRIM_PATH}")
    try:
        omni.kit.commands.execute(
            "CreatePayloadCommand",
            usd_context=omni.usd.get_context(),
            path_to=motion_library_path,
            asset_path=asset,
            prim_path=None,
            instanceable=False,
            select_prim=False,
        )
    except Exception as exc:
        print(f"[spot_bridge_with_people] Failed to create behavior motion library from {asset}: {exc}")
        return None

    for _ in range(5):
        simulation_app.update()

    prim = stage.GetPrimAtPath(motion_library_path)
    return motion_library_path if prim and prim.IsValid() else None


def ensure_behavior_agents() -> None:
    skelroots = get_people_skelroots()
    if not skelroots:
        print("[spot_bridge_with_people] No character SkelRoots found for Isaac 6 BehaviorAgentAPI setup.")
        return

    motion_library_path = ensure_behavior_motion_library()
    if motion_library_path is None:
        print("[spot_bridge_with_people] Behavior motion library unavailable; LookAt may report missing agents.")
        return

    try:
        import BehaviorSchema

        missing = [prim for prim in skelroots if not prim.HasAPI(BehaviorSchema.BehaviorAgentAPI)]
    except Exception:
        missing = skelroots

    if not missing:
        print(f"[spot_bridge_with_people] Isaac 6 BehaviorAgentAPI already present on {len(skelroots)} characters.")
        return

    try:
        omni.kit.commands.execute(
            "ApplyBehaviorAgentAPICommand",
            skelroot_prim_paths=[prim.GetPath() for prim in missing],
            motion_library_prim_path=motion_library_path,
            motion_library_skeleton_rig="Human",
        )
    except Exception as exc:
        print(f"[spot_bridge_with_people] Failed to apply Isaac 6 BehaviorAgentAPI: {exc}")
        return

    for _ in range(10):
        simulation_app.update()

    print(
        "[spot_bridge_with_people] Applied Isaac 6 BehaviorAgentAPI: "
        f"{len(missing)}/{len(skelroots)} characters"
    )


def init_behavior_scripts() -> None:
    script_manager = ScriptManager.get_instance()
    if script_manager is None or script_manager._stage is None:
        print("[spot_bridge_with_people] ScriptManager is not ready for people behavior init.")
        return

    agent_manager = get_legacy_agent_manager()
    if agent_manager is None:
        print("[spot_bridge_with_people] Legacy Replicator AgentManager is not available; skipping script command registration.")
        return

    skelroot_paths = sorted(get_people_skelroot_paths())
    if not skelroot_paths:
        print("[spot_bridge_with_people] No people SkelRoots found for behavior init.")
        return

    script_manager._allow_scripts_to_execute = True
    for path in skelroot_paths:
        prim = script_manager._stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            script_manager._apply_scripts(prim)

    for _ in range(50):
        simulation_app.update()

    remove_stale_script_manager_people_instances(set(skelroot_paths))

    initialized = 0
    missing = []
    for path in skelroot_paths:
        scripts = script_manager._prim_to_scripts.get(path, {})
        behavior_scripts = [(script_path, inst) for script_path, inst in scripts.items() if inst and hasattr(inst, "init_character")]
        if not behavior_scripts:
            missing.append(path)
            continue
        for _, inst in behavior_scripts:
            try:
                inst.on_play()
                initialized_ok = inst.init_character()
            except Exception as exc:
                print(f"[spot_bridge_with_people] Failed to initialize people script on {path}: {exc}")
                continue
            if initialized_ok:
                agent_manager.register_agent(inst.get_agent_name(), inst.prim_path)
                initialized += 1

    print(f"[spot_bridge_with_people] Initialized people behavior scripts: {initialized}/{len(skelroot_paths)}")
    if missing:
        print(f"[spot_bridge_with_people] SkelRoots missing behavior script instances: {len(missing)}")
        print(f"[spot_bridge_with_people] First missing behavior script instance: {missing[0]}")


def load_yaml_config() -> dict:
    with open(COMMANDS_YAML_FILE, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def normalize_command_line(line: str) -> str:
    parts = line.split()
    if len(parts) < 2:
        return line.strip()
    command = parts[1].upper()
    if command == "IDLE":
        parts[1] = "Idle"
    elif command in {"LOOKAROUND", "LOOK_AROUND"}:
        parts[1] = "LookAround"
    elif command == "GOTO":
        parts[1] = "GoTo"
    if parts[1] == "GoTo" and len(parts) == 5:
        parts.append("_")
    return " ".join(parts)


def command_agent_name(line: str) -> str:
    parts = line.split()
    return parts[0] if parts else ""


def command_type(line: str) -> str:
    parts = normalize_command_line(line).split()
    return parts[1] if len(parts) > 1 else ""


def format_template(value, variables: dict | None = None) -> str:
    text = str(value)
    if not variables:
        return text
    try:
        return text.format(**variables)
    except KeyError as exc:
        print(f"[people_control_test] Missing template variable {exc} in: {text}")
        return text


def combo_box_index(model) -> int:
    value_model = model.get_item_value_model()
    if hasattr(value_model, "get_value_as_int"):
        return value_model.get_value_as_int()
    return value_model.as_int if hasattr(value_model, "as_int") else -1


def get_agent(name: str):
    agent_manager = get_legacy_agent_manager()
    if agent_manager is None:
        return None

    return agent_manager.get_agent_script_instance_by_name(name)


def get_legacy_agent_manager():
    try:
        from isaacsim.replicator.agent.core.agent_manager import AgentManager
    except ModuleNotFoundError:
        return None
    return AgentManager.get_instance()


def random_look_at_target(agent, radius: float, height: float) -> tuple[float, float, float] | None:
    try:
        pos = agent.get_world_translation()
    except Exception:
        return None

    angle = random.uniform(0.0, math.tau)
    distance = random.uniform(max(0.5, radius * 0.45), max(0.5, radius))
    z_offset = random.uniform(-0.25, 0.35)
    return (
        float(pos[0]) + math.cos(angle) * distance,
        float(pos[1]) + math.sin(angle) * distance,
        float(pos[2]) + height + z_offset,
    )


def look_at_all_characters(duration: float = LOOK_AT_DEFAULT_DURATION, radius: float = LOOK_AT_DEFAULT_RADIUS) -> None:
    import omni.anim.behavior.core as bh_core

    behavior = bh_core.acquire_interface()
    skelroots = sorted(get_people_skelroots(), key=lambda prim: str(prim.GetPath()))
    if not skelroots:
        print("[people_control_test] No character SkelRoots found for LookAt.")
        return

    invalid_task_id = getattr(bh_core, "BEHAVIOR_TASK_ID_INVALID", -1)
    started = 0
    missing_agents = []
    failed = []

    for skelroot in skelroots:
        path = str(skelroot.GetPath())
        agent = behavior.get_agent(path)
        if agent is None:
            missing_agents.append(path)
            continue

        target = random_look_at_target(agent, radius, LOOK_AT_DEFAULT_HEIGHT)
        if target is None:
            failed.append(path)
            continue

        try:
            task_id = agent.look_at(target=target, duration=duration)
        except Exception as exc:
            print(f"[people_control_test] LookAt failed for {path}: {exc}")
            failed.append(path)
            continue

        if task_id == invalid_task_id:
            failed.append(path)
            continue
        started += 1

    print(
        "[people_control_test] LookAt all characters: "
        f"started={started}, missing_agents={len(missing_agents)}, failed={len(failed)}, total={len(skelroots)}"
    )
    if missing_agents:
        print(f"[people_control_test] First LookAt missing behavior agent: {missing_agents[0]}")
    if failed:
        print(f"[people_control_test] First LookAt failed character: {failed[0]}")

    if STATUS_LABEL:
        STATUS_LABEL.text = f"LookAt all: {started}/{len(skelroots)}"


def agent_command_done(agent) -> bool:
    return agent is not None and agent.current_command is None and not agent.commands


def parse_repeat_count(value):
    if value is None:
        return 1
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"inf", "infinite", "forever"}:
            return "inf"
        value = int(normalized)
    value = int(value)
    return "inf" if value == 0 else max(1, value)


class PlanNode:
    def tick(self, controller) -> bool:
        return True

    def status(self) -> str:
        return "done"


class CommandNode(PlanNode):
    def __init__(self, command: str):
        self.command = str(command)
        self.started = False
        self.started_at = None
        self.expected_command_name = ""
        self.command_text = ""

    def tick(self, controller) -> bool:
        agent = get_agent(controller.character_name)
        if agent is None:
            return False

        if not self.started:
            self.command_text = controller.render_command(self.command)
            agent_name = command_agent_name(self.command_text)
            if agent_name and agent_name != controller.character_name:
                print(
                    f"[people_control_test] {controller.character_name} controller received command for {agent_name}; releasing."
                )
                controller.released = True
                return True

            self.expected_command_name = command_type(self.command_text)
            print(f"[people_control_test] {controller.character_name} -> {self.command_text}")
            agent.replace_command([self.command_text])
            self.started = True
            self.started_at = time.monotonic()
            return False

        if agent.current_command is not None and self.expected_command_name:
            running_name = agent.current_command.get_command_name()
            if running_name != self.expected_command_name and time.monotonic() - self.started_at > 0.5:
                print(
                    f"[people_control_test] {controller.character_name} switched to {running_name}; releasing controller."
                )
                controller.released = True
                return True

        return agent_command_done(agent)

    def status(self) -> str:
        command = self.expected_command_name or command_type(self.command)
        return f"command={command}" if command else "command"


class WaitNode(PlanNode):
    def __init__(self, seconds):
        self.seconds = max(0.0, float(seconds))
        self.started_at = None

    def tick(self, controller) -> bool:
        if self.started_at is None:
            self.started_at = time.monotonic()
        return time.monotonic() - self.started_at >= self.seconds

    def status(self) -> str:
        if self.started_at is None:
            return f"wait={self.seconds:g}s"
        remaining = max(0.0, self.seconds - (time.monotonic() - self.started_at))
        return f"wait={remaining:.1f}s"


class SequenceNode(PlanNode):
    def __init__(self, children: list[PlanNode]):
        self.children = children
        self.index = 0

    def tick(self, controller) -> bool:
        while self.index < len(self.children):
            if not self.children[self.index].tick(controller):
                return False
            self.index += 1
        return True

    def status(self) -> str:
        if not self.children:
            return "sequence=done"
        return f"step={min(self.index + 1, len(self.children))}/{len(self.children)}"


class ParallelNode(PlanNode):
    def __init__(self, children: list[PlanNode]):
        self.children = children
        self.done_indexes: set[int] = set()

    def tick(self, controller) -> bool:
        for index, child in enumerate(self.children):
            if index in self.done_indexes:
                continue
            if child.tick(controller):
                self.done_indexes.add(index)
        return len(self.done_indexes) == len(self.children)

    def status(self) -> str:
        return f"parallel={len(self.done_indexes)}/{len(self.children)}"


class RepeatNode(PlanNode):
    def __init__(self, child_spec, count):
        self.child_spec = child_spec
        self.count = parse_repeat_count(count)
        self.completed = 0
        self.child = make_plan_node(child_spec)

    def tick(self, controller) -> bool:
        if self.count != "inf" and self.completed >= self.count:
            return True
        if not self.child.tick(controller):
            return False

        self.completed += 1
        if self.count != "inf" and self.completed >= self.count:
            return True

        self.child = make_plan_node(self.child_spec)
        return False

    def status(self) -> str:
        if self.count == "inf":
            return f"loop={self.completed + 1}/inf"
        return f"loop={min(self.completed + 1, self.count)}/{self.count}"


def make_plan_node(spec) -> PlanNode:
    if isinstance(spec, str):
        return CommandNode(spec)
    if isinstance(spec, list):
        return SequenceNode([make_plan_node(step) for step in spec])
    if not isinstance(spec, dict):
        return PlanNode()

    node_type = str(spec.get("type", "")).lower()
    if not node_type:
        if "command" in spec:
            node_type = "command"
        elif "commands" in spec:
            node_type = "commands"
        elif "steps" in spec:
            node_type = "sequence"
        elif "children" in spec:
            node_type = "parallel"

    if node_type == "command":
        return CommandNode(str(spec.get("command", "")))
    if node_type == "commands":
        return SequenceNode([CommandNode(command) for command in spec.get("commands", [])])
    if node_type == "wait":
        return WaitNode(spec.get("seconds", spec.get("duration", 0.0)))
    if node_type == "sequence":
        return SequenceNode([make_plan_node(step) for step in spec.get("steps", [])])
    if node_type == "parallel":
        return ParallelNode([make_plan_node(child) for child in spec.get("children", [])])
    if node_type == "repeat":
        child_spec = spec.get("child")
        if child_spec is None:
            if "steps" in spec:
                child_spec = {"type": "sequence", "steps": spec.get("steps", [])}
            elif "commands" in spec:
                child_spec = {"type": "commands", "commands": spec.get("commands", [])}
            else:
                child_spec = spec.get("command", "")
        return RepeatNode(child_spec, spec.get("count", 1))

    print(f"[people_control_test] Unknown scenario node type: {node_type}")
    return PlanNode()


class CharacterController:
    def __init__(self, character_name: str, plan_spec, variables: dict | None = None, label: str = ""):
        self.character_name = character_name
        self.plan_spec = plan_spec
        self.variables = variables or {}
        self.label = label
        self.plan = make_plan_node(plan_spec)
        self.released = False

    def render_command(self, command: str) -> str:
        variables = dict(self.variables)
        variables.setdefault("character", self.character_name)
        return normalize_command_line(format_template(command, variables))

    def tick(self) -> bool:
        return self.released or self.plan.tick(self)

    def cancel(self) -> None:
        self.released = True

    def status_line(self) -> str:
        agent = get_agent(self.character_name)
        queued = len(agent.commands) if agent is not None else 0
        return f"{self.character_name}: {self.label}, {current_command_name(agent)}, queued={queued}, {self.plan.status()}"


def extract_command_lines(spec) -> list[str]:
    if isinstance(spec, str):
        return [spec]
    if isinstance(spec, list):
        lines = []
        for item in spec:
            lines.extend(extract_command_lines(item))
        return lines
    if not isinstance(spec, dict):
        return []

    lines = []
    if "command" in spec:
        lines.append(str(spec["command"]))
    if isinstance(spec.get("commands"), list):
        lines.extend(str(command) for command in spec["commands"])
    if isinstance(spec.get("steps"), list):
        lines.extend(extract_command_lines(spec["steps"]))
    if isinstance(spec.get("children"), list):
        lines.extend(extract_command_lines(spec["children"]))
    if "child" in spec:
        lines.extend(extract_command_lines(spec["child"]))
    return lines


def command_plan(commands: list[str], count=1):
    plan = {"type": "sequence", "steps": [{"type": "command", "command": command} for command in commands]}
    parsed_count = parse_repeat_count(count)
    if parsed_count == 1:
        return plan
    return {"type": "repeat", "count": parsed_count, "child": plan}


class ScenarioRunner:
    def __init__(self):
        self.controllers: dict[str, CharacterController] = {}
        self.label = "idle"

    def start(self, label: str, scenario, variables: dict | None = None) -> None:
        variables = variables or {}
        character_plans = self._compile_scenario(scenario, variables)
        if not character_plans:
            print(f"[people_control_test] Scenario '{label}' did not produce any character plans.")
            return

        self.label = label
        for character_name, plan_spec in character_plans.items():
            self.replace_controller(character_name, plan_spec, variables, label)
        refresh_status(force_log=True)

    def stop_all(self) -> None:
        for controller in self.controllers.values():
            controller.cancel()
        self.controllers.clear()
        self.label = "idle"
        if STATUS_LABEL is not None:
            STATUS_LABEL.text = "Scenario: idle"

    def replace_controller(self, character_name: str, plan_spec, variables: dict, label: str) -> None:
        old_controller = self.controllers.pop(character_name, None)
        if old_controller is not None:
            old_controller.cancel()
        self.controllers[character_name] = CharacterController(character_name, plan_spec, variables, label)

    def tick(self) -> None:
        for character_name, controller in list(self.controllers.items()):
            if controller.tick():
                self.controllers.pop(character_name, None)
        if not self.controllers:
            self.label = "idle"

    def status_lines(self) -> list[str]:
        return [controller.status_line() for _, controller in sorted(self.controllers.items())]

    def _compile_scenario(self, scenario, variables: dict) -> dict[str, object]:
        if isinstance(scenario, dict) and isinstance(scenario.get("characters"), dict):
            plans = {}
            for raw_name, plan_spec in scenario["characters"].items():
                character_name = format_template(raw_name, variables)
                if character_name:
                    plans[character_name] = plan_spec
            return plans

        if isinstance(scenario, dict) and str(scenario.get("type", "")).lower() == "parallel":
            plans = {}
            for child in scenario.get("children", []):
                for character_name, plan_spec in self._compile_scenario(child, variables).items():
                    if character_name in plans:
                        print(f"[people_control_test] Duplicate controller for {character_name}; using the later plan.")
                    plans[character_name] = plan_spec
            return plans

        if isinstance(scenario, dict) and isinstance(scenario.get("commands"), list):
            count = scenario.get("count", scenario.get("repeat", 1))
            return self._compile_command_list(scenario["commands"], count, variables)

        if isinstance(scenario, list):
            return self._compile_command_list(scenario, 1, variables)

        commands = extract_command_lines(scenario)
        command_names = {
            command_agent_name(normalize_command_line(format_template(command, variables))) for command in commands
        }
        command_names.discard("")
        if len(command_names) == 1:
            return {next(iter(command_names)): scenario}
        if len(command_names) > 1:
            print("[people_control_test] Multi-character sequence is ambiguous; use 'characters' or 'parallel'.")
        return {}

    def _compile_command_list(self, commands: list[str], count, variables: dict) -> dict[str, object]:
        grouped: dict[str, list[str]] = {}
        for command in commands:
            rendered = normalize_command_line(format_template(command, variables))
            character_name = command_agent_name(rendered)
            if character_name:
                grouped.setdefault(character_name, []).append(rendered)
        return {character_name: command_plan(character_commands, count) for character_name, character_commands in grouped.items()}


SCENARIO_RUNNER = ScenarioRunner()


def current_command_name(agent) -> str:
    if agent is None:
        return "not registered"
    if agent.current_command is not None:
        return agent.current_command.get_command_name()
    return "queued" if agent.commands else "done"


def refresh_status(force_log: bool = False) -> None:
    global LAST_STATUS_LOG_TIME

    SCENARIO_RUNNER.tick()
    lines = SCENARIO_RUNNER.status_lines()
    if not lines:
        if STATUS_LABEL is not None:
            STATUS_LABEL.text = "Scenario: idle"
        return

    if STATUS_LABEL is not None:
        STATUS_LABEL.text = f"{SCENARIO_RUNNER.label}: {len(lines)} active"

    now = time.monotonic()
    if force_log or now - LAST_STATUS_LOG_TIME > 1.0:
        print("[people_control_test] " + " | ".join(lines))
        LAST_STATUS_LOG_TIME = now


def ui_variables(selected_character: dict, x_model, y_model, r_model) -> dict:
    return {
        "character": selected_character.get("character", ""),
        "x": x_model.model.get_value_as_float(),
        "y": y_model.model.get_value_as_float(),
        "r": r_model.model.get_value_as_float(),
    }


def start_named_scenario(name: str, selected_character: dict, x_model, y_model, r_model) -> None:
    cfg = load_yaml_config()
    scenarios = cfg.get("scenarios", {})
    if not isinstance(scenarios, dict):
        print(f"[people_control_test] YAML 'scenarios' must be a mapping in {COMMANDS_YAML_FILE}.")
        return

    scenario = scenarios.get(name)
    if not isinstance(scenario, dict):
        print(f"[people_control_test] Scenario '{name}' is not configured in {COMMANDS_YAML_FILE}.")
        return

    label = str(scenario.get("label", name))
    SCENARIO_RUNNER.start(label, scenario, ui_variables(selected_character, x_model, y_model, r_model))


def run_button(button: dict, selected_character: dict, x_model, y_model, r_model) -> None:
    action = str(button.get("action", "")).lower()
    scenario_name = button.get("scenario")

    if action == "reset":
        SCENARIO_RUNNER.stop_all()
    elif action == "look_at_all":
        SCENARIO_RUNNER.stop_all()
        duration = float(button.get("duration", LOOK_AT_DEFAULT_DURATION))
        radius = float(button.get("radius", LOOK_AT_DEFAULT_RADIUS))
        look_at_all_characters(duration=duration, radius=radius)
        return
    elif action == "stop":
        SCENARIO_RUNNER.stop_all()
        return

    if scenario_name:
        start_named_scenario(str(scenario_name), selected_character, x_model, y_model, r_model)


def yaml_buttons() -> list[dict]:
    cfg = load_yaml_config()
    buttons = cfg.get("buttons", [])
    if not isinstance(buttons, list):
        print(f"[people_control_test] YAML 'buttons' must be a list in {COMMANDS_YAML_FILE}.")
        buttons = []

    normalized = [button if isinstance(button, dict) else {"label": str(button)} for button in buttons[:9]]
    while len(normalized) < 9:
        normalized.append({"label": "-"})
    return normalized


def build_ui() -> ui.Window:
    global STATUS_LABEL

    people = sorted(load_people())
    default_index = people.index("Male_patient_01") if "Male_patient_01" in people else 0
    selected_character = {"character": people[default_index] if people else ""}
    buttons = yaml_buttons()

    window = ui.Window("People Control Test", width=380, height=290)
    with window.frame:
        with ui.VStack(spacing=10, style={"margin": 8}):
            with ui.HStack(spacing=8):
                ui.Label("Character", width=80)
                selected_model = ui.ComboBox(default_index, *people).model

                def on_character_changed(model, _item):
                    index = combo_box_index(model)
                    if 0 <= index < len(people):
                        selected_character["character"] = people[index]
                        print(f"[people_control_test] UI selected character: {selected_character['character']}")
                    else:
                        selected_character["character"] = ""
                        print(f"[people_control_test] UI selected character index is invalid: {index}")

                selected_model.add_item_changed_fn(on_character_changed)

            with ui.HStack(spacing=8):
                ui.Label("X", width=16)
                x = ui.FloatField()
                x.model.set_value(0.0)
                ui.Label("Y", width=16)
                y = ui.FloatField()
                y.model.set_value(0.0)
                ui.Label("Yaw", width=28)
                r = ui.FloatField()
                r.model.set_value(0.0)

            for row in range(3):
                with ui.HStack(spacing=6):
                    for button in buttons[row * 3 : row * 3 + 3]:
                        enabled = bool(button.get("scenario") or button.get("action"))
                        ui.Button(
                            str(button.get("label", "-")),
                            clicked_fn=lambda b=button: run_button(b, selected_character, x, y, r),
                            enabled=enabled,
                            height=34,
                        )
            STATUS_LABEL = ui.Label("Scenario: idle")
    return window



def remove_stale_people_scripts() -> None:
    """Remove behavior scripts authored on non-SkelRoot character descendants."""
    import OmniScriptingSchema
    import omni.kit.commands

    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(CHARACTER_ROOT)
    if not root.IsValid():
        print(f"[spot_bridge_with_people] Character root not found: {CHARACTER_ROOT}")
        return

    valid_paths = get_people_skelroot_paths()
    stale_paths = []
    for prim in Usd.PrimRange(root):
        prim_path = str(prim.GetPath())
        if prim == root or prim_path in valid_paths or prim.IsA(UsdSkel.Root):
            continue
        if prim.HasAPI(OmniScriptingSchema.OmniScriptingAPI) or prim.HasAttribute("omni:scripting:scripts"):
            stale_paths.append(prim_path)

    print(f"[spot_bridge_with_people] Stale non-SkelRoot people scripts: {len(stale_paths)}")
    if stale_paths:
        omni.kit.commands.execute("RemoveScriptingAPICommand", paths=[Sdf.Path(path) for path in stale_paths])
        for path in stale_paths:
            prim = stage.GetPrimAtPath(path)
            attr = prim.GetAttribute("omni:scripting:scripts")
            if attr:
                attr.Set([])

    script_manager = ScriptManager.get_instance()
    if script_manager is not None:
        for path in stale_paths:
            if path not in script_manager._prim_to_scripts:
                continue
            destroy_script_manager_entry(script_manager, path)
        remove_stale_script_manager_people_instances(valid_paths)


def force_load_skelroot_people_scripts() -> None:
    """Instantiate behavior scripts on the valid character SkelRoots."""
    script_manager = ScriptManager.get_instance()
    if script_manager is None or script_manager._stage is None:
        print("[spot_bridge_with_people] ScriptManager is not ready for SkelRoot script load.")
        return

    script_manager._allow_scripts_to_execute = True
    loaded = 0
    for skelroot in get_people_skelroots():
        prim = script_manager._stage.GetPrimAtPath(str(skelroot.GetPath()))
        if prim and prim.IsValid():
            script_manager._apply_scripts(prim)
            loaded += 1
    print(f"[spot_bridge_with_people] Requested behavior script load for SkelRoots: {loaded}")


def log_people_registration() -> None:
    skelroots = [str(prim.GetPath()) for prim in get_people_skelroots()]
    print(f"[spot_bridge_with_people] Character SkelRoots: {len(skelroots)}")
    if skelroots:
        print(f"[spot_bridge_with_people] First Character SkelRoot: {skelroots[0]}")

    agent_manager = get_legacy_agent_manager()
    if agent_manager is None:
        print("[spot_bridge_with_people] Legacy registered people agents: unavailable in this Isaac version")
        return

    registered = list(agent_manager.get_all_agent_names())
    print(f"[spot_bridge_with_people] Registered people agents: {len(registered)}")
    if registered:
        print(f"[spot_bridge_with_people] First registered people agent: {registered[0]}")


def setup_people_controls(open_stage_file: bool = False) -> ui.Window:
    configure_people()
    if open_stage_file:
        open_stage()
    strip_nested_rigid_bodies()
    configure_people()
    remove_stale_people_scripts()
    bake_navmesh()
    setup_saved_characters()
    ensure_behavior_agents()
    remove_stale_people_scripts()
    force_load_skelroot_people_scripts()
    init_behavior_scripts()
    log_people_registration()
    return build_ui()

# Keep the entrypoint lifecycle aligned with people_control_sim.py:
# configure people, open the populated USD, register people behavior scripts,
# and only then add the Spot ROS bridge on top of that running stage.
enable_people_extensions()
configure_people()
open_stage()
configure_people()
control_window = setup_people_controls(open_stage_file=False)

# Enable ROS2 bridge before importing rclpy or creating ROS2 OmniGraph nodes.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# rclpy must be imported AFTER the ROS2 bridge extension is enabled
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import JointState

first_step = True
reset_needed = False

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
print(f"[spot_bridge_with_people] PhysX threads: {carb.settings.get_settings().get('/physics/numThreads')}")
my_world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)

# Reuse the stage that people_control_sim.py opened. Do not add a second
# hospital reference here; that can leave the people extension looking at stale
# character prims from a different composition path.
print(f"[spot_bridge_with_people] Using people-loaded stage: {USD_PATH}")

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
            print(f"[spot_bridge_with_people] Removing nested RigidBodyAPI from {_path}")
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
        print(f"[spot_bridge_with_people] Created body camera {_cam_path}")
else:
    print("[spot_bridge_with_people] Fisheye cameras disabled (ENABLE_FISHEYE_CAMERAS=False)")

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
        carb.log_warn("[spot_bridge_with_people] No Camera prims found under rsd455 — RealSense graphs skipped")
        return None, None, FRAME_REALSENSE, FRAME_REALSENSE

    color_cams = [c for c in camera_prims if "color" in c.lower() or "rgb" in c.lower()]
    depth_cams = [c for c in camera_prims if "depth" in c.lower() or "ir" in c.lower()]
    color_cam = color_cams[0] if color_cams else camera_prims[0]
    depth_cam = depth_cams[0] if depth_cams else camera_prims[-1]
    color_frame = _frame_id_from_prim_path(color_cam, "realsense_color")
    depth_frame = _frame_id_from_prim_path(depth_cam, "realsense_depth")
    print(f"[spot_bridge_with_people] RealSense cameras: color={color_cam}, depth={depth_cam}")
    print(f"[spot_bridge_with_people] RealSense frames: color={color_frame}, depth={depth_frame}")
    return color_cam, depth_cam, color_frame, depth_frame


# Auto-discover the actual Camera prim inside the front camera xform.
# The outer xform and the inner Camera often share the same name.
_fc_cams = _find_camera_prims_under(FRONT_CAMERA_XFORM)
FRONT_CAMERA_PRIM = _fc_cams[0] if _fc_cams else FRONT_CAMERA_XFORM
print(f"[spot_bridge_with_people] Front camera prim: {FRONT_CAMERA_PRIM}")


def configure_front_camera_as_fisheye() -> None:
    prim = _stage.GetPrimAtPath(FRONT_CAMERA_PRIM)
    camera = UsdGeom.Camera(prim)
    if not camera:
        carb.log_warn(f"[spot_bridge_with_people] Cannot configure front camera as fisheye; not a USD Camera: {FRONT_CAMERA_PRIM}")
        return

    h_ap = 2.0 * FISHEYE_FOCAL_LEN_MM * math.tan(FRONT_CAMERA_FISHEYE_HFOV_RAD * 0.5)
    v_ap = h_ap * FRONT_CAMERA_RES[1] / FRONT_CAMERA_RES[0]
    camera.GetFocalLengthAttr().Set(FISHEYE_FOCAL_LEN_MM)
    camera.GetHorizontalApertureAttr().Set(h_ap)
    camera.GetVerticalApertureAttr().Set(v_ap)
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(*FISHEYE_CLIP_RANGE))
    print(f"[spot_bridge_with_people] Front camera configured as fisheye-style camera: {FRONT_CAMERA_PRIM}")


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
        print(f"[spot_bridge_with_people] RealSense camera prims found under {_rs_root}: {_rs_cams}")
    else:
        # Dump all descendant prim types so we can identify the right path/type
        for _candidate in [REALSENSE_PRIM, "/World/spot/body/rsd455"]:
            for _p in _stage.Traverse():
                _pp = str(_p.GetPath())
                if _pp.startswith(_candidate + "/"):
                    carb.log_warn(f"[spot_bridge_with_people] rsd455 child: {_pp}  type={_p.GetTypeName()}")
    REALSENSE_COLOR_CAM, REALSENSE_DEPTH_CAM, FRAME_REALSENSE_COLOR, FRAME_REALSENSE_DEPTH = _select_realsense_camera_prims(_rs_cams)
else:
    print("[spot_bridge_with_people] RealSense disabled (ENABLE_REALSENSE=False)")
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
    carb.log_warn("[spot_bridge_with_people] No leg prims found under /World/spot — check prim names in USD")
else:
    print(f"[spot_bridge_with_people] Found {len(LEG_PRIMS)} leg prims for TF: {LEG_PRIMS}")

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
        print("[spot_bridge_with_people] Leg TF disabled (ENABLE_LEG_TF=False) — relying on external robot_state_publisher")
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
            print(f"[spot_bridge_with_people] Publishing RealSense camera TFs: {_realsense_camera_prims}")
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

base_command = np.zeros(3)
omni.timeline.get_timeline_interface().play()

my_world.reset()
my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)

# Initialize rclpy and subscribe to /cmd_vel to drive the robot
rclpy.init()
ros_node = rclpy.create_node("spot_cmd_vel_listener")

# robot command [vx, vy, yaw_rate]


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
    refresh_status()
    # Process incoming /cmd_vel messages every step (non-blocking)
    rclpy.spin_once(ros_node, timeout_sec=0.0)
    if my_world.is_stopped():
        reset_needed = True

# shutdown
ros_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
