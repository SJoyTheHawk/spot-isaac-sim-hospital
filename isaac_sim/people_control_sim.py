"""Standalone people-control test for Isaac Sim."""

import os

from isaacsim import SimulationApp


def _env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _env_optional_float(name: str) -> float | None:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


PEOPLE_TEST_HEADLESS = _env_flag("PEOPLE_TEST_HEADLESS", _env_flag("SPOT_ISAAC_HEADLESS", False))

simulation_app = SimulationApp({"headless": PEOPLE_TEST_HEADLESS})

import math
import random
import time
import traceback

import carb
import omni.timeline
import omni.ui as ui
import omni.usd
import yaml
from isaacsim.core.utils.extensions import enable_extension
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel


REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
USD_PATH = os.path.realpath(
    os.environ.get("PEOPLE_TEST_USD", os.path.join(REPO_DIR, "assets", "isaac_hospital_scene_spot_w_characters_6.usd"))
)
COMMANDS_YAML_FILE = os.path.realpath(
    os.environ.get("PEOPLE_INITIAL_COMMANDS", os.path.join(REPO_DIR, "assets", "people_initial_commands.yaml"))
)
CHARACTER_ROOT = "/World/Characters"
MOTION_LIBRARY_PRIM_PATH = f"{CHARACTER_ROOT}/HumanMotionLibrary"
LOOK_AT_DEFAULT_DURATION = 8.0
LOOK_AT_DEFAULT_RADIUS = 4.0
LOOK_AT_DEFAULT_HEIGHT = 1.45
LOOK_AROUND_DEFAULT_INTERVAL = 3.0
TALK_DEFAULT_DURATION = 10.0
TALK_DEFAULT_INTERVAL = 1.8
TALK_DEFAULT_GESTURES = ["open", "point", "relaxed"]
SIT_SNAP_TO_SEAT = _env_flag("PEOPLE_SIT_SNAP_TO_SEAT", True)
SIT_HIPS_OFFSET_X = _env_optional_float("PEOPLE_SIT_HIPS_OFFSET_X")
SIT_HIPS_OFFSET_Y = _env_optional_float("PEOPLE_SIT_HIPS_OFFSET_Y")
SIT_HIPS_OFFSET_Z = _env_optional_float("PEOPLE_SIT_HIPS_OFFSET_Z")
SIT_HIPS_ROTATE_X = _env_optional_float("PEOPLE_SIT_HIPS_ROTATE_X")
SIT_HIPS_ROTATE_Y = _env_optional_float("PEOPLE_SIT_HIPS_ROTATE_Y")
SIT_HIPS_ROTATE_Z = _env_optional_float("PEOPLE_SIT_HIPS_ROTATE_Z")
PEOPLE_TEST_AUTO_LOOK_AT = _env_flag("PEOPLE_TEST_AUTO_LOOK_AT")
PEOPLE_TEST_AUTO_GOTO = _env_flag("PEOPLE_TEST_AUTO_GOTO")
PEOPLE_TEST_AUTO_PATROL = _env_flag("PEOPLE_TEST_AUTO_PATROL")
PEOPLE_TEST_AUTO_SIT = _env_flag("PEOPLE_TEST_AUTO_SIT")
PEOPLE_TEST_EXIT_AFTER_AUTO = _env_flag("PEOPLE_TEST_EXIT_AFTER_AUTO")
STATUS_LABEL = None
LAST_STATUS_LOG_TIME = 0.0
SCENARIO_RUNNER = None
LAST_CONFIG_VALIDATION_MTIME = None

NATIVE_ACTIONS = {
    "idle",
    "move_to",
    "move_along",
    "follow",
    "dodge",
    "fall",
    "sit",
    "ride",
    "pickup_object",
    "place_object",
    "release_object",
    "custom_action",
    "look_at",
    "reach_hand",
    "pose_hand",
    "reset",
    "teleport",
}
SCHEDULER_ACTIONS = {"wait", "repeat"}
COMPOSITE_ACTIONS = {"patrol", "look_around", "talk", "talk_with"}
KNOWN_ACTIONS = NATIVE_ACTIONS | SCHEDULER_ACTIONS | COMPOSITE_ACTIONS
BUTTON_ACTIONS = {"go_to_selected", "look_at_all", "reset", "stop"}


def enable_people_extensions() -> None:
    print(f"[people_control_test] Isaac launch: headless={PEOPLE_TEST_HEADLESS}", flush=True)
    for ext in [
        "omni.behavior.scripting.core",
        "omni.anim.behavior.core",
        "omni.anim.timeline",
        "omni.anim.graph.core",
        "omni.anim.retarget.core",
        "omni.anim.navigation.core",
        "isaacsim.replicator.agent.core",
        "omni.kit.mesh.raycast",
    ]:
        print(f"[people_control_test] Enabling extension: {ext}", flush=True)
        enable_extension(ext)
        print(f"[people_control_test] Extension enabled: {ext}", flush=True)
    simulation_app.update()
    print("[people_control_test] People extensions ready.", flush=True)


def open_stage() -> None:
    print(f"[people_control_test] Loading: {USD_PATH}", flush=True)
    if not omni.usd.get_context().open_stage(USD_PATH):
        raise RuntimeError(USD_PATH)
    for _ in range(8):
        simulation_app.update()
    print("[people_control_test] Stage loaded.", flush=True)


def strip_nested_rigid_bodies() -> None:
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith("/World/spot/body/") and path != "/World/spot/body":
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                prim.GetAttribute("physics:rigidBodyEnabled").Set(False)


def configure_people() -> None:
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


def character_display_name(skelroot_prim) -> str:
    path_parts = [part for part in str(skelroot_prim.GetPath()).split("/") if part]
    root_parts = [part for part in CHARACTER_ROOT.split("/") if part]
    if path_parts[: len(root_parts)] == root_parts:
        relative_parts = path_parts[len(root_parts) :]
        if len(relative_parts) >= 2 and relative_parts[0].endswith("_Group"):
            return relative_parts[1]
        if relative_parts:
            return relative_parts[0]
    return skelroot_prim.GetName()


def get_character_skelroot(character_name: str):
    for skelroot in get_people_skelroots():
        if character_display_name(skelroot) == character_name:
            return skelroot
    return None


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
    print(f"[people_control_test] Creating Isaac 6 behavior motion library at {MOTION_LIBRARY_PRIM_PATH}")
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
        print(f"[people_control_test] Failed to create behavior motion library from {asset}: {exc}")
        return None

    for _ in range(5):
        simulation_app.update()

    prim = stage.GetPrimAtPath(motion_library_path)
    return motion_library_path if prim and prim.IsValid() else None


def ensure_behavior_agents() -> None:
    skelroots = get_people_skelroots()
    if not skelroots:
        print("[people_control_test] No character SkelRoots found for Isaac 6 BehaviorAgentAPI setup.")
        return

    motion_library_path = ensure_behavior_motion_library()
    if motion_library_path is None:
        print("[people_control_test] Behavior motion library unavailable; LookAt may report missing agents.")
        return

    try:
        import BehaviorSchema

        missing = [prim for prim in skelroots if not prim.HasAPI(BehaviorSchema.BehaviorAgentAPI)]
    except Exception:
        missing = skelroots

    if not missing:
        print(f"[people_control_test] Isaac 6 BehaviorAgentAPI already present on {len(skelroots)} characters.")
        return

    try:
        omni.kit.commands.execute(
            "ApplyBehaviorAgentAPICommand",
            skelroot_prim_paths=[prim.GetPath() for prim in missing],
            motion_library_prim_path=motion_library_path,
            motion_library_skeleton_rig="Human",
        )
    except Exception as exc:
        print(f"[people_control_test] Failed to apply Isaac 6 BehaviorAgentAPI: {exc}")
        return

    for _ in range(10):
        simulation_app.update()

    print(
        "[people_control_test] Applied Isaac 6 BehaviorAgentAPI: "
        f"{len(missing)}/{len(skelroots)} characters"
    )


def load_yaml_config() -> dict:
    with open(COMMANDS_YAML_FILE, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    validate_yaml_config_once(cfg)
    return cfg


def validate_yaml_config_once(cfg: dict) -> None:
    global LAST_CONFIG_VALIDATION_MTIME

    try:
        mtime = os.path.getmtime(COMMANDS_YAML_FILE)
    except OSError:
        mtime = None
    if LAST_CONFIG_VALIDATION_MTIME == mtime:
        return
    LAST_CONFIG_VALIDATION_MTIME = mtime

    unknown_actions = sorted({name for name in iter_yaml_actions(cfg) if name and name not in KNOWN_ACTIONS | BUTTON_ACTIONS})
    if unknown_actions:
        print(f"[people_control_test] Unknown YAML actions: {unknown_actions}")
    else:
        print(f"[people_control_test] YAML action registry validation passed: {COMMANDS_YAML_FILE}")


def iter_yaml_actions(value):
    if isinstance(value, list):
        for item in value:
            yield from iter_yaml_actions(item)
        return
    if not isinstance(value, dict):
        return

    name = action_name(value)
    if name:
        yield name
    for item in value.values():
        yield from iter_yaml_actions(item)


def normalize_command_line(line: str) -> str:
    parts = line.split()
    if len(parts) < 2:
        return line.strip()
    command = parts[1].upper()
    if command == "IDLE":
        parts[1] = "Idle"
    elif command == "SIT":
        parts[1] = "Sit"
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


def render_templates(value, variables: dict | None = None):
    if isinstance(value, str):
        return format_template(value, variables)
    if isinstance(value, list):
        return [render_templates(item, variables) for item in value]
    if isinstance(value, tuple):
        return tuple(render_templates(item, variables) for item in value)
    if isinstance(value, dict):
        rendered = {}
        for key, item in value.items():
            rendered_key = format_template(key, variables) if isinstance(key, str) else key
            rendered[rendered_key] = render_templates(item, variables)
        return rendered
    return value


def as_float(value, default: float = 0.0) -> float:
    if value is None or value == "":
        return default
    return float(value)


def as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def is_blank(value) -> bool:
    return value is None or (isinstance(value, str) and value.strip() == "")


def optional_duration(spec: dict, default: float | None = None) -> float | None:
    if not isinstance(spec, dict) or is_blank(spec.get("duration")):
        return default
    return float(spec["duration"])


def vector3(value, label: str = "vector") -> carb.Float3:
    if isinstance(value, dict):
        if "position" in value:
            return vector3(value["position"], label)
        return carb.Float3(float(value.get("x", 0.0)), float(value.get("y", 0.0)), float(value.get("z", 0.0)))
    try:
        x, y, z = vec3_components(value)
        return carb.Float3(x, y, z)
    except Exception as exc:
        raise ValueError(f"Invalid {label}: {value}") from exc


def facing_direction(value):
    if is_blank(value):
        return None
    if isinstance(value, str):
        text = value.strip()
        if text.startswith("/"):
            return text
        value = float(text)
    if isinstance(value, (int, float)):
        yaw_radians = math.radians(float(value) - 90.0)
        return carb.Float3(math.cos(yaw_radians), math.sin(yaw_radians), 0.0)
    return vector3(value, "facing")


def vec3d_or_none(value) -> Gf.Vec3d | None:
    if value is None:
        return None
    if isinstance(value, dict):
        return Gf.Vec3d(float(value.get("x", 0.0)), float(value.get("y", 0.0)), float(value.get("z", 0.0)))
    x, y, z = vec3_components(value)
    return Gf.Vec3d(x, y, z)


def character_target_path(character_name: str) -> str | None:
    if not character_name:
        return None
    if str(character_name).startswith("/"):
        return str(character_name)
    skelroot = get_character_skelroot(str(character_name))
    return str(skelroot.GetPath()) if skelroot is not None else None


def resolve_target(value):
    if isinstance(value, dict):
        if "prim" in value:
            return str(value["prim"])
        if "path" in value:
            return str(value["path"])
        if "character" in value:
            return character_target_path(str(value["character"])) or str(value["character"])
        if "target_character" in value:
            return character_target_path(str(value["target_character"])) or str(value["target_character"])
        if "position" in value:
            return vector3(value["position"], "target.position")
    if isinstance(value, (list, tuple)):
        return vector3(value, "target")
    if isinstance(value, str):
        text = value.strip()
        if text.startswith("/"):
            return text
        return character_target_path(text) or text
    return value


def action_name(spec: dict) -> str:
    return str(spec.get("action", spec.get("type", ""))).strip().lower()


def behavior_hand_usage(value):
    import omni.anim.behavior.core as bh_core

    hand = str(value or "right").strip().lower().replace("-", "_")
    mapping = {
        "right": "RIGHT_HAND",
        "right_hand": "RIGHT_HAND",
        "left": "LEFT_HAND",
        "left_hand": "LEFT_HAND",
        "both": "BOTH_HANDS",
        "both_hands": "BOTH_HANDS",
        "none": "NONE",
    }
    enum_name = mapping.get(hand, "RIGHT_HAND")
    return getattr(bh_core.BehaviorHandUsage, enum_name)


def behavior_hand_pose(value):
    import omni.anim.behavior.core as bh_core

    preset = str(value or "relaxed").strip().lower().replace("-", "_")
    mapping = {
        "open": "OPEN",
        "fist": "FIST",
        "point": "POINT",
        "relaxed": "RELAXED",
    }
    enum_name = mapping.get(preset, "RELAXED")
    return getattr(bh_core.BehaviorHandPosePreset, enum_name)


def behavior_root_animation(value):
    if value in {None, ""}:
        return None

    import omni.anim.behavior.core as bh_core

    enum_type = getattr(bh_core, "BehaviorRootAnimation", None)
    if enum_type is None:
        return None

    normalized = str(value).strip().lower().replace("-", "_")
    candidates_by_value = {
        "ignore": ["IGNORE_ROOT_ANIMATION", "IGNORE_ROOT", "IGNORE"],
        "keep": ["KEEP_ROOT_ANIMATION", "KEEP_ROOT", "KEEP"],
        "use": ["USE_ROOT_ANIMATION", "USE_ROOT", "USE"],
    }
    for candidate in candidates_by_value.get(normalized, [str(value)]):
        if hasattr(enum_type, candidate):
            return getattr(enum_type, candidate)
    return None


def task_is_running(agent, task_id: int | None) -> bool:
    if agent is None or task_id is None:
        return False
    try:
        return agent.is_task_running(task_id)
    except Exception:
        return False


def cancel_task(agent, task_id: int | None) -> None:
    if not task_is_running(agent, task_id):
        return
    try:
        agent.cancel_task(task_id)
    except Exception as exc:
        print(f"[people_control_test] Unable to cancel behavior task {task_id}: {exc}")


def cancel_active_action_task(agent) -> None:
    try:
        active_task = agent.get_action_task_id()
    except Exception:
        return
    if active_task and active_task != behavior_task_id_invalid():
        try:
            agent.cancel_task(active_task)
        except Exception as exc:
            print(f"[people_control_test] Unable to cancel active behavior task {active_task}: {exc}")


def current_facing_direction(agent):
    try:
        return agent.get_facing_direction()
    except Exception:
        return None


def default_facing_direction() -> carb.Float3:
    return carb.Float3(1.0, 0.0, 0.0)


def idle_facing_direction(agent, requested_facing=None):
    facing = facing_direction(requested_facing)
    if facing is not None:
        return facing
    facing = current_facing_direction(agent)
    if facing is not None:
        return facing
    return default_facing_direction()


def combo_box_index(model) -> int:
    value_model = model.get_item_value_model()
    if hasattr(value_model, "get_value_as_int"):
        return value_model.get_value_as_int()
    return value_model.as_int if hasattr(value_model, "as_int") else -1


def parse_goto_command(line: str) -> dict | None:
    parts = normalize_command_line(line).split()
    if len(parts) < 5 or len(parts) > 6 or parts[1] != "GoTo":
        return None

    try:
        return {
            "character": parts[0],
            "x": float(parts[2]),
            "y": float(parts[3]),
            "z": float(parts[4]),
            "yaw": 0.0 if len(parts) < 6 or parts[5] == "_" else float(parts[5]),
        }
    except ValueError as exc:
        print(f"[people_control_test] Invalid GoTo command '{line}': {exc}")
        return None


def parse_sit_command(line: str) -> dict | None:
    parts = normalize_command_line(line).split()
    if len(parts) < 3 or len(parts) > 4 or parts[1] != "Sit":
        return None

    try:
        duration = float(parts[3]) if len(parts) > 3 else -1.0
    except ValueError as exc:
        print(f"[people_control_test] Invalid Sit command '{line}': {exc}")
        return None

    return {
        "character": parts[0],
        "target": parts[2],
        "duration": duration,
    }


def parse_idle_command(line: str) -> dict | None:
    parts = normalize_command_line(line).split()
    if len(parts) < 2 or len(parts) > 3 or parts[1] != "Idle":
        return None

    try:
        duration = float(parts[2]) if len(parts) > 2 else -1.0
    except ValueError as exc:
        print(f"[people_control_test] Invalid Idle command '{line}': {exc}")
        return None

    return {
        "character": parts[0],
        "duration": duration,
    }


def vec3_components(value) -> tuple[float, float, float]:
    try:
        return float(value[0]), float(value[1]), float(value[2])
    except (TypeError, IndexError, KeyError):
        return float(value.x), float(value.y), float(value.z)


def navmesh_target_for_xy(agent, x: float, y: float, z: float = 0.0) -> carb.Float3:
    target = carb.Float3(float(x), float(y), float(z))
    try:
        import omni.anim.navigation.core as nav

        navmesh = nav.acquire_interface().get_navmesh()
        if navmesh is None:
            return target

        radius = float(agent.get_radius())
        height = float(agent.get_height())
        if radius > 0.0 and height > 0.0:
            agent_desc = nav.NavAgentDesc(radius=radius, height=height, collision_gap=0.0)
            result = navmesh.query_closest_point(target, agent=agent_desc)
        else:
            result = navmesh.query_closest_point(target)

        closest = result[0] if result else None
        if closest is None:
            return target

        cx, cy, cz = vec3_components(closest)
        return carb.Float3(cx, cy, cz)
    except Exception as exc:
        print(f"[people_control_test] Navmesh target snap failed; using raw target ({x:g}, {y:g}, {z:g}): {exc}")
        return target


def get_behavior_agent(character_name: str):
    import omni.anim.behavior.core as bh_core

    character_name = str(character_name or "").strip()
    if not character_name:
        return None, None

    skelroot = get_character_skelroot(character_name)
    if skelroot is None:
        return None, None

    path = str(skelroot.GetPath())
    agent = bh_core.acquire_interface().get_agent(path)
    return agent, path


def behavior_task_id_invalid() -> int:
    import omni.anim.behavior.core as bh_core

    return getattr(bh_core, "BEHAVIOR_TASK_ID_INVALID", -1)


def ensure_behavior_prop_api(prim) -> None:
    try:
        import BehaviorSchema

        if prim and prim.IsValid() and not prim.HasAPI(BehaviorSchema.BehaviorPropAPI):
            BehaviorSchema.BehaviorPropAPI.Apply(prim)
    except Exception as exc:
        print(f"[people_control_test] Unable to apply BehaviorPropAPI to {prim.GetPath()}: {exc}")


def sit_hips_default_transform(stage, target_path: str) -> tuple[Gf.Vec3d, Gf.Vec3d]:
    offset = Gf.Vec3d(0.0, 0.0, 0.0)
    rotation = Gf.Vec3d(0.0, 0.0, 0.0)

    if stage is not None and UsdGeom.GetStageUpAxis(stage) == UsdGeom.Tokens.z:
        rotation = Gf.Vec3d(90.0, 0.0, 0.0)

    if target_path.startswith("/World/Chair/"):
        # These proxy prims are already authored at seat/hips height. Their
        # walk_to_offset child is the floor approach point, not the seated pose.
        offset = Gf.Vec3d(0.0, 0.0, 0.0)

    return offset, rotation


def apply_sit_hips_overrides(offset: Gf.Vec3d, rotation: Gf.Vec3d) -> tuple[Gf.Vec3d, Gf.Vec3d]:
    return (
        Gf.Vec3d(
            offset[0] if SIT_HIPS_OFFSET_X is None else SIT_HIPS_OFFSET_X,
            offset[1] if SIT_HIPS_OFFSET_Y is None else SIT_HIPS_OFFSET_Y,
            offset[2] if SIT_HIPS_OFFSET_Z is None else SIT_HIPS_OFFSET_Z,
        ),
        Gf.Vec3d(
            rotation[0] if SIT_HIPS_ROTATE_X is None else SIT_HIPS_ROTATE_X,
            rotation[1] if SIT_HIPS_ROTATE_Y is None else SIT_HIPS_ROTATE_Y,
            rotation[2] if SIT_HIPS_ROTATE_Z is None else SIT_HIPS_ROTATE_Z,
        ),
    )


def parse_sit_target_spec(target_spec) -> tuple[str, Gf.Vec3d | None, Gf.Vec3d | None, bool]:
    if isinstance(target_spec, dict):
        target_path = str(target_spec.get("prim", target_spec.get("path", ""))).strip()
        hips_offset = vec3d_or_none(target_spec.get("hips_offset"))
        hips_rotation = vec3d_or_none(target_spec.get("hips_rotation"))
        snap_to_seat = as_bool(target_spec.get("snap_to_seat"), SIT_SNAP_TO_SEAT)
        return target_path, hips_offset, hips_rotation, snap_to_seat

    return str(target_spec or "").strip(), None, None, SIT_SNAP_TO_SEAT


def ensure_sit_effector(
    target_path: str,
    hips_offset: Gf.Vec3d | None = None,
    hips_rotation: Gf.Vec3d | None = None,
) -> str:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return target_path

    target_path = str(target_path or "").strip()
    target_prim = stage.GetPrimAtPath(target_path)
    if not target_prim or not target_prim.IsValid():
        print(f"[people_control_test] Sit target prim does not exist: {target_path}")
        return target_path

    ensure_behavior_prop_api(target_prim)

    try:
        import BehaviorSchema
    except Exception as exc:
        print(f"[people_control_test] BehaviorSchema unavailable for sit effector setup: {exc}")
        return target_path

    behavior_path = Sdf.Path(f"{target_path}/Behavior")
    sit_path = behavior_path.AppendChild("Sit")
    hips_path = behavior_path.AppendChild("Sit_Hips")

    if not stage.GetPrimAtPath(behavior_path).IsValid():
        stage.DefinePrim(behavior_path, "Scope")

    sit_prim = stage.GetPrimAtPath(sit_path)
    if not sit_prim.IsValid():
        sit_prim = stage.DefinePrim(sit_path, "BehaviorTaskEffectors")
    if not sit_prim.HasAPI(BehaviorSchema.BehaviorTaskEffectorAPI):
        BehaviorSchema.BehaviorTaskEffectorAPI.Apply(sit_prim)

    sit_prim.CreateAttribute("behavior:task", Sdf.ValueTypeNames.Token).Set("Sit")
    sit_prim.CreateRelationship("behavior:task:effectorHips").SetTargets([hips_path])
    target_prim.CreateRelationship("behavior:task:effectors").AddTarget(sit_path)

    hips_prim = stage.GetPrimAtPath(hips_path)
    if not hips_prim.IsValid():
        hips_prim = stage.DefinePrim(hips_path, "Xform")
    hips_xform = UsdGeom.Xformable(hips_prim)
    default_offset, default_rotation = sit_hips_default_transform(stage, target_path)
    offset = hips_offset if hips_offset is not None else default_offset
    rotation = hips_rotation if hips_rotation is not None else default_rotation
    offset, rotation = apply_sit_hips_overrides(offset, rotation)
    hips_xform.ClearXformOpOrder()
    hips_xform.AddTranslateOp().Set(offset)
    hips_xform.AddRotateXYZOp().Set(rotation)
    hips_xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    print(
        "[people_control_test] Sit_Hips configured: "
        f"target={target_path}, offset=({offset[0]:.3f}, {offset[1]:.3f}, {offset[2]:.3f}), "
        f"rotateXYZ=({rotation[0]:.1f}, {rotation[1]:.1f}, {rotation[2]:.1f})"
    )

    return target_path


def start_behavior_goto(
    character_name: str,
    x: float,
    y: float,
    z: float = 0.0,
    yaw_degrees: float = 0.0,
    log_prefix: str = "GoTo",
) -> tuple[object | None, int | None]:
    character_name = str(character_name or "").strip()
    if not character_name:
        print(f"[people_control_test] {log_prefix} requested without a character.")
        return None, None

    agent, path = get_behavior_agent(character_name)
    if path is None:
        print(f"[people_control_test] {log_prefix} character not found: {character_name}")
        return None, None
    if agent is None:
        print(f"[people_control_test] {log_prefix} behavior agent is not ready for {character_name}: {path}")
        return None, None

    target = navmesh_target_for_xy(agent, x, y, z)
    tx, ty, tz = vec3_components(target)
    invalid_task_id = behavior_task_id_invalid()

    try:
        task_id = agent.move_to(target=target, auto_brake=True)
    except Exception as exc:
        print(f"[people_control_test] {log_prefix} failed for {character_name}: {exc}")
        return None, None

    if task_id == invalid_task_id:
        print(f"[people_control_test] {log_prefix} rejected for {character_name}: target=({tx:.3f}, {ty:.3f}, {tz:.3f})")
        return None, None

    print(
        f"[people_control_test] {log_prefix} started: "
        f"character={character_name}, target=({tx:.3f}, {ty:.3f}, {tz:.3f}), yaw={yaw_degrees:g}, task_id={task_id}"
    )
    return agent, task_id


def start_behavior_sit(
    character_name: str,
    target_spec,
    log_prefix: str = "Sit",
) -> tuple[object | None, int | None]:
    character_name = str(character_name or "").strip()
    target_path, hips_offset, hips_rotation, snap_to_seat = parse_sit_target_spec(target_spec)
    target_path = ensure_sit_effector(target_path, hips_offset=hips_offset, hips_rotation=hips_rotation)
    if not character_name or not target_path:
        print(f"[people_control_test] {log_prefix} requested without a character or target.")
        return None, None

    agent, path = get_behavior_agent(character_name)
    if path is None:
        print(f"[people_control_test] {log_prefix} character not found: {character_name}")
        return None, None
    if agent is None:
        print(f"[people_control_test] {log_prefix} behavior agent is not ready for {character_name}: {path}")
        return None, None

    invalid_task_id = behavior_task_id_invalid()
    try:
        task_id = agent.sit(target_path, snap_to_seat=snap_to_seat)
    except Exception as exc:
        print(f"[people_control_test] {log_prefix} failed for {character_name}: {exc}")
        return None, None

    if task_id == invalid_task_id:
        print(f"[people_control_test] {log_prefix} rejected for {character_name}: target={target_path}")
        return None, None

    print(
        f"[people_control_test] {log_prefix} started: "
        f"character={character_name}, target={target_path}, snap_to_seat={snap_to_seat}, task_id={task_id}"
    )
    return agent, task_id


def start_behavior_idle(
    character_name: str,
    log_prefix: str = "Idle",
) -> tuple[object | None, int | None]:
    character_name = str(character_name or "").strip()
    if not character_name:
        print(f"[people_control_test] {log_prefix} requested without a character.")
        return None, None

    agent, path = get_behavior_agent(character_name)
    if path is None:
        print(f"[people_control_test] {log_prefix} character not found: {character_name}")
        return None, None
    if agent is None:
        print(f"[people_control_test] {log_prefix} behavior agent is not ready for {character_name}: {path}")
        return None, None

    invalid_task_id = behavior_task_id_invalid()
    try:
        cancel_active_action_task(agent)
        task_id = agent.idle(facing=idle_facing_direction(agent))
    except Exception as exc:
        print(f"[people_control_test] {log_prefix} failed for {character_name}: {exc}")
        return None, None

    if task_id == invalid_task_id:
        print(f"[people_control_test] {log_prefix} rejected for {character_name}.")
        return None, None

    print(f"[people_control_test] {log_prefix} started: character={character_name}, task_id={task_id}")
    return agent, task_id


def action_target(spec: dict, agent=None, snap_position: bool = False):
    if "position" in spec:
        target = vector3(spec["position"], "position")
        if snap_position and agent is not None:
            x, y, z = vec3_components(target)
            return navmesh_target_for_xy(agent, x, y, z)
        return target
    if "target_character" in spec:
        return resolve_target({"character": spec["target_character"]})
    if "target" in spec:
        return resolve_target(spec["target"])
    return None


def call_with_optional_duration(func, duration: float | None, **kwargs):
    if duration is None:
        return func(**kwargs)
    return func(duration=duration, **kwargs)


def start_behavior_action(
    character_name: str,
    spec: dict,
    log_prefix: str | None = None,
) -> tuple[object | None, int | None, bool]:
    action = action_name(spec)
    label = log_prefix or action
    if action not in NATIVE_ACTIONS:
        print(f"[people_control_test] Unsupported structured action for {character_name}: {action}")
        return None, None, False

    character_name = str(character_name or spec.get("character", "")).strip()
    if not character_name:
        print(f"[people_control_test] {label} requested without a character.")
        return None, None, False

    agent, path = get_behavior_agent(character_name)
    if path is None:
        print(f"[people_control_test] {label} character not found: {character_name}")
        return None, None, False
    if agent is None:
        print(f"[people_control_test] {label} behavior agent is not ready for {character_name}: {path}")
        return None, None, False

    try:
        if action == "idle":
            cancel_active_action_task(agent)
            task_id = agent.idle(facing=idle_facing_direction(agent, spec.get("facing", spec.get("yaw"))))
        elif action == "move_to":
            target = action_target(spec, agent=agent, snap_position="position" in spec)
            if target is None:
                raise ValueError("move_to needs position or target")
            task_id = agent.move_to(target=target, auto_brake=as_bool(spec.get("auto_brake"), True))
        elif action == "move_along":
            if "waypoints" in spec:
                target = [vector3(waypoint, "waypoint") for waypoint in spec.get("waypoints", [])]
            else:
                target = action_target(spec)
            if not target:
                raise ValueError("move_along needs waypoints or target")
            task_id = agent.move_along(
                target=target,
                start_from_closest_point=as_bool(spec.get("start_from_closest_point"), False),
                auto_brake=as_bool(spec.get("auto_brake"), True),
            )
        elif action == "follow":
            target = action_target(spec)
            if target is None:
                raise ValueError("follow needs target or target_character")
            task_id = agent.follow(target=target, distance=float(spec.get("distance", -1.0)))
        elif action == "dodge":
            task_id = agent.dodge(
                direction=vector3(spec.get("direction", [1.0, 0.0, 0.0]), "direction"),
                motion_scale=float(spec.get("motion_scale", 1.0)),
            )
        elif action == "fall":
            task_id = agent.fall()
        elif action == "sit":
            target_spec = spec.get("target", spec.get("prim", ""))
            return (*start_behavior_sit(character_name, target_spec, log_prefix=label), False)
        elif action == "ride":
            target = action_target(spec)
            if target is None:
                raise ValueError("ride needs target")
            task_id = agent.ride(target=target)
        elif action == "pickup_object":
            target = action_target(spec)
            if target is None:
                raise ValueError("pickup_object needs target")
            task_id = agent.pickup_object(target, snap_to_hand=as_bool(spec.get("snap_to_hand"), False))
        elif action == "place_object":
            target = resolve_target(spec.get("target"))
            placement_target = resolve_target(spec.get("placement_target"))
            if target is None or placement_target is None:
                raise ValueError("place_object needs target and placement_target")
            task_id = agent.place_object(target, placement_target)
        elif action == "release_object":
            target = action_target(spec)
            if target is None:
                raise ValueError("release_object needs target")
            task_id = agent.release_object(target)
        elif action == "custom_action":
            name = str(spec.get("name", "")).strip()
            if not name:
                raise ValueError("custom_action needs name")
            duration = optional_duration(spec)
            root_animation = behavior_root_animation(spec.get("root_animation"))
            kwargs = {}
            if duration is not None:
                kwargs["duration"] = duration
            if root_animation is not None:
                kwargs["root_animation"] = root_animation
            try:
                task_id = agent.custom_action(name, **kwargs)
            except TypeError:
                kwargs.pop("root_animation", None)
                task_id = agent.custom_action(name, **kwargs)
        elif action == "look_at":
            target = action_target(spec)
            if target is None:
                raise ValueError("look_at needs target, target_character, or position")
            task_id = call_with_optional_duration(agent.look_at, optional_duration(spec), target=target)
        elif action == "reach_hand":
            target = action_target(spec)
            if target is None:
                raise ValueError("reach_hand needs target, target_character, or position")
            kwargs = {
                "hand_usage": behavior_hand_usage(spec.get("hand", spec.get("hand_usage", "right"))),
                "target": target,
            }
            if spec.get("palm_direction") is not None:
                kwargs["palm_direction"] = vector3(spec["palm_direction"], "palm_direction")
            if spec.get("finger_direction") is not None:
                kwargs["finger_direction"] = vector3(spec["finger_direction"], "finger_direction")
            if spec.get("motion_scale") is not None:
                kwargs["motion_scale"] = float(spec["motion_scale"])
            task_id = call_with_optional_duration(agent.reach_hand, optional_duration(spec), **kwargs)
        elif action == "pose_hand":
            kwargs = {
                "hand_usage": behavior_hand_usage(spec.get("hand", spec.get("hand_usage", "right"))),
                "preset": behavior_hand_pose(spec.get("preset", "relaxed")),
            }
            task_id = call_with_optional_duration(agent.pose_hand, optional_duration(spec), **kwargs)
        elif action == "reset":
            target = action_target(spec)
            kwargs = {}
            if target is not None:
                kwargs["target"] = target
            facing = facing_direction(spec.get("facing"))
            if facing is not None:
                kwargs["facing"] = facing
            agent.reset(**kwargs)
            print(f"[people_control_test] reset applied: character={character_name}")
            return agent, None, True
        elif action == "teleport":
            target = action_target(spec)
            if target is None:
                raise ValueError("teleport needs position or target")
            kwargs = {"target": target}
            facing = facing_direction(spec.get("facing"))
            if facing is not None:
                kwargs["facing"] = facing
            agent.teleport(**kwargs)
            print(f"[people_control_test] teleport applied: character={character_name}")
            return agent, None, True
        else:
            print(f"[people_control_test] No handler implemented for action: {action}")
            return None, None, False
    except Exception as exc:
        print(f"[people_control_test] {label} failed for {character_name}: {exc}")
        print(traceback.format_exc(limit=4).rstrip())
        return None, None, False

    invalid_task_id = behavior_task_id_invalid()
    if task_id == invalid_task_id:
        print(f"[people_control_test] {label} rejected for {character_name}: action={action}")
        return None, None, False

    print(f"[people_control_test] {label} started: character={character_name}, action={action}, task_id={task_id}")
    return agent, task_id, False


def move_character_to_xy(character_name: str, x: float, y: float, yaw_degrees: float = 0.0) -> None:
    agent, task_id = start_behavior_goto(character_name, x, y, 0.0, yaw_degrees)
    if agent is None or task_id is None:
        return

    if STATUS_LABEL:
        STATUS_LABEL.text = f"GoTo {character_name}: ({float(x):.2f}, {float(y):.2f})"


def random_look_at_target(agent, radius: float, height: float) -> tuple[float, float, float] | None:
    try:
        pos = agent.get_world_translation()
    except Exception:
        return None

    x, y, z = vec3_components(pos)
    angle = random.uniform(0.0, math.tau)
    distance = random.uniform(max(0.5, radius * 0.45), max(0.5, radius))
    z_offset = random.uniform(-0.25, 0.35)
    return (
        x + math.cos(angle) * distance,
        y + math.sin(angle) * distance,
        z + height + z_offset,
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

    def cancel(self) -> None:
        pass


def legacy_command_to_action(command_text: str) -> dict | None:
    command_text = normalize_command_line(command_text)

    goto = parse_goto_command(command_text)
    if goto is not None:
        return {
            "character": goto["character"],
            "action": "move_to",
            "position": [goto["x"], goto["y"], goto["z"]],
            "yaw": goto["yaw"],
        }

    sit = parse_sit_command(command_text)
    if sit is not None:
        spec = {"character": sit["character"], "action": "sit", "target": {"prim": sit["target"]}}
        if sit["duration"] > 0.0:
            spec["duration"] = sit["duration"]
        return spec

    idle = parse_idle_command(command_text)
    if idle is not None:
        spec = {"character": idle["character"], "action": "idle"}
        if idle["duration"] > 0.0:
            spec["duration"] = idle["duration"]
        return spec

    parts = command_text.split()
    if len(parts) >= 2:
        command = parts[1].lower()
        if command in {"lookaround", "look_around"}:
            spec = {"character": parts[0], "action": "look_around"}
            if len(parts) >= 3:
                spec["duration"] = float(parts[2])
            return spec
        if command in {"talk", "talkwith", "talk_with"} and len(parts) >= 3:
            spec = {
                "character": parts[0],
                "action": "talk_with" if command in {"talkwith", "talk_with"} else "talk",
                "target_character": parts[2],
            }
            if len(parts) >= 4:
                spec["duration"] = float(parts[3])
            return spec
        if command == "fall":
            return {"character": parts[0], "action": "fall"}

    return None


class ActionNode(PlanNode):
    def __init__(self, spec: dict):
        self.raw_spec = dict(spec)
        self.spec = {}
        self.started = False
        self.started_at = None
        self.action = ""
        self.behavior_agent = None
        self.behavior_task_id = None
        self.duration = None
        self.cancel_requested = False

    def tick(self, controller) -> bool:
        if not self.started:
            self.spec = controller.render_action(self.raw_spec)
            self.action = action_name(self.spec)
            agent_name = str(self.spec.get("character") or controller.character_name).strip()
            if agent_name and agent_name != controller.character_name:
                print(
                    f"[people_control_test] {controller.character_name} controller received action for {agent_name}; releasing."
                )
                controller.released = True
                return True

            self.spec["character"] = controller.character_name
            self.duration = optional_duration(self.spec)
            print(f"[people_control_test] {controller.character_name} -> action={self.action}")
            self.behavior_agent, self.behavior_task_id, completed = start_behavior_action(
                controller.character_name,
                self.spec,
                log_prefix=self.action,
            )
            self.started = True
            self.started_at = time.monotonic()
            if completed:
                return True
            if self.behavior_agent is None or self.behavior_task_id is None:
                controller.released = True
                return True
            return False

        if self.cancel_requested:
            return not task_is_running(self.behavior_agent, self.behavior_task_id)

        if self.duration is not None and self.duration > 0.0 and self.started_at is not None:
            if time.monotonic() - self.started_at < self.duration:
                return False
            self.cancel()
            self.cancel_requested = True
            return False
        return not task_is_running(self.behavior_agent, self.behavior_task_id)

    def status(self) -> str:
        if self.behavior_agent is not None and self.behavior_task_id is not None:
            try:
                status = self.behavior_agent.get_task_status(self.behavior_task_id)
                return f"behavior={self.action}, status={status}"
            except Exception:
                return f"behavior={self.action}"
        return f"action={self.action}" if self.action else "action"

    def cancel(self) -> None:
        cancel_task(self.behavior_agent, self.behavior_task_id)


class CommandNode(PlanNode):
    def __init__(self, command: str):
        self.command = str(command)
        self.child: PlanNode | None = None
        self.command_text = ""

    def tick(self, controller) -> bool:
        if self.child is None:
            self.command_text = controller.render_command(self.command)
            spec = legacy_command_to_action(self.command_text)
            if spec is None:
                print(f"[people_control_test] Unsupported legacy command; releasing {controller.character_name}: {self.command_text}")
                controller.released = True
                return True
            self.child = make_plan_node(spec)
        return self.child.tick(controller)

    def status(self) -> str:
        if self.child is None:
            return f"legacy={command_type(self.command)}"
        return self.child.status()

    def cancel(self) -> None:
        if self.child is not None:
            self.child.cancel()


class LookAroundNode(PlanNode):
    def __init__(self, spec: dict):
        self.raw_spec = dict(spec)
        self.spec = {}
        self.started_at = None
        self.next_look_at = 0.0
        self.duration = None
        self.interval = LOOK_AROUND_DEFAULT_INTERVAL
        self.radius = LOOK_AT_DEFAULT_RADIUS
        self.behavior_agent = None
        self.behavior_task_id = None

    def tick(self, controller) -> bool:
        now = time.monotonic()
        if self.started_at is None:
            self.spec = controller.render_action(self.raw_spec)
            self.duration = optional_duration(self.spec, LOOK_AT_DEFAULT_DURATION)
            self.interval = float(self.spec.get("interval", LOOK_AROUND_DEFAULT_INTERVAL))
            self.radius = float(self.spec.get("radius", LOOK_AT_DEFAULT_RADIUS))
            self.started_at = now
            self.next_look_at = 0.0

        if self.duration is not None and self.duration > 0.0 and now - self.started_at >= self.duration:
            self.cancel()
            return True

        if now >= self.next_look_at:
            self.behavior_agent, _path = get_behavior_agent(controller.character_name)
            if self.behavior_agent is None:
                print(f"[people_control_test] look_around behavior agent is not ready for {controller.character_name}.")
                controller.released = True
                return True

            target = random_look_at_target(self.behavior_agent, self.radius, LOOK_AT_DEFAULT_HEIGHT)
            if target is not None:
                look_duration = max(0.1, min(self.interval + 0.2, self.duration or self.interval + 0.2))
                try:
                    self.behavior_task_id = self.behavior_agent.look_at(target=target, duration=look_duration)
                except Exception as exc:
                    print(f"[people_control_test] look_around failed for {controller.character_name}: {exc}")
            self.next_look_at = now + max(0.1, self.interval)

        return False

    def status(self) -> str:
        if self.started_at is None or self.duration is None:
            return "look_around"
        if self.duration <= 0.0:
            return "look_around=forever"
        remaining = max(0.0, self.duration - (time.monotonic() - self.started_at))
        return f"look_around={remaining:.1f}s"

    def cancel(self) -> None:
        cancel_task(self.behavior_agent, self.behavior_task_id)


class TalkOverlayNode(PlanNode):
    def __init__(self, spec: dict):
        self.raw_spec = dict(spec)
        self.spec = {}
        self.started_at = None
        self.next_gesture_at = 0.0
        self.gesture_index = 0
        self.duration = TALK_DEFAULT_DURATION
        self.interval = TALK_DEFAULT_INTERVAL
        self.sequence = list(TALK_DEFAULT_GESTURES)
        self.hand = "right"
        self.target_character = ""
        self.source_agent = None
        self.target_agent = None
        self.task_ids: list[tuple[object, int]] = []

    def _start_task(self, agent, task_id) -> None:
        if task_id is not None and task_id != behavior_task_id_invalid():
            self.task_ids.append((agent, task_id))

    def _start_look_at(self, source_name: str, target_name: str, duration: float) -> None:
        source_agent, _source_path = get_behavior_agent(source_name)
        target_path = character_target_path(target_name)
        if source_agent is None or target_path is None:
            return
        try:
            self._start_task(source_agent, source_agent.look_at(target=target_path, duration=duration))
        except Exception as exc:
            print(f"[people_control_test] talk look_at failed for {source_name} -> {target_name}: {exc}")

    def _start_pose(self, agent, preset: str) -> None:
        try:
            self._start_task(
                agent,
                agent.pose_hand(
                    hand_usage=behavior_hand_usage(self.hand),
                    preset=behavior_hand_pose(preset),
                    duration=max(0.1, self.interval),
                ),
            )
        except Exception as exc:
            print(f"[people_control_test] talk pose_hand failed: {exc}")

    def tick(self, controller) -> bool:
        now = time.monotonic()
        if self.started_at is None:
            self.spec = controller.render_action(self.raw_spec)
            self.target_character = str(self.spec.get("target_character", self.spec.get("target", ""))).strip()
            if not self.target_character:
                print(f"[people_control_test] talk requested without target_character for {controller.character_name}.")
                controller.released = True
                return True

            gesture = self.spec.get("gesture", {}) if isinstance(self.spec.get("gesture"), dict) else {}
            self.hand = str(gesture.get("hand", self.spec.get("hand", "right")))
            self.sequence = list(gesture.get("sequence", self.spec.get("sequence", TALK_DEFAULT_GESTURES)))
            if not self.sequence:
                self.sequence = list(TALK_DEFAULT_GESTURES)
            self.interval = float(gesture.get("interval", self.spec.get("interval", TALK_DEFAULT_INTERVAL)))
            self.duration = optional_duration(self.spec, TALK_DEFAULT_DURATION)
            self.source_agent, _source_path = get_behavior_agent(controller.character_name)
            self.target_agent, _target_path = get_behavior_agent(self.target_character)
            if self.source_agent is None or self.target_agent is None:
                print(
                    f"[people_control_test] talk behavior agent is not ready: "
                    f"{controller.character_name} -> {self.target_character}"
                )
                controller.released = True
                return True

            look_duration = self.duration if self.duration and self.duration > 0.0 else self.interval * 2.0
            self._start_look_at(controller.character_name, self.target_character, look_duration)
            self._start_look_at(self.target_character, controller.character_name, look_duration)
            self.started_at = now
            self.next_gesture_at = 0.0
            print(
                f"[people_control_test] talk overlay started: "
                f"{controller.character_name} <-> {self.target_character}, hand={self.hand}, sequence={self.sequence}"
            )

        if self.duration is not None and self.duration > 0.0 and now - self.started_at >= self.duration:
            self.cancel()
            return True

        if now >= self.next_gesture_at:
            preset = str(self.sequence[self.gesture_index % len(self.sequence)])
            self._start_pose(self.source_agent, preset)
            self._start_pose(self.target_agent, preset)
            self.gesture_index += 1
            self.next_gesture_at = now + max(0.1, self.interval)

        self.task_ids = [(agent, task_id) for agent, task_id in self.task_ids if task_is_running(agent, task_id)]
        return False

    def status(self) -> str:
        if self.started_at is None:
            return "talk"
        if self.duration is None or self.duration <= 0.0:
            return f"talk={self.target_character}"
        remaining = max(0.0, self.duration - (time.monotonic() - self.started_at))
        return f"talk={self.target_character}, {remaining:.1f}s"

    def cancel(self) -> None:
        for agent, task_id in self.task_ids:
            cancel_task(agent, task_id)
        self.task_ids.clear()


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
        if self.index >= len(self.children):
            return True
        if not self.children[self.index].tick(controller):
            return False
        if controller.released:
            return True
        self.index += 1
        return self.index >= len(self.children)

    def status(self) -> str:
        if not self.children:
            return "sequence=done"
        return f"step={min(self.index + 1, len(self.children))}/{len(self.children)}"

    def cancel(self) -> None:
        if 0 <= self.index < len(self.children):
            self.children[self.index].cancel()


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

    def cancel(self) -> None:
        for index, child in enumerate(self.children):
            if index not in self.done_indexes:
                child.cancel()


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
        if controller.released:
            return True

        self.completed += 1
        if self.count != "inf" and self.completed >= self.count:
            return True

        self.child = make_plan_node(self.child_spec)
        return False

    def status(self) -> str:
        if self.count == "inf":
            return f"loop={self.completed + 1}/inf"
        return f"loop={min(self.completed + 1, self.count)}/{self.count}"

    def cancel(self) -> None:
        self.child.cancel()


def make_plan_node(spec) -> PlanNode:
    if isinstance(spec, str):
        return CommandNode(spec)
    if isinstance(spec, list):
        return SequenceNode([make_plan_node(step) for step in spec])
    if not isinstance(spec, dict):
        return PlanNode()

    if "repeat" in spec and action_name(spec) not in {"repeat", "patrol"}:
        repeated_spec = dict(spec)
        repeat_count = repeated_spec.pop("repeat")
        return RepeatNode(repeated_spec, repeat_count)

    action = action_name(spec)
    if action == "wait":
        return WaitNode(spec.get("seconds", spec.get("duration", 0.0)))
    if action == "repeat":
        child_spec = spec.get("child")
        if child_spec is None:
            child_spec = {"steps": spec.get("steps", [])} if "steps" in spec else spec.get("action_spec", {})
        return RepeatNode(child_spec, spec.get("count", spec.get("repeat", 1)))
    if action == "patrol":
        points = spec.get("points", spec.get("waypoints", []))
        steps = []
        for point in points:
            if isinstance(point, dict):
                move_step = {
                    "character": spec.get("character"),
                    "action": "move_to",
                    "position": point.get("position", point.get("target", point)),
                    "yaw": point.get("yaw", spec.get("yaw", 0.0)),
                    "auto_brake": point.get("auto_brake", spec.get("auto_brake", True)),
                }
                steps.append(move_step)
                if point.get("wait") is not None or point.get("seconds") is not None:
                    steps.append({"action": "wait", "seconds": point.get("wait", point.get("seconds", 0.0))})
            else:
                steps.append(
                    {
                        "character": spec.get("character"),
                        "action": "move_to",
                        "position": point,
                        "yaw": spec.get("yaw", 0.0),
                        "auto_brake": spec.get("auto_brake", True),
                    }
                )
        patrol_plan = {"steps": steps}
        return RepeatNode(patrol_plan, spec.get("count", spec.get("repeat", "forever")))
    if action == "look_around":
        return LookAroundNode(spec)
    if action in {"talk", "talk_with"}:
        return TalkOverlayNode(spec)
    if action in NATIVE_ACTIONS:
        return ActionNode(spec)

    if "steps" in spec and not spec.get("type"):
        return SequenceNode([make_plan_node(step) for step in spec.get("steps", [])])

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

    def render_action(self, spec: dict) -> dict:
        variables = dict(self.variables)
        variables.setdefault("character", self.character_name)
        rendered = render_templates(spec, variables)
        return rendered if isinstance(rendered, dict) else {}

    def tick(self) -> bool:
        if self.released:
            return True
        done = self.plan.tick(self)
        return self.released or done

    def cancel(self) -> None:
        self.plan.cancel()
        self.released = True

    def status_line(self) -> str:
        return f"{self.character_name}: {self.label}, {self.plan.status()}"


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
        for controller_key, plan_spec in character_plans.items():
            if isinstance(plan_spec, dict) and plan_spec.get("__overlay"):
                character_name = str(plan_spec.get("__character", controller_key))
                self.add_controller(
                    str(controller_key),
                    character_name,
                    plan_spec.get("__plan", {}),
                    variables,
                    f"{label} overlay",
                    replace=False,
                )
            else:
                self.add_controller(str(controller_key), str(controller_key), plan_spec, variables, label, replace=True)
        refresh_status(force_log=True)

    def stop_all(self) -> None:
        for controller in self.controllers.values():
            controller.cancel()
        self.controllers.clear()
        self.label = "idle"
        if STATUS_LABEL is not None:
            STATUS_LABEL.text = "Scenario: idle"

    def add_controller(
        self,
        controller_key: str,
        character_name: str,
        plan_spec,
        variables: dict,
        label: str,
        replace: bool,
    ) -> None:
        old_controller = self.controllers.pop(controller_key, None)
        if old_controller is not None:
            old_controller.cancel()
        self.controllers[controller_key] = CharacterController(character_name, plan_spec, variables, label)

    def replace_controller(self, character_name: str, plan_spec, variables: dict, label: str) -> None:
        self.add_controller(character_name, character_name, plan_spec, variables, label, replace=True)

    def tick(self) -> None:
        for character_name, controller in list(self.controllers.items()):
            if controller.tick():
                self.controllers.pop(character_name, None)
        if not self.controllers:
            self.label = "idle"

    def status_lines(self) -> list[str]:
        return [controller.status_line() for _, controller in sorted(self.controllers.items())]

    def _compile_scenario(self, scenario, variables: dict) -> dict[str, object]:
        if isinstance(scenario, dict) and isinstance(scenario.get("actions"), list):
            return self._compile_actions(scenario["actions"], variables)

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

    def _compile_actions(self, actions: list, variables: dict) -> dict[str, object]:
        grouped: dict[str, list] = {}
        overlay_plans: dict[str, object] = {}
        for index, raw_action in enumerate(actions):
            if not isinstance(raw_action, dict):
                continue
            rendered = render_templates(raw_action, variables)
            if not isinstance(rendered, dict):
                continue
            character_name = str(rendered.get("character", "")).strip()
            if not character_name:
                print(f"[people_control_test] Structured action is missing character: {rendered}")
                continue
            if action_name(rendered) in {"talk", "talk_with"}:
                overlay_plans[f"{character_name}#overlay#{index}"] = {
                    "__overlay": True,
                    "__character": character_name,
                    "__plan": rendered,
                }
                continue
            grouped.setdefault(character_name, []).append(rendered)

        plans = {}
        for character_name, character_actions in grouped.items():
            if len(character_actions) == 1:
                plans[character_name] = character_actions[0]
            else:
                plans[character_name] = {"steps": character_actions}
        plans.update(overlay_plans)
        return plans

    def _compile_command_list(self, commands: list[str], count, variables: dict) -> dict[str, object]:
        grouped: dict[str, list[str]] = {}
        for command in commands:
            rendered = normalize_command_line(format_template(command, variables))
            character_name = command_agent_name(rendered)
            if character_name:
                grouped.setdefault(character_name, []).append(rendered)
        return {character_name: command_plan(character_commands, count) for character_name, character_commands in grouped.items()}


SCENARIO_RUNNER = ScenarioRunner()


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
    elif action == "go_to_selected":
        SCENARIO_RUNNER.stop_all()
        if scenario_name:
            start_named_scenario(str(scenario_name), selected_character, x_model, y_model, r_model)
        else:
            variables = ui_variables(selected_character, x_model, y_model, r_model)
            move_character_to_xy(variables["character"], variables["x"], variables["y"], variables["r"])
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
    print(f"[people_control_test] UI character names: {people}", flush=True)
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


enable_people_extensions()
configure_people()
open_stage()
strip_nested_rigid_bodies()
configure_people()
bake_navmesh()
ensure_behavior_agents()
control_window = build_ui()

omni.timeline.get_timeline_interface().play()
if PEOPLE_TEST_AUTO_LOOK_AT or PEOPLE_TEST_AUTO_GOTO or PEOPLE_TEST_AUTO_PATROL or PEOPLE_TEST_AUTO_SIT:
    for _ in range(30):
        simulation_app.update()

if PEOPLE_TEST_AUTO_GOTO:
    move_character_to_xy(
        os.environ.get("PEOPLE_TEST_AUTO_GOTO_CHARACTER", "Male_patient_01"),
        float(os.environ.get("PEOPLE_TEST_AUTO_GOTO_X", "6.7")),
        float(os.environ.get("PEOPLE_TEST_AUTO_GOTO_Y", "0.0")),
        float(os.environ.get("PEOPLE_TEST_AUTO_GOTO_R", "0.0")),
    )

if PEOPLE_TEST_AUTO_LOOK_AT:
    look_at_all_characters()

if PEOPLE_TEST_AUTO_PATROL:
    cfg = load_yaml_config()
    scenario = cfg.get("scenarios", {}).get("set_patrol")
    if scenario:
        SCENARIO_RUNNER.start("Set patrol", scenario, {})
        for _ in range(int(os.environ.get("PEOPLE_TEST_AUTO_PATROL_FRAMES", "120"))):
            refresh_status()
            simulation_app.update()

if PEOPLE_TEST_AUTO_SIT:
    cfg = load_yaml_config()
    scenario = cfg.get("scenarios", {}).get("set_sit")
    if scenario:
        SCENARIO_RUNNER.start("Set Sit", scenario, {})
        for _ in range(int(os.environ.get("PEOPLE_TEST_AUTO_SIT_FRAMES", "120"))):
            refresh_status()
            simulation_app.update()

if (
    PEOPLE_TEST_AUTO_LOOK_AT
    or PEOPLE_TEST_AUTO_GOTO
    or PEOPLE_TEST_AUTO_PATROL
    or PEOPLE_TEST_AUTO_SIT
) and PEOPLE_TEST_EXIT_AFTER_AUTO:
    for _ in range(30):
        simulation_app.update()
    simulation_app.close()
    raise SystemExit(0)

while simulation_app.is_running():
    refresh_status()
    simulation_app.update()

simulation_app.close()
