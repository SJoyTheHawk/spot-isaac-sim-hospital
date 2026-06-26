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
        return yaml.safe_load(f) or {}


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


def ensure_sit_effector(target_path: str) -> str:
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
    offset, rotation = apply_sit_hips_overrides(*sit_hips_default_transform(stage, target_path))
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
    target_path: str,
    log_prefix: str = "Sit",
) -> tuple[object | None, int | None]:
    character_name = str(character_name or "").strip()
    target_path = ensure_sit_effector(str(target_path or "").strip())
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
        task_id = agent.sit(target_path, snap_to_seat=SIT_SNAP_TO_SEAT)
    except Exception as exc:
        print(f"[people_control_test] {log_prefix} failed for {character_name}: {exc}")
        return None, None

    if task_id == invalid_task_id:
        print(f"[people_control_test] {log_prefix} rejected for {character_name}: target={target_path}")
        return None, None

    print(
        f"[people_control_test] {log_prefix} started: "
        f"character={character_name}, target={target_path}, snap_to_seat={SIT_SNAP_TO_SEAT}, task_id={task_id}"
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
        task_id = agent.idle()
    except Exception as exc:
        print(f"[people_control_test] {log_prefix} failed for {character_name}: {exc}")
        return None, None

    if task_id == invalid_task_id:
        print(f"[people_control_test] {log_prefix} rejected for {character_name}.")
        return None, None

    print(f"[people_control_test] {log_prefix} started: character={character_name}, task_id={task_id}")
    return agent, task_id


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


class CommandNode(PlanNode):
    def __init__(self, command: str):
        self.command = str(command)
        self.started = False
        self.started_at = None
        self.expected_command_name = ""
        self.command_text = ""
        self.behavior_agent = None
        self.behavior_task_id = None
        self.uses_behavior_agent = False
        self.duration = -1.0

    def tick(self, controller) -> bool:
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

            goto = parse_goto_command(self.command_text)
            if goto is not None:
                self.behavior_agent, self.behavior_task_id = start_behavior_goto(
                    goto["character"],
                    goto["x"],
                    goto["y"],
                    goto["z"],
                    goto["yaw"],
                    log_prefix="Patrol GoTo",
                )
                self.uses_behavior_agent = True
                self.started = True
                self.started_at = time.monotonic()
                if self.behavior_agent is None or self.behavior_task_id is None:
                    print(
                        "[people_control_test] Patrol command could not start; "
                        f"releasing {controller.character_name} controller."
                    )
                    controller.released = True
                    return True
                return False

            sit = parse_sit_command(self.command_text)
            if sit is not None:
                self.behavior_agent, self.behavior_task_id = start_behavior_sit(
                    sit["character"],
                    sit["target"],
                    log_prefix="Sit",
                )
                self.uses_behavior_agent = True
                self.duration = sit["duration"]
                self.started = True
                self.started_at = time.monotonic()
                if self.behavior_agent is None or self.behavior_task_id is None:
                    print(
                        "[people_control_test] Sit command could not start; "
                        f"releasing {controller.character_name} controller."
                    )
                    controller.released = True
                    return True
                return False

            idle = parse_idle_command(self.command_text)
            if idle is not None:
                self.behavior_agent, self.behavior_task_id = start_behavior_idle(
                    idle["character"],
                    log_prefix="Idle",
                )
                self.uses_behavior_agent = True
                self.duration = idle["duration"]
                self.started = True
                self.started_at = time.monotonic()
                if self.behavior_agent is None or self.behavior_task_id is None:
                    print(
                        "[people_control_test] Idle command could not start; "
                        f"releasing {controller.character_name} controller."
                    )
                    controller.released = True
                    return True
                return False

            print(
                "[people_control_test] Unsupported Isaac 6 command; "
                f"releasing {controller.character_name}: {self.command_text}"
            )
            controller.released = True
            self.started = True
            self.started_at = time.monotonic()
            return True

        if self.uses_behavior_agent:
            if self.behavior_agent is None or self.behavior_task_id is None:
                return True
            if self.duration > 0.0 and self.started_at is not None:
                if time.monotonic() - self.started_at >= self.duration:
                    self.cancel()
                    return True
            return not self.behavior_agent.is_task_running(self.behavior_task_id)

        return True

    def status(self) -> str:
        command = self.expected_command_name or command_type(self.command)
        if self.uses_behavior_agent and self.behavior_agent is not None and self.behavior_task_id is not None:
            try:
                status = self.behavior_agent.get_task_status(self.behavior_task_id)
                return f"behavior={command}, status={status}"
            except Exception:
                return f"behavior={command}"
        return f"command={command}" if command else "command"

    def cancel(self) -> None:
        if self.uses_behavior_agent and self.behavior_agent is not None and self.behavior_task_id is not None:
            try:
                if self.behavior_agent.is_task_running(self.behavior_task_id):
                    self.behavior_agent.cancel_task(self.behavior_task_id)
            except Exception as exc:
                print(f"[people_control_test] Unable to cancel behavior task {self.behavior_task_id}: {exc}")


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
            if controller.released:
                return True
            self.index += 1
        return True

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
