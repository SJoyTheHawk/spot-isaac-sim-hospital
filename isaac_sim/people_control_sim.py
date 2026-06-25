"""Standalone people-control test for Isaac Sim."""

import os

from isaacsim import SimulationApp


def _env_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None or value.strip() == "":
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


PEOPLE_TEST_HEADLESS = _env_flag("PEOPLE_TEST_HEADLESS", _env_flag("SPOT_ISAAC_HEADLESS", False))

simulation_app = SimulationApp({"headless": PEOPLE_TEST_HEADLESS})

import math
import random
import time
from pathlib import Path

import carb
import omni.timeline
import omni.ui as ui
import omni.usd
import yaml
from isaacsim.core.utils.extensions import enable_extension
from omni.behavior.scripting.core.scripts.script_manager import ScriptManager
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel


REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
USD_PATH = os.path.realpath(
    os.environ.get("PEOPLE_TEST_USD", os.path.join(REPO_DIR, "assets", "isaac_hospital_scene_spot_w_characters_6.usd"))
)
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
SEAT_PROXY_ROOT = "/World/PeopleTestSeatTargets"
LOOK_AT_DEFAULT_DURATION = 8.0
LOOK_AT_DEFAULT_RADIUS = 4.0
LOOK_AT_DEFAULT_HEIGHT = 1.45
PEOPLE_TEST_AUTO_LOOK_AT = _env_flag("PEOPLE_TEST_AUTO_LOOK_AT")
PEOPLE_TEST_EXIT_AFTER_AUTO = _env_flag("PEOPLE_TEST_EXIT_AFTER_AUTO")
STATUS_LABEL = None
LAST_STATUS_LOG_TIME = 0.0
SCENARIO_RUNNER = None

# STARTUP_SEATED_PEOPLE = {
#     "Female_visitor_02": "/World/hospital/SM_Chair_02a7",
#     "Male_visitor_02": "/World/hospital/SM_Chair_02a5",
#     "Male_visitor_01": "/World/hospital/SM_Chair_02a4",
#     "Male_patient_04": "/World/hospital/SM_WheelChair_01a4",
#     "Female_patient_05": "/World/hospital/SM_Chair_01a7",
#     "Male_patient_05": "/World/hospital/SM_Chair_01a12",
#     "Female_nurse_02": "/World/hospital/SM_Chair_01a13",
#     "Male_patient_01": "/World/hospital/SM_Chair_01a3",
# }


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


def setup_saved_characters() -> None:
    try:
        from isaacsim.replicator.agent.core.settings import BehaviorScriptPaths
        from isaacsim.replicator.agent.core.stage_util import CharacterUtil
    except ModuleNotFoundError:
        print("[people_control_test] Legacy character setup API is not available; using Isaac 6 authored characters.")
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


def init_behavior_scripts() -> None:
    script_manager = ScriptManager.get_instance()
    agent_manager = get_legacy_agent_manager()
    if agent_manager is None:
        print("[people_control_test] Legacy Replicator AgentManager is not available; skipping script command registration.")
        return

    for _ in range(50):
        simulation_app.update()
    for scripts in script_manager._prim_to_scripts.values():
        for _, inst in scripts.items():
            if inst and hasattr(inst, "init_character"):
                inst.on_play()
                if inst.init_character():
                    agent_manager.register_agent(inst.get_agent_name(), inst.prim_path)


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


def vec3_components(value) -> tuple[float, float, float]:
    try:
        return float(value[0]), float(value[1]), float(value[2])
    except (TypeError, IndexError, KeyError):
        return float(value.x), float(value.y), float(value.z)


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
setup_saved_characters()
ensure_behavior_agents()
init_behavior_scripts()
control_window = build_ui()

omni.timeline.get_timeline_interface().play()
if PEOPLE_TEST_AUTO_LOOK_AT:
    for _ in range(30):
        simulation_app.update()
    look_at_all_characters()
    if PEOPLE_TEST_EXIT_AFTER_AUTO:
        for _ in range(30):
            simulation_app.update()
        simulation_app.close()
        raise SystemExit(0)

while simulation_app.is_running():
    refresh_status()
    simulation_app.update()

simulation_app.close()
