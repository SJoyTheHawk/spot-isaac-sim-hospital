"""Standalone people-control test for Isaac Sim."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import os
import time
from pathlib import Path

import carb
import omni.timeline
import omni.ui as ui
import omni.usd
import yaml
from isaacsim.core.utils.extensions import enable_extension
from omni.kit.scripting.scripts.script_manager import ScriptManager
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


REPO_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
USD_PATH = os.path.realpath(
    os.environ.get("PEOPLE_TEST_USD", os.path.join(REPO_DIR, "assets", "isaac_hospital_scene_spot_w_characters.usd"))
)
COMMANDS_YAML_FILE = os.path.realpath(
    os.environ.get("PEOPLE_INITIAL_COMMANDS", os.path.join(REPO_DIR, "assets", "people_initial_commands.yaml"))
)
PEOPLE_COMMAND_FILE = os.path.realpath(
    os.environ.get("PEOPLE_COMMAND_FILE", os.path.join(REPO_DIR, "assets", "people_runtime_commands.txt"))
)
CHARACTER_ROOT = "/World/Characters"
SEAT_PROXY_ROOT = "/World/PeopleTestSeatTargets"
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
    for ext in [
        "omni.kit.scripting",
        "omni.anim.timeline",
        "omni.anim.graph.core",
        "omni.anim.retarget.core",
        "omni.anim.navigation.core",
        "omni.anim.people",
        "isaacsim.replicator.agent.core",
        "omni.kit.mesh.raycast",
    ]:
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


def configure_people() -> None:
    settings = carb.settings.get_settings()
    settings.set("/persistent/exts/omni.anim.people/character_prim_path", CHARACTER_ROOT)
    settings.set("/exts/isaacsim.replicator.agent/characters_parent_prim_path", CHARACTER_ROOT)
    settings.set("/exts/omni.anim.people/command_settings/command_file_path", PEOPLE_COMMAND_FILE)
    settings.set("/exts/omni.anim.people/command_settings/number_of_loop", 0)
    settings.set("/exts/omni.anim.people/navigation_settings/navmesh_enabled", True)
    settings.set("/exts/omni.anim.people/navigation_settings/dynamic_avoidance_enabled", True)


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
    return [child.GetName() for child in root.GetAllChildren() if child.GetName() != "Biped_Setup"]


def setup_saved_characters() -> None:
    from isaacsim.replicator.agent.core.settings import BehaviorScriptPaths
    from isaacsim.replicator.agent.core.stage_util import CharacterUtil

    biped_prim = CharacterUtil.load_default_biped_to_stage()
    anim_graph = CharacterUtil.get_anim_graph_from_character(biped_prim)
    skelroots = [prim for prim in CharacterUtil.get_characters_in_stage() if "/Biped_Setup/" not in str(prim.GetPath())]
    CharacterUtil.setup_animation_graph_to_character(skelroots, anim_graph)
    CharacterUtil.setup_python_scripts_to_character(skelroots, BehaviorScriptPaths.behavior_script_path())
    for _ in range(15):
        simulation_app.update()


def init_behavior_scripts() -> None:
    from isaacsim.replicator.agent.core.agent_manager import AgentManager

    script_manager = ScriptManager.get_instance()
    agent_manager = AgentManager.get_instance()
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
    from isaacsim.replicator.agent.core.agent_manager import AgentManager

    return AgentManager.get_instance().get_agent_script_instance_by_name(name)


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


enable_people_extensions()
configure_people()
open_stage()
strip_nested_rigid_bodies()
configure_people()
bake_navmesh()
setup_saved_characters()
init_behavior_scripts()
control_window = build_ui()

omni.timeline.get_timeline_interface().play()
while simulation_app.is_running():
    refresh_status()
    simulation_app.update()

simulation_app.close()
