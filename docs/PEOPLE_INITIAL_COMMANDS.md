# People Initial Commands YAML Guide

This guide explains how to edit `assets/people_initial_commands.yaml` for the Isaac Sim 6 people-control test UI.

The file describes the buttons shown by `scripts/run_people_sim.sh` and the scenarios those buttons start. Scenarios use structured YAML action objects instead of the old Isaac Sim 5.1 command strings.

## File Layout

The YAML has four main sections:

```yaml
behavior_catalog:
  native: []
  scheduler: []
  composite: []

buttons:
  - label: "Initialize"
    action: "reset"
    scenario: "initialize"

seat_defaults: &seat_defaults
  hips_offset: [0.0, 0.0, 0.0]
  hips_rotation: [90.0, 0.0, 0.0]
  snap_to_seat: true

scenarios:
  initialize:
    label: "Initialize characters"
    actions: []
```

`behavior_catalog` lists the action names supported by the current Python runner. It is mainly documentation and validation help.

`buttons` defines the 3 x 3 UI button grid. Only the first nine entries are used.

`seat_defaults` and `talk_gesture` are YAML anchors. They avoid repeating common sit and talk settings.

`scenarios` contains the actual character behavior plans.

## Buttons

A button can run a built-in UI action, a scenario, or both.

```yaml
- label: "Set Sit"
  scenario: "set_sit"
```

Common button fields:

| Field | Meaning |
|---|---|
| `label` | Text shown on the UI button. |
| `action` | Optional built-in UI action such as `reset`, `go_to_selected`, or `look_at_all`. |
| `scenario` | Optional scenario name from the `scenarios` section. |
| `duration` | Used by `look_at_all` button action. |
| `radius` | Used by `look_at_all` button action. |

Current built-in button actions:

| Button action | Effect |
|---|---|
| `reset` | Stops all active scenario controllers before running the optional scenario. |
| `go_to_selected` | Uses the selected character and the UI X/Y/Yaw fields. If a scenario is provided, it runs that scenario with template values. |
| `look_at_all` | Makes all characters look at random nearby points once. |
| `stop` | Stops all active scenario controllers. |

For the current GoTo button, the YAML points to `selected_goto`:

```yaml
- label: "GoTo"
  action: "go_to_selected"
  scenario: "selected_goto"
```

That scenario uses template variables from the UI fields:

```yaml
selected_goto:
  label: "GoTo"
  actions:
    - character: "{character}"
      action: move_to
      position: ["{x}", "{y}", 0.0]
      yaw: "{r}"
```

The Fall button uses the selected character only:

```yaml
- label: "Fall"
  scenario: "selected_fall"

selected_fall:
  label: "Fall"
  actions:
    - character: "{character}"
      action: fall
```

## Scenario Rules

Each scenario has a `label` and an `actions` list.

```yaml
scenarios:
  set_sit:
    label: "Set Sit"
    actions:
      - character: Male_patient_01
        action: sit
        target:
          <<: *seat_defaults
          prim: /World/Chair/SM_Chair_01a3
```

Top-level entries in `actions` run in parallel when they belong to different characters.

If several top-level actions use the same character, they run in order for that character.

Per-character `steps` always run in order:

```yaml
- character: Female_patient_02
  repeat: forever
  steps:
    - action: move_to
      position: [6.53459, 17.05795, 0.0]
      yaw: 90.0
    - action: wait
      seconds: 60
    - action: move_to
      position: [6.89391, 5.64066, 0.0]
      yaw: -90.0
    - action: wait
      seconds: 20
```

`talk` and `talk_with` are special overlay actions. They do not replace the character's main sit/walk plan. They start a separate overlay controller that makes both characters look at each other and gesture.

## Duration And Wait

Use `duration` only when an action should be held, looped, or overlaid for a set time.

Good uses of `duration`:

```yaml
- action: idle
  duration: 10

- action: look_around
  duration: 60

- action: talk_with
  target_character: Male_patient_05
  duration: 9999
```

For scheduler pauses, use `wait.seconds`, not `duration`:

```yaml
- action: wait
  seconds: 60
```

If `duration` is omitted, the Isaac behavior task runs until its natural completion. This is useful for normal `move_to`, long-running `sit`, and long-running `idle` actions.

## Native Actions

These actions map directly to Isaac Sim 6 `BehaviorAgent` calls.

| YAML action | Important fields | Notes |
|---|---|---|
| `idle` | optional `facing` or `yaw`, optional `duration` | Keeps a character idle. |
| `move_to` | `position` or `target`, optional `yaw`, optional `auto_brake` | Best replacement for old single-point `GoTo`. |
| `move_along` | `waypoints` or curve `target`, optional `start_from_closest_point`, optional `auto_brake` | Use for continuous path walking without per-point waits. |
| `follow` | `target` or `target_character`, optional `distance` | Follow another prim or character. |
| `dodge` | `direction`, optional `motion_scale` | Direction is `[x, y, z]`. |
| `fall` | none | Triggers fall behavior. |
| `sit` | `target.prim`, optional seat offsets/rotation | Creates/updates `/Behavior/Sit` and `/Behavior/Sit_Hips` under the target prim. |
| `ride` | `target` | Target can be a prim path. |
| `pickup_object` | `target`, optional `snap_to_hand` | Target is the object prim. |
| `place_object` | `target`, `placement_target` | Places object at another target. |
| `release_object` | `target` | Releases held object. |
| `custom_action` | `name`, optional `root_animation`, optional `duration` | Runs a named behavior action. |
| `look_at` | `target`, `target_character`, or `position`, optional `duration` | Direct look-at action. |
| `reach_hand` | `hand`, `target` or `position`, optional direction fields | `hand` can be `right`, `left`, or `both`. |
| `pose_hand` | `hand`, `preset`, optional `duration` | Presets include `open`, `point`, `relaxed`, and `fist`. |
| `reset` | optional `position`, optional `facing` | Calls BehaviorAgent reset. |
| `teleport` | `position` or `target`, optional `facing` | Moves the character instantly. |

`facing` can be a yaw angle in degrees, a direction vector such as `[1.0, 0.0, 0.0]`, or a prim path. Numeric yaw values are converted by the runner to the direction vector that Isaac Sim 6 expects.

### move_to

Use `move_to` for a single destination.

```yaml
- character: Male_patient_01
  action: move_to
  position: [6.7, 0.0, 0.0]
  yaw: -90.0
  auto_brake: true
```

`position` is `[x, y, z]`. In this project, `z` is usually `0.0`. The runner snaps position targets to the navmesh when possible.

### move_along

Use `move_along` when the character should walk a continuous path, not stop at each point.

```yaml
- character: Male_police_01
  action: move_along
  waypoints:
    - [-28.4, 4.2, 0.0]
    - [-25.8, 10.1, 0.0]
    - [5.0, 10.1, 0.0]
  start_from_closest_point: true
  auto_brake: true
```

For patrol routes where the character may wait, look around, or do other behavior at points, use `steps` with repeated `move_to` instead.

### sit

Use `sit` with a target prim path. The runner authors or updates the required Isaac Sim 6 sit effector prims under the target:

```text
/World/Chair/SM_Chair_01a3/Behavior/Sit
/World/Chair/SM_Chair_01a3/Behavior/Sit_Hips
```

Recommended format:

```yaml
- character: Male_patient_01
  action: sit
  target:
    prim: /World/Chair/SM_Chair_01a3
    hips_offset: [0.0, 0.0, 0.0]
    hips_rotation: [90.0, 0.0, 0.0]
    snap_to_seat: true
```

You can use the shared defaults:

```yaml
- character: Male_patient_01
  action: sit
  target:
    <<: *seat_defaults
    prim: /World/Chair/SM_Chair_01a3
```

Adjust `hips_offset` when the body is too far forward/back/left/right/up/down relative to the chair. Adjust `hips_rotation` when the sitting orientation is wrong.

### look_at

Look at another character:

```yaml
- character: Female_nurse_05
  action: look_at
  target_character: Male_patient_05
  duration: 5
```

Look at a fixed point:

```yaml
- character: Female_nurse_05
  action: look_at
  position: [4.0, 3.0, 1.5]
  duration: 5
```

### pose_hand

Use `pose_hand` for simple gestures.

```yaml
- character: Female_nurse_05
  action: pose_hand
  hand: right
  preset: point
  duration: 2
```

Useful presets are `open`, `point`, `relaxed`, and `fist`.

## Scheduler And Composite Actions

These actions are handled by the project runner, not directly by Isaac.

| YAML action | Important fields | Notes |
|---|---|---|
| `wait` | `seconds` | Pauses one character sequence. |
| `repeat` | `count` or `repeat` | Repeats one action or a step list. Usually use `repeat: forever`. |
| `patrol` | `points`, optional `repeat` | Shortcut for repeating `move_to` points. |
| `look_around` | optional `duration`, `radius`, `interval` | Repeated random `look_at`. |
| `talk` | `target_character`, optional `gesture`, optional `duration` | Overlay gesture/look behavior. |
| `talk_with` | `target_character`, optional `gesture`, optional `duration` | Same overlay behavior, named for two-person use. |

### repeat with steps

Use this for special routines with waits or mixed behavior.

```yaml
- character: Male_patient_02
  repeat: forever
  steps:
    - action: move_to
      position: [-32.6271, 3.23897, 0.0]
      yaw: -90.0
    - action: look_around
      duration: 60
    - action: move_to
      position: [-3.99849, 11.27831, 0.0]
      yaw: 90.0
    - action: look_around
      duration: 60
```

### patrol

Use `patrol` for repeated movement through points with no extra behavior between points.

```yaml
- character: Female_police_01
  action: patrol
  repeat: forever
  points:
    - position: [19.2, 30.89, 0.0]
      yaw: 0.0
    - position: [17.5, 3.19, 0.0]
      yaw: 180.0
```

If you need waits at patrol points, use `steps` instead:

```yaml
- character: Female_police_01
  repeat: forever
  steps:
    - action: move_to
      position: [19.2, 30.89, 0.0]
      yaw: 0.0
    - action: wait
      seconds: 10
    - action: move_to
      position: [17.5, 3.19, 0.0]
      yaw: 180.0
```

### look_around

Use this when one character should keep looking at random nearby points.

```yaml
- character: Female_visitor_04
  action: look_around
  duration: 9999
  radius: 4
  interval: 3
```

`radius` and `interval` are optional. If omitted, the runner uses its defaults.

### talk and talk_with

`talk` and `talk_with` are overlays. They do not stop the main plan for either character.

The overlay does two things:

1. Both characters look at each other.
2. Both characters loop subtle hand poses.

```yaml
- character: Female_nurse_05
  action: talk_with
  target_character: Male_patient_05
  duration: 9999
  gesture:
    hands:
      left:
        action: [open, relaxed]
      right:
        action: [relaxed, point]
    interval: 1.8
    look_height: 1.45
    reach_height: 1.05
    reach_distance: 0.55
    hand_spread: 0.28
    motion_scale: 0.45
    interval_jitter: 0.35
    look_height_jitter: 0.12
    reach_height_jitter: 0.12
    reach_distance_jitter: 0.2
    initial_delay_jitter: 1.2
    response_chance: 0.65
    randomize: true
```

`look_height` aims gaze at the other person's upper body instead of their root prim near the floor. `reach_height` and `reach_distance` place the hand gesture forward at chest height.
`hands.left.action` and `hands.right.action` send separate hand pose actions. `hand_spread` offsets those hand targets sideways so both hands do not point at the same exact point.
The jitter fields add small random changes each gesture cycle. `response_chance` controls how often both people gesture in the same cycle; lower values make the conversation alternate more.
`initial_delay_jitter` staggers the first gesture so multiple talking pairs do not begin on the same frame.

You can reuse the shared gesture anchor:

```yaml
- character: Female_nurse_05
  action: talk_with
  target_character: Male_patient_05
  duration: 9999
  gesture: *talk_gesture
```

## Action Prototypes

This section gives a copy-paste prototype for each supported case. Replace character names, prim paths, positions, and durations for your scene.

### Scenario prototype

```yaml
scenarios:
  my_scenario:
    label: "My Scenario"
    actions:
      - character: Male_patient_01
        action: idle
```

### Button prototypes

Run a scenario:

```yaml
- label: "My Scenario"
  scenario: "my_scenario"
```

Stop current controllers, then run a scenario:

```yaml
- label: "Initialize"
  action: "reset"
  scenario: "initialize"
```

Use the selected character and X/Y/Yaw UI fields:

```yaml
- label: "GoTo"
  action: "go_to_selected"
  scenario: "selected_goto"
```

Use the selected character without position fields:

```yaml
- label: "Fall"
  scenario: "selected_fall"
```

Trigger one random look-at action for all characters:

```yaml
- label: "Look Around"
  action: "look_at_all"
  duration: 8
  radius: 4
```

Stop all active scenario controllers:

```yaml
- label: "Stop"
  action: "stop"
```

### idle

```yaml
- character: Male_patient_01
  action: idle
```

Timed idle inside a sequence:

```yaml
- action: idle
  duration: 10
```

### move_to

```yaml
- character: Male_patient_01
  action: move_to
  position: [6.7, 0.0, 0.0]
  yaw: -90.0
  auto_brake: true
```

With the UI selected character fields:

```yaml
- character: "{character}"
  action: move_to
  position: ["{x}", "{y}", 0.0]
  yaw: "{r}"
```

### move_along

```yaml
- character: Male_police_01
  action: move_along
  waypoints:
    - [-28.4, 4.2, 0.0]
    - [-25.8, 10.1, 0.0]
    - [5.0, 10.1, 0.0]
  start_from_closest_point: true
  auto_brake: true
```

With a curve or path prim target:

```yaml
- character: Male_police_01
  action: move_along
  target: /World/Paths/GuardRoute
  start_from_closest_point: true
  auto_brake: true
```

### follow

```yaml
- character: Female_nurse_05
  action: follow
  target_character: Male_patient_05
  distance: 1.5
```

Follow a prim:

```yaml
- character: Female_nurse_05
  action: follow
  target: /World/SomeMovingPrim
  distance: 1.5
```

### dodge

```yaml
- character: Male_patient_01
  action: dodge
  direction: [1.0, 0.0, 0.0]
  motion_scale: 1.0
```

### fall

`fall` is reserved for a future stable fall animation. Native Isaac ragdoll fall is not enabled by this project because enabling `/exts/omni.anim.behavior.core/enableRagdollPhysics` crashes the full hospital scene in Isaac Sim 6.0.1. For now, the Fall button logs a skip instead of calling `agent.fall()`.

```yaml
- character: Male_patient_01
  action: fall
```

### sit

```yaml
- character: Male_patient_01
  action: sit
  target:
    prim: /World/Chair/SM_Chair_01a3
    hips_offset: [0.0, 0.0, 0.0]
    hips_rotation: [90.0, 0.0, 0.0]
    snap_to_seat: true
```

Using shared defaults:

```yaml
- character: Male_patient_01
  action: sit
  target:
    <<: *seat_defaults
    prim: /World/Chair/SM_Chair_01a3
```

### ride

```yaml
- character: Male_patient_01
  action: ride
  target: /World/Wheelchair
```

### pickup_object

```yaml
- character: Male_patient_01
  action: pickup_object
  target: /World/Props/Cup_01
  snap_to_hand: true
```

### place_object

```yaml
- character: Male_patient_01
  action: place_object
  target: /World/Props/Cup_01
  placement_target: /World/Table/Tabletop_Target
```

### release_object

```yaml
- character: Male_patient_01
  action: release_object
  target: /World/Props/Cup_01
```

### custom_action

```yaml
- character: Male_patient_01
  action: custom_action
  name: SomeCustomActionName
  root_animation: ignore
  duration: 5
```

### look_at

Look at another character:

```yaml
- character: Female_nurse_05
  action: look_at
  target_character: Male_patient_05
  duration: 5
```

Look at a point:

```yaml
- character: Female_nurse_05
  action: look_at
  position: [4.0, 3.0, 1.5]
  duration: 5
```

Look at a prim:

```yaml
- character: Female_nurse_05
  action: look_at
  target: /World/Props/Monitor_01
  duration: 5
```

### reach_hand

```yaml
- character: Female_nurse_05
  action: reach_hand
  hand: right
  target: /World/Props/Cup_01
  palm_direction: [0.0, 0.0, -1.0]
  finger_direction: [1.0, 0.0, 0.0]
  motion_scale: 1.0
  duration: 3
```

Reach to a point:

```yaml
- character: Female_nurse_05
  action: reach_hand
  hand: left
  position: [3.0, 2.0, 1.1]
  duration: 3
```

### pose_hand

```yaml
- character: Female_nurse_05
  action: pose_hand
  hand: right
  preset: point
  duration: 2
```

### reset

```yaml
- character: Male_patient_01
  action: reset
```

Reset to a position and facing:

```yaml
- character: Male_patient_01
  action: reset
  position: [0.0, 0.0, 0.0]
  facing: 90.0
```

Equivalent vector form:

```yaml
- character: Male_patient_01
  action: reset
  position: [0.0, 0.0, 0.0]
  facing: [0.0, 1.0, 0.0]
```

### teleport

```yaml
- character: Male_patient_01
  action: teleport
  position: [0.0, 0.0, 0.0]
  facing: 90.0
```

### wait

Only use `wait` inside `steps`.

```yaml
- action: wait
  seconds: 60
```

### repeat one action

```yaml
- character: Female_visitor_04
  repeat: forever
  action: look_around
  duration: 8
```

### repeat steps

```yaml
- character: Female_patient_02
  repeat: forever
  steps:
    - action: move_to
      position: [6.53459, 17.05795, 0.0]
      yaw: 90.0
    - action: wait
      seconds: 60
    - action: move_to
      position: [6.89391, 5.64066, 0.0]
      yaw: -90.0
    - action: wait
      seconds: 20
```

### patrol

```yaml
- character: Female_police_01
  action: patrol
  repeat: forever
  points:
    - position: [19.2, 30.89, 0.0]
      yaw: 0.0
    - position: [17.5, 3.19, 0.0]
      yaw: 180.0
```

Patrol with per-point waits should be written as repeated steps:

```yaml
- character: Female_police_01
  repeat: forever
  steps:
    - action: move_to
      position: [19.2, 30.89, 0.0]
      yaw: 0.0
    - action: wait
      seconds: 10
    - action: move_to
      position: [17.5, 3.19, 0.0]
      yaw: 180.0
```

### look_around

```yaml
- character: Female_visitor_04
  action: look_around
  duration: 9999
  radius: 4
  interval: 3
```

### talk

```yaml
- character: Male_nurse_04
  action: talk
  target_character: Male_nurse_02
  duration: 9999
  gesture:
    hands:
      left:
        action: [open, relaxed]
      right:
        action: [relaxed, point]
    interval: 1.8
```

### talk_with

```yaml
- character: Female_nurse_05
  action: talk_with
  target_character: Male_patient_05
  duration: 9999
  gesture: *talk_gesture
```

### parallel actions

Different top-level actions run together:

```yaml
actions:
  - character: Male_patient_01
    action: sit
    target:
      <<: *seat_defaults
      prim: /World/Chair/SM_Chair_01a3

  - character: Female_nurse_05
    action: talk_with
    target_character: Male_patient_01
    duration: 9999
    gesture: *talk_gesture
```

### sequential actions for one character

Use `steps` when one character must do actions in order:

```yaml
- character: Female_nurse_04
  repeat: forever
  steps:
    - action: sit
      duration: 10
      target:
        <<: *seat_defaults
        prim: /World/Chair/SM_Chair_04a2_88
    - action: idle
      duration: 10
```

## Common Recipes

### Make one character sit on a chair

```yaml
- character: Male_patient_01
  action: sit
  target:
    <<: *seat_defaults
    prim: /World/Chair/SM_Chair_01a3
```

### Make one character walk to one point

```yaml
- character: Male_patient_01
  action: move_to
  position: [6.7, 0.0, 0.0]
  yaw: -90.0
```

### Make one character patrol four points

```yaml
- character: Male_police_01
  action: patrol
  repeat: forever
  points:
    - position: [-28.4, 4.2, 0.0]
      yaw: 180.0
    - position: [-25.8, 10.1, 0.0]
      yaw: 90.0
    - position: [5.0, 10.1, 0.0]
      yaw: 0.0
    - position: [6.7, 0.0, 0.0]
      yaw: -90.0
```

### Move, wait under the staircase, then move back

```yaml
- character: Female_patient_02
  repeat: forever
  steps:
    - action: move_to
      position: [6.53459, 17.05795, 0.0]
      yaw: 90.0
    - action: wait
      seconds: 120
    - action: move_to
      position: [6.89391, 5.64066, 0.0]
      yaw: -90.0
    - action: wait
      seconds: 20
```

### Keep sitting while talking

Put the `sit` action and the `talk_with` action as separate top-level actions. The talk action will run as an overlay.

```yaml
- character: Male_patient_05
  action: sit
  target:
    <<: *seat_defaults
    prim: /World/Chair/SM_Chair_01a12

- character: Female_nurse_05
  action: talk_with
  target_character: Male_patient_05
  duration: 9999
  gesture: *talk_gesture
```

## Editing Checklist

Before running Isaac, check these points:

- Every scenario has `actions`, not old `commands` or `command` strings.
- Every top-level action has `character`.
- Every action name is in `behavior_catalog`.
- Use `wait.seconds` for pauses.
- Use `duration` only when an action should hold, loop, or overlay for a fixed time.
- For `sit`, confirm `target.prim` exists in the USD stage.
- For `talk` and `talk_with`, confirm `target_character` is a character name from the UI selector.
- For movement, keep `z` as `0.0` unless you have a specific reason to change it.

## Static Checks

From the repo root, you can do quick checks without starting Isaac:

```bash
python3 -c "import yaml; yaml.safe_load(open('assets/people_initial_commands.yaml')); print('yaml ok')"
PYTHONPYCACHEPREFIX=/tmp/codex_pycache python3 -m py_compile isaac_sim/people_control_sim.py
```

Then run the people UI:

```bash
./scripts/run_people_sim.sh
```

Useful runtime smoke tests:

- Press `GoTo` after selecting a character and entering X/Y/Yaw.
- Press `Set patrol` and confirm both police characters move.
- Press `Set Sit` and confirm chair alignment.
- Press `Set Talk` and confirm characters look at each other and gesture.
- Press `Initialize` and confirm sitting, patrols, strange routines, and talk overlays start together.

## Troubleshooting

If a character does nothing, check the character name exactly matches the UI selector.

If `move_to` does not move, the point may be off the navmesh. Try a nearby point or rebake/check the navmesh.

If a character sits on the floor, confirm `target.prim` is the chair prim and not the floor approach point.

If a character sits in the wrong direction, tune `hips_rotation` first. In this project the current default is `[90.0, 0.0, 0.0]`.

If a character sits too deep into the chair or too far forward, tune `hips_offset` in small increments.

If talk gestures interrupt a main plan, make sure the talk action is a top-level scenario action, not placed as a normal `steps` item inside the character's main movement sequence.
