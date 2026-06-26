# Changelog

## Unreleased

### Added

- Added environment overrides for combined Spot + people Isaac startup settings.
- Added troubleshooting guidance for native RTX startup crashes before the people USD is loaded.

### Fixed

- Hardened the combined Spot + people launcher by using TAA antialiasing, disabling multi-GPU, and skipping Isaac's default empty stage creation before opening the character hospital USD.

### Removed

- Removed the deprecated Isaac Sim 5 command-string scheduler and legacy command adapter from the people-only control UI.
- Removed ragdoll fall startup settings and native `agent.fall()` calls after confirming they crash the full Isaac Sim 6 hospital scene.
- Removed deprecated runtime people command-file setup from the combined Spot + people launcher.

### Changed

- Updated README people-simulation examples to use the Isaac Sim 6 structured YAML action format and character scene path.

## 2.1.0 - 2026-06-26

### Added

- Added structured YAML people command support for Isaac Sim 6 BehaviorAgent actions.
- Added a people action registry for native actions, scheduler actions, and composite actions.
- Added per-character sequence, repeat, wait, patrol, look-around, talk, and talk-with behavior execution.
- Added sit target setup through `/Behavior/Sit` and `/Behavior/Sit_Hips`, including configurable hips offsets, hips rotation, and seat snapping.
- Added documentation and action prototypes for configuring `assets/people_initial_commands.yaml`.

### Changed

- Converted `assets/people_initial_commands.yaml` from Isaac Sim 5-style command strings to structured Isaac Sim 6 action objects.
- Updated the people-only simulation flow to use the Isaac Sim 6 character hierarchy under `/World/Characters/Behavior_Tree_Group`.
- Updated selected-character commands so `GoTo` uses the selected UI character and target X/Y fields through the structured action system.
- Updated reset and teleport direction handling to keep the official `facing` field while applying the required numeric yaw compensation safely.

### Removed

- Removed deprecated Isaac Sim 5-only paths from the people-only control UI, including legacy command-string execution and runtime command-file control.

### Fixed

- Fixed selector display names so characters are shown by their scenario prim names instead of nested `ManRoot` prim names.
- Fixed `idle` after `sit` by cancelling active action tasks and passing a safe fallback facing direction.
- Fixed timed action handoff so duration-based actions advance cleanly without immediately repeating or getting stuck.
- Fixed missing scene path updates for the Isaac Sim 6 hospital character scene.

## 2.0.1 - 2026-05-10

### Added

- Added a root `LICENSE` file with the Apache-2.0 license text.
- Added public release license wording to the README, including project copyright, owner contact, and NVIDIA Isaac Sim asset license references.
- Added README GIF media references for people simulation and Nav2/RViz captures.
- Added `FRONT_CAMERA_AS_FISHEYE` / `FRONT_CAMERA_FISHEYE_HFOV_RAD` configuration and README documentation so the front RGB camera can be configured as a fisheye-style camera.
- Added README guidance to disable extra fisheye cameras first when the simulation feels low-FPS or compute-bound.
- Added the full Nav2 navigation video link alongside the trimmed README preview GIF.

### Changed

- Renamed the public project/report title from `spot-isaac-lab-hospital` to **Spot Isaac Sim Hospital**.
- Updated clone/path examples to use `spot-isaac-sim-hospital`.
- Updated ROS package maintainer/author metadata to `Johnny Sze`, `Rocky Road Studio`, and `rockyroadstudio@outlook.com`.
- Updated `env/spot_isaac.env.template` to default `ISAAC_SIM_PATH` to `$HOME/isaac-sim` instead of a machine-specific absolute path.
- Updated documentation paths in `README.md`, `docs/MODIFY_WORLD.md`, and `scripts/dump_scene_positions.py`.

### Removed

- Removed obsolete `argparse` / `--test` parsing from the Spot Isaac Sim entrypoints.
- Removed stale duplicated constants, unused helper functions, and unused local variables from `spot_standalone.py` and `spot_bridge_with_people.py`.
- Removed compatibility environment writes for `PEOPLE_TEST_USD` and `HOSPITAL_USD` from the combined Spot + people bridge.

### Fixed

- Avoided RealSense camera prim discovery when `ENABLE_REALSENSE = False`.
- Avoided side/back fisheye TF setup when `ENABLE_FISHEYE_CAMERAS = False`.
- Replaced stale README media links that pointed at removed MP4 files with current GIF paths.

## 2.0.0 - 2026-05-09

### Added

- Added `scripts/run_spot_bridge_with_people.sh` for one-process Spot + people simulation.
- Added `isaac_sim/spot_bridge_with_people.py`, a self-contained combined Isaac Sim entrypoint.
- Added people-first startup for the combined bridge: load the character USD, initialize people behavior scripts, then enable the ROS 2 bridge and Spot sensor graphs.
- Added automatic creation of an empty people command scratch file under `/tmp/spot_isaac_people_runtime_commands.txt`.
- Added README media for people scenario demos, RViz2 costmap, RealSense point cloud, and camera streams.

### Changed

- Updated documentation to make the combined Spot + people bridge the recommended v2.0 workflow.
- Updated `run_people_sim.sh` and the people scripts to use `/tmp/spot_isaac_people_runtime_commands.txt` instead of an assets-folder placeholder.
- Bumped the ROS bringup package version to `2.0.0`.

### Fixed

- Fixed people behavior initialization in the combined bridge by ignoring stale non-`SkelRoot` behavior script instances.
- Fixed RealSense color/depth frame IDs to use discovered USD camera prim frames.
- Fixed RealSense depth point cloud TF alignment by publishing camera TFs for the discovered RealSense prims in both `spot_bridge_with_people.py` and `spot_standalone.py`.
- Fixed the missing `assets/people_runtime_commands.txt` startup error by removing that file from the default workflow.

### Notes

- `run_isaac.sh` remains the Spot-only runtime.
- `run_people_sim.sh` remains the people-only scenario UI.
- `run_spot_bridge_with_people.sh` is the recommended mode for robot + animated people in one simulation.
