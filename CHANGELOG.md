# Changelog

## Unreleased

### Added

- Added environment overrides for combined Spot + people Isaac startup settings.
- Added troubleshooting guidance for native RTX startup crashes before the people USD is loaded.

### Fixed

- Hardened the combined Spot + people launcher by using TAA antialiasing, disabling multi-GPU, and skipping Isaac's default empty stage creation before opening the character hospital USD.

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
