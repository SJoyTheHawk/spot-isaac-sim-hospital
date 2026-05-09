# Changelog

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
