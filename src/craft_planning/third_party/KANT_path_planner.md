# Imported planner provenance

- Repository: https://github.com/JunmyeongPark/KANT_path_planner
- Revision: `9af403b68bc4378516132201e9f215cea4518ea1`
- Author/account: JunmyeongPark
- Imported core: `astar_arc.py` through the `run()` API, now `src/kant_astar_arc.py`.
- Imported examples: `maps/map20*.txt`, now `maps/kant/`.
- Original README: `KANT_path_planner_README.md` alongside this file.
- No LICENSE file or license declaration was present at the imported revision.

Local changes omit standalone reporting, output-file and matplotlib visualization
helpers and their unused imports. The search/ESDF/corner-fillet algorithms remain
upstream code. `src/path_planner.py` is the ROS map/pose adapter added in this project.
The original `run()` API remains available for block-map mission/waypoint experiments.

## User-supplied STL variant

- Archive: `KANT_path_planner.zip`, supplied in this conversation.
- SHA-256: `ac0d2fbcb865e96a53d5b009f9c601a587daf1fe5bb4679d2c1ab5f44cf9315d`.
- Variant README: `KANT_path_planner_ZIP_README.md`.
- Ported changes: `GridMap.origin` and world/cell conversion, `load_stl_map()`
  horizontal slicing and occupancy rasterization, 0.05 m default block size.
- Local adaptations: binary STL header/count validation, non-finite/empty-slice
  checks, YAML-configured heights and path, ROS world/local adapter integration.
- The STL regression tests adapt the archive's `test_load_stl_map.py` fixture.
- Machine-specific `MAZE_STL`, standalone visualization, Git data and generated
  outputs were not imported. The archive contains no actual `maze.stl` asset.
