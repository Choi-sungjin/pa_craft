# Path planning integration

The planning core comes from [JunmyeongPark/KANT_path_planner](https://github.com/JunmyeongPark/KANT_path_planner),
revision `9af403b68bc4378516132201e9f215cea4518ea1`.
The STL/world-origin input changes are adapted from the user-supplied
`KANT_path_planner.zip`. Source provenance and both READMEs are installed under
`third_party/`.

```text
scripts/planning_master.py     ROS subscriptions, parameters and publishers
src/path_planner.py            ROS map/pose <-> KANT grid/path adapter
src/kant_astar_arc.py          imported A*, ESDF, obstacle inflation, corner fillets
config/planning.yaml          planner geometry and cost settings
maps/maze.stl                  default Gazebo maze mesh
maps/kant/*.txt                upstream example block maps
```

The adapter preserves `set_goal()`, `set_odom()` and `get_target_path()` calls.
It caches the path while odometry advances. A new goal or map invalidates it;
a failed plan is retried when the starting cell/heading changes. Planning runs
in the planning node, separately from the control timer process.

## ROS interfaces

| Topic | Type | Direction |
| --- | --- | --- |
| `/perception/state/odom` | `nav_msgs/Odometry` | Current robot pose input |
| `/planning/target_base_pose` | `geometry_msgs/PoseStamped` | New mobile-base goal input |
| `/planning/target_path` | `nav_msgs/Path` | Collision-checked sampled path output |
| `/planning/odom` | `nav_msgs/Odometry` | Forwarded odometry for control |

The existing TCP goal and IK bridge are unchanged. Dynamic lidar obstacle
updates are still a TODO; this integration uses static occupancy maps.

Map, goal and odometry **must use the same frame**. The adapter applies the map
origin's translation and yaw when converting cells to world coordinates, but
does not silently relabel frames or run TF transformations. If the estimator
publishes in `odom` while the map is in `map`, transform the pose upstream first.

## Maps and settings

The supplied ZIP's main map input is a **binary STL in metres, Z-up**, whose XY
vertices already use Gazebo world coordinates. Set `map_file` to that `.stl`.
`stl_slice_heights: [0.10, 0.20, 0.30]` selects horizontal wall intersections;
their union is sampled at resolution/4 and rasterized at `path_planner.res`
(default 0.05 m). ESDF and robot-footprint inflation are applied afterwards.

The mesh's minimum intersected XY coordinates define the grid origin. Negative
world coordinates are preserved: planning uses local grid coordinates internally
and converts the output back to the same world frame exactly once. The core
`load_stl_map(path, params, start=(x,y), goal=(x,y), targets=[(x,y,'d')])` and
`run(grid)` APIs also preserve world coordinates, matching the supplied code.
`map_frame` must match the estimator/goal frame. STL carries no ROS frame ID,
resolution, SDF model pose or scale; apply any mesh-to-world transform before
supplying the file. ASCII STL is rejected explicitly.

The default map is `maps/maze.stl` ("Twin Closed Maze B", 580k triangles,
XY extent about 4.46 x 6.36 m, walls 0.39 m high). It rasterizes to an 89 x 127
grid with origin `(-2.217, -3.167)` m, i.e. two closed mazes separated by a wall;
a goal in the other maze fails as disconnected. Relative `map_file` paths resolve
against the installed `craft_planning` share directory. Use another mesh with:

```bash
ros2 run craft_planning planning_master --ros-args \
  --params-file install/craft_planning/share/craft_planning/config/planning.yaml \
  -p map_file:=/absolute/path/to/other.stl
```

The planning map is fixed to `map_file` and is required; the node does not
subscribe to `/map`. SLAM is used for localization only. The STL is in Gazebo
world coordinates, so odometry and goals must be expressed in that frame
(labelled `map_frame`); SLAM's own map origin is the robot start pose.

`map_file` may also be an upstream `.txt` block map, or a ROS
trinary `.yaml` map referencing an image. A raw image alone lacks resolution and
origin metadata and is rejected. YAML images use the configured thresholds and
negate flag. Unknown occupancy, unknown gray pixels and transparent pixels are
blocked. Outside the map is also blocked.

ROS YAML maps supply their actual resolution; STL uses
`path_planner.res` and `.txt` maps additionally use `path_planner.block_size`.
Targets and starts are snapped
to grid cell centers. Footprint inflation uses `r_robot + margin`; measure the
circumscribed radius for the actual robot and carried load before operation.

All upstream `Params` fields are declared as `path_planner.*` ROS parameters in
`config/planning.yaml`. `a_lon_max` is only an estimated stop-time cost, not a
host motor acceleration ramp; DYNAMIXEL profiles remain firmware-owned.

```bash
colcon build --packages-select craft_planning
source install/setup.bash
ros2 run craft_planning planning_master --ros-args \
  --params-file install/craft_planning/share/craft_planning/config/planning.yaml
```

To try the original text map, pass `-p map_file:=/absolute/path/to/map20.txt`
and `-p path_planner.block_size:=0.25` (the ZIP defaults to 0.05).
In that 20x20 map, block `(2, 2)` has world center `(0.625, 4.375)` m and block
`(17, 17)` has center `(4.375, 0.625)` m. File loading does not replace the live
odometry start or invent a goal from the example. Publish a goal explicitly:

```bash
ros2 topic pub --once /planning/target_base_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 4.375, y: 0.625}, orientation: {w: 1.0}}}'
```

Bringup loads `planning.yaml` with launch argument `map_file` (default
`maps/maze.stl`).

## Path versus execution

The core retains `(x, y, yaw, curvature, v_limit)` samples and
`PASS` / `STOP_TURN` / `ALIGN` waypoint modes in `PathPlanner.last_leg`.
The original multi-target `run()` API remains available directly from
`craft_planning.kant_astar_arc`; the ROS adapter plans one explicit goal at a time.

`nav_msgs/Path` carries only poses. Current control uses the sampled positions
with Pure Pursuit and does not consume the upstream curvature, speed limits, or
waypoint modes. It cannot execute required in-place turns or ALIGN actions yet.
The mission/motion FSM must handle those actions and issue a suitable
`/planning/target_v` before complete automatic driving is available. This change
does not publish a nonzero target speed or pretend these modes are executed.

By default `goal_heading_required=false`. When enabled, goal yaw is quantized
to the closest cardinal direction, and KANT preserves the straight approach
segment and ALIGN metadata. It does not solve arbitrary final orientations.

Failures (blocked/out-of-bounds endpoints, disconnected maps, frame mismatch)
publish an empty path and report the reason; previous geometry is not reused as
a successful result for a failed new goal.

## Verification

```bash
colcon build --packages-select craft_planning --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select craft_planning --event-handlers console_direct+
```

STL tests follow the synthetic offset-origin box and partition-wall example in
the supplied ZIP. They check wall placement, world-coordinate output, obstacle
detours, slice settings, and malformed inputs. A separate test plans inside the
packaged `maps/maze.stl` and checks that crossing between the twin mazes fails.
