import math
from pathlib import Path as FilePath

import numpy as np
import pytest
from PIL import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry

from craft_planning.kant_astar_arc import ALIGN, Params, load_stl_map, parse_map, run
from craft_planning.path_planner import PathPlanner


def grid():
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.width = msg.info.height = 20
    msg.info.resolution = 0.1
    msg.info.origin.orientation.w = 1.0
    cells = np.zeros((20, 20), dtype=np.int8)
    cells[0, :] = cells[-1, :] = cells[:, 0] = cells[:, -1] = 100
    msg.data = cells.flatten().tolist()
    return msg


def goal(x=1.65, y=1.65, yaw=0.0, frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


def odom(x=0.35, y=0.35, yaw=0.0, frame='map'):
    msg = Odometry()
    pose = goal(x, y, yaw, frame)
    msg.header.frame_id = frame
    msg.pose.pose = pose.pose
    return msg


def planner(**kwargs):
    params = Params(r_robot=0.05, margin=0.01, res=0.1, block_size=0.2,
                    r_min=0.05, r_hi_global=0.2)
    result = PathPlanner(params=params, **kwargs)
    assert result.set_map(grid())
    assert result.set_odom(odom())
    assert result.set_goal(goal())
    return result


def test_generates_path_and_caches_while_robot_moves():
    result = planner()
    path = result.get_target_path()
    assert path.header.frame_id == 'map'
    assert len(path.poses) > 2, result.last_error
    assert path.poses[-1].pose.position.x == pytest.approx(1.65)
    assert path.poses[-1].pose.position.y == pytest.approx(1.65)
    assert result.last_leg is not None
    result.set_odom(odom(0.45, 0.35))
    assert result.get_target_path() is path
    result.set_goal(goal(1.65, 0.35))
    assert result.get_target_path() is not path


def test_routes_around_wall_and_checks_inflated_cells():
    result = planner()
    msg = grid()
    cells = np.asarray(msg.data).reshape(20, 20)
    cells[1:16, 10] = 100
    msg.data = cells.flatten().tolist()
    assert result.set_map(msg)
    result.set_goal(goal(1.65, 0.35))
    path = result.get_target_path()
    assert path.poses, result.last_error
    assert max(p.pose.position.y for p in path.poses) >= 1.6
    assert all(result.grid.point_free(p.pose.position.x, p.pose.position.y) for p in path.poses)


@pytest.mark.parametrize('occupancy', [100, -1])
def test_blocked_or_unknown_wall_yields_empty_path(occupancy):
    result = planner()
    msg = grid()
    cells = np.asarray(msg.data).reshape(20, 20)
    cells[:, 10] = occupancy
    msg.data = cells.flatten().tolist()
    result.set_map(msg)
    assert not result.get_target_path().poses
    assert result.last_error


def test_invalid_goal_and_frame_mismatch_do_not_return_old_path():
    result = planner()
    assert result.get_target_path().poses
    result.set_goal(goal(frame='odom'))
    assert not result.get_target_path().poses
    assert 'same frame' in result.last_error
    assert not result.set_goal(goal(float('nan'), 0.35))
    assert not result.get_target_path().poses


def test_odometry_frame_recovery_clears_cached_error():
    result = planner()
    path = result.get_target_path()
    assert path.poses
    result.set_odom(odom(frame='odom'))
    assert not result.get_target_path().poses
    assert result.last_error
    result.set_odom(odom())
    assert result.get_target_path() is path
    assert not result.last_error


def test_out_of_bounds_and_occupied_endpoints_are_rejected():
    result = planner()
    result.set_goal(goal(-1.0, 0.35))
    assert not result.get_target_path().poses
    assert 'outside' in result.last_error
    result.set_goal(goal(0.05, 0.35))
    assert not result.get_target_path().poses


def test_rotated_map_origin_transforms_path_back_to_world():
    result = planner()
    msg = grid()
    msg.info.origin.position.x = 10.0
    msg.info.origin.position.y = 20.0
    msg.info.origin.orientation.z = math.sin(math.pi / 4)
    msg.info.origin.orientation.w = math.cos(math.pi / 4)
    result.set_map(msg)
    result.set_odom(odom(9.65, 20.35, math.pi / 2))
    result.set_goal(goal(9.65, 21.65, math.pi / 2))
    path = result.get_target_path()
    assert path.poses, result.last_error
    assert path.poses[-1].pose.position.x == pytest.approx(9.65)
    assert path.poses[-1].pose.position.y == pytest.approx(21.65)


def test_cardinal_goal_heading_preserves_align_metadata():
    result = planner(goal_heading_required=True)
    result.set_goal(goal(1.65, 0.35))
    path = result.get_target_path()
    assert path.poses, result.last_error
    assert result.last_leg.waypoints[-1].mode == ALIGN
    assert result.last_leg.goal_h == 0


def test_bad_map_cancels_previous_path():
    result = planner()
    assert result.get_target_path().poses
    msg = grid()
    msg.data = []
    assert not result.set_map(msg)
    assert not result.get_target_path().poses
    msg = grid()
    msg.info.resolution = float('nan')
    assert not result.set_map(msg)


def test_ros_yaml_map_loads_and_unknown_pixels_are_blocked(tmp_path):
    pixels = np.full((20, 20), 255, dtype=np.uint8)
    pixels[0, :] = pixels[-1, :] = pixels[:, 0] = pixels[:, -1] = 0
    pixels[10, 10] = 205
    Image.fromarray(pixels).save(tmp_path / 'map.pgm')
    metadata = tmp_path / 'map.yaml'
    metadata.write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    result = planner()
    result.load_map(metadata)
    assert result.grid.occ[9, 10]
    assert result.get_target_path().poses, result.last_error


def test_upstream_block_map_api_and_ros_adapter():
    source = FilePath(__file__).resolve().parents[1] / 'maps' / 'kant' / 'map20.txt'
    rows = parse_map(source.read_text())
    reference = run(rows, start=(2, 2), goal=(17, 17), p=Params(block_size=0.25))
    assert reference.ok
    assert reference.path.shape[1] == 5
    result = PathPlanner(str(source), params=Params(block_size=0.25))
    sx, sy = result.grid.cell_center(*result.grid.block_to_cell(2, 2))
    gx, gy = result.grid.cell_center(*result.grid.block_to_cell(17, 17))
    result.set_odom(odom(sx, sy))
    result.set_goal(goal(gx, gy))
    assert result.get_target_path().poses, result.last_error


def test_invalid_parameters_fail_early():
    with pytest.raises(ValueError):
        PathPlanner(params=Params(res=0.0))
    with pytest.raises(ValueError):
        PathPlanner(params=Params(r_robot=-1.0))
    with pytest.raises(ValueError):
        PathPlanner(params=Params(r_robot=0.0, margin=0.0))


def write_maze_stl(filename):
    # Adapted from test_load_stl_map.py in the supplied ZIP: offset box and a partial wall.
    def quad(a, b):
        return [((*a, 0.0), (*b, 0.0), (*b, 0.4)),
                ((*a, 0.0), (*b, 0.4), (*a, 0.4))]

    origin = (-1.0, -0.5)
    box = [origin, (1.5, -0.5), (1.5, 1.5), (-1.0, 1.5)]
    triangles = [t for a, b in zip(box, box[1:] + box[:1]) for t in quad(a, b)]
    triangles += quad((0.22, -0.5), (0.22, 0.5))
    records = np.zeros(len(triangles), dtype=[('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
    records['v'] = triangles
    filename.write_bytes(b'\0' * 80 + len(triangles).to_bytes(4, 'little') + records.tobytes())
    return filename


def test_supplied_zip_stl_loader_preserves_world_origin_and_detours(tmp_path):
    source = write_maze_stl(tmp_path / 'maze.stl')
    params = Params()
    maze = load_stl_map(source, params, start=(-0.4, 0.0), goal=(0.9, 0.0))
    assert np.allclose(maze.origin, (-1.0, -0.5))
    assert maze.occ[:20, int(1.22 / params.res)].all()
    assert not maze.occ[int(0.5 / params.res), int(0.6 / params.res)]
    result = run(maze, p=params)
    assert result.ok
    assert result.path[:, 1].max() > 0.5
    assert abs(result.path[0, 0] + 0.4) <= params.res
    assert abs(result.path[-1, 0] - 0.9) <= params.res


def test_stl_ros_adapter_applies_world_origin_once(tmp_path):
    source = write_maze_stl(tmp_path / 'maze.stl')
    result = PathPlanner(str(source))
    assert result.origin == (-1.0, -0.5, 0.0)
    assert result.set_odom(odom(-0.4, 0.0))
    assert result.set_goal(goal(0.9, 0.0))
    path = result.get_target_path()
    assert path.poses, result.last_error
    assert path.header.frame_id == 'map'
    assert abs(path.poses[0].pose.position.x + 0.4) <= result.params.res
    assert abs(path.poses[-1].pose.position.x - 0.9) <= result.params.res
    assert abs(path.poses[-1].pose.position.y) <= result.params.res
    assert max(p.pose.position.y for p in path.poses) > 0.5


def test_stl_slice_heights_and_malformed_files_are_checked(tmp_path):
    source = write_maze_stl(tmp_path / 'maze.stl')
    with pytest.raises(ValueError, match='no wall intersections'):
        PathPlanner(str(source), stl_slice_heights=(0.5,))
    with pytest.raises(ValueError, match='nonempty'):
        PathPlanner(str(source), stl_slice_heights=())
    corrupt = tmp_path / 'corrupt.stl'
    corrupt.write_bytes(b'\0' * 80 + (99).to_bytes(4, 'little'))
    with pytest.raises(ValueError, match='triangle count'):
        PathPlanner(str(corrupt))
    corrupt.write_text('solid ascii_stl\nendsolid\n')
    with pytest.raises(ValueError, match='binary STL'):
        PathPlanner(str(corrupt))


MAZE_STL = FilePath(__file__).resolve().parents[1] / 'maps' / 'maze.stl'


@pytest.mark.skipif(not MAZE_STL.exists(), reason='maps/maze.stl is not available')
def test_packaged_maze_stl_plans_inside_each_closed_maze():
    result = PathPlanner(str(MAZE_STL), Params())
    height, width = result.grid.shape
    assert result.origin[:2] == pytest.approx((-2.217, -3.167), abs=1e-3)
    assert (width, height) == (89, 127)
    assert result.set_odom(odom(-1.9, 0.5))
    assert result.set_goal(goal(1.9, 2.85))
    path = result.get_target_path()
    assert len(path.poses) > 2, result.last_error
    assert path.poses[-1].pose.position.x == pytest.approx(1.9, abs=result.grid.res)
    assert path.poses[-1].pose.position.y == pytest.approx(2.85, abs=result.grid.res)
    # The twin mazes are separated by a wall; crossing between them must fail.
    assert result.set_goal(goal(-1.9, -2.85))
    assert not result.get_target_path().poses
    assert result.last_error
