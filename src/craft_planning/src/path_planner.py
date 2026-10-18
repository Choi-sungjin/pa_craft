"""ROS map/pose adapter for KANT's A* and collision-checked corner fillets."""
import math
from dataclasses import fields, replace
from pathlib import Path as FilePath

import numpy as np
from scipy import ndimage
import yaml
from PIL import Image
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped

from craft_planning.kant_astar_arc import GridMap, Params, plan_leg


class PathPlanner:
    def __init__(self, map_file='', params=None, map_frame='map', occupancy_threshold=65,
                 goal_heading_required=False, stl_slice_heights=(0.1, 0.2, 0.3)):
        self.params = params or Params()
        nonnegative = {'r_robot', 'margin', 'lam', 'w_prox'}
        for field in fields(self.params):
            value = getattr(self.params, field.name)
            if not math.isfinite(value) or value < 0 or (value == 0 and field.name not in nonnegative):
                raise ValueError(f'Invalid planner parameter: {field.name}')
        if self.params.r_min > self.params.r_hi_global:
            raise ValueError('r_min must not exceed r_hi_global')
        if self.params.pad <= 0:
            raise ValueError('Robot radius plus margin must be positive')
        if not map_frame or not 1 <= occupancy_threshold <= 100:
            raise ValueError('Invalid map frame or occupancy threshold')
        self.map_file = map_file
        self.map_frame = map_frame
        self.occupancy_threshold = occupancy_threshold
        self.goal_heading_required = goal_heading_required
        self.stl_slice_heights = tuple(stl_slice_heights)
        if not self.stl_slice_heights or not all(math.isfinite(v) for v in self.stl_slice_heights):
            raise ValueError('STL slice heights must be nonempty and finite')
        self.goal = None
        self.odom = None
        self.grid = None
        self.origin = (0.0, 0.0, 0.0)
        self.last_leg = None  # preserves upstream PASS / STOP_TURN / ALIGN waypoint metadata
        self.last_error = ''
        self._path = Path()
        self._dirty = True
        self._attempted_start = None
        if map_file:
            self.load_map(map_file)

    def _invalidate(self):
        self._dirty = True
        self._path = Path()
        self.last_leg = None
        self.last_error = ''
        self._attempted_start = None

    @staticmethod
    def _yaw(pose):
        p, q = pose.position, pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            raise ValueError('Pose contains non-finite coordinates')
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if norm < 1e-9:
            raise ValueError('Pose quaternion is zero')
        x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
        return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))

    def _set_grid(self, occupied, resolution, origin, frame):
        if not frame or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('Map needs a frame and positive resolution')
        if occupied.ndim != 2 or not occupied.size or not all(math.isfinite(v) for v in origin):
            raise ValueError('Invalid map dimensions or origin')
        # Same ESDF/C-space convention as upstream build_map; outside the map is occupied.
        padded = np.pad(occupied, 1, constant_values=True)
        distance = ndimage.distance_transform_edt(~padded)[1:-1, 1:-1]
        esdf = np.maximum(distance * resolution - 0.5 * resolution, 0.0)
        self.grid = GridMap(occupied, esdf < self.params.pad, esdf, resolution,
                            (0, 0), None, resolution, (occupied.shape[1], occupied.shape[0]))
        self.origin, self.map_frame = origin, frame
        self._invalidate()

    def set_map(self, msg: OccupancyGrid):
        # Unknown cells are occupied; no implicit map/odom transform is applied.
        self.grid = None
        self._invalidate()
        try:
            if msg.info.width <= 0 or msg.info.height <= 0:
                raise ValueError('Map dimensions must be positive')
            values = np.asarray(msg.data, dtype=np.int16)
            if values.size != msg.info.width * msg.info.height or np.any((values < -1) | (values > 100)):
                raise ValueError('Invalid occupancy data')
            occupied = ((values < 0) | (values >= self.occupancy_threshold)).reshape(
                msg.info.height, msg.info.width)
            origin = msg.info.origin.position
            self._set_grid(occupied, msg.info.resolution,
                           (origin.x, origin.y, self._yaw(msg.info.origin)), msg.header.frame_id)
            return True
        except ValueError as exc:
            self.last_error = str(exc)
            return False

    def load_map(self, filename):
        """Load Gazebo binary STL, upstream block text, or ROS trinary YAML/image maps."""
        self.grid = None
        self._invalidate()
        filename = FilePath(filename).expanduser()
        if filename.suffix.lower() == '.stl':
            from craft_planning.kant_astar_arc import load_stl_map
            mesh_grid = load_stl_map(filename, self.params, start=(0.0, 0.0),
                                     heights=self.stl_slice_heights)
            # The adapter's search grid is local; apply the mesh world origin exactly once on output.
            self._set_grid(mesh_grid.occ, mesh_grid.res,
                           (*mesh_grid.origin, 0.0), self.map_frame)
        elif filename.suffix.lower() == '.txt':
            from craft_planning.kant_astar_arc import build_map, parse_map
            ratio = self.params.block_size / self.params.res
            if ratio < 1 or not math.isclose(ratio, round(ratio)):
                raise ValueError('block_size must be an integer multiple of res')
            self.grid = build_map(parse_map(filename.read_text(encoding='utf-8')),
                                  self.params, start=(0, 0), goal=(0, 0))
            self.origin = (0.0, 0.0, 0.0)
        elif filename.suffix.lower() in ('.yaml', '.yml'):
            data = yaml.safe_load(filename.read_text(encoding='utf-8'))
            if not isinstance(data, dict) or data.get('mode', 'trinary') != 'trinary':
                raise ValueError('Only ROS trinary map YAML is supported')
            origin = tuple(float(v) for v in data['origin'])
            if len(origin) != 3:
                raise ValueError('Map origin must contain x, y, yaw')
            free, occupied = float(data.get('free_thresh', 0.196)), float(data.get('occupied_thresh', 0.65))
            negate = int(data.get('negate', 0))
            if not 0 <= free < occupied <= 1 or negate not in (0, 1):
                raise ValueError('Invalid map thresholds or negate flag')
            with Image.open(filename.parent / data['image']) as image:
                pixels = np.asarray(image.convert('L'), dtype=float) / 255.0
                alpha = np.asarray(image.convert('RGBA'))[:, :, 3]
            probability = pixels if negate else 1.0 - pixels
            # Gray/transparent unknown pixels are blocked, as with OccupancyGrid unknowns.
            cells = np.flipud((probability >= free) | (alpha < 255))
            self._set_grid(cells, float(data['resolution']), origin, self.map_frame)
        else:
            raise ValueError('Use a binary .stl, .txt block map or ROS .yaml map')
        self.map_file = str(filename)

    def set_goal(self, goal: PoseStamped):
        self._invalidate()
        try:
            self._yaw(goal.pose)
            if not goal.header.frame_id:
                raise ValueError('Goal frame_id is empty')
        except ValueError as exc:
            self.goal = None
            self.last_error = str(exc)
            return False
        self.goal = goal
        return True

    def set_odom(self, odom: Odometry):
        try:
            self._yaw(odom.pose.pose)
            if not odom.header.frame_id:
                raise ValueError('Odometry frame_id is empty')
        except ValueError as exc:
            self.odom = None
            self._invalidate()
            self.last_error = str(exc)
            return False
        self.odom = odom
        return True

    def _local_pose(self, pose):
        ox, oy, yaw = self.origin
        dx, dy = pose.position.x - ox, pose.position.y - oy
        c, s = math.cos(yaw), math.sin(yaw)
        x, y = c*dx + s*dy, -s*dx + c*dy
        heading = int(round((self._yaw(pose) - yaw) / (math.pi / 2))) % 4
        return self.grid.world_to_cell(x, y), heading

    def get_target_path(self) -> Path:
        if self.grid is None or self.goal is None or self.odom is None:
            return Path()
        if self.goal.header.frame_id != self.map_frame or self.odom.header.frame_id != self.map_frame:
            self.last_error = 'Map, goal and odometry must use the same frame; transform upstream'
            return Path()
        start, start_h = self._local_pose(self.odom.pose.pose)
        if not self._dirty and (self._path.poses or self._attempted_start == (start, start_h)):
            if self._path.poses:
                self.last_error = ''
            return self._path
        self._dirty = False
        self._attempted_start = (start, start_h)
        try:
            goal, goal_h = self._local_pose(self.goal.pose)
            height, width = self.grid.shape
            if any(not (0 <= x < width and 0 <= y < height) for x, y in (start, goal)):
                raise ValueError('Start or goal is outside the map')
            params = replace(self.params, res=self.grid.res,
                             arc_sample=min(self.params.arc_sample, self.grid.res / 2))
            leg = plan_leg(self.grid, params, start, start_h, goal,
                           goal_h if self.goal_heading_required else None)
            if not np.isfinite(leg.path).all() or any(
                    not self.grid.point_free(x, y) for x, y in leg.path[:, :2]):
                raise ValueError('Planned path violates the inflated obstacle map')
            # Upstream output spacing is 2 cm. Check between samples too for finer ROS grids.
            for a, b in zip(leg.path[:-1, :2], leg.path[1:, :2]):
                steps = max(1, int(math.ceil(np.linalg.norm(b - a) / (self.grid.res / 2))))
                if any(not self.grid.point_free(x, y) for x, y in np.linspace(a, b, steps + 1)):
                    raise ValueError('Planned path segment crosses an inflated obstacle')
            result = Path()
            result.header.frame_id = self.map_frame
            ox, oy, yaw = self.origin
            c, s = math.cos(yaw), math.sin(yaw)
            for x, y, heading, _curvature, _v_limit in leg.path:
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.pose.position.x = ox + c*x - s*y
                pose.pose.position.y = oy + s*x + c*y
                pose.pose.orientation.z = math.sin((heading + yaw) / 2)
                pose.pose.orientation.w = math.cos((heading + yaw) / 2)
                result.poses.append(pose)
            self.last_leg, self._path = leg, result
            self.last_error = ''
        except (ValueError, RuntimeError) as exc:
            self._path = Path()
            self.last_leg = None
            self.last_error = str(exc)
        return self._path
