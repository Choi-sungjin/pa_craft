"""path_planner: called by planning_master (set_goal(), set_odom(), get_target_path())."""
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class PathPlanner:
    def __init__(self, map_file=''):
        self.map_file = map_file  # map.pgm
        self.goal = None
        self.odom = None

    def set_goal(self, goal: PoseStamped):
        self.goal = goal

    def set_odom(self, odom: Odometry):
        self.odom = odom

    def get_target_path(self) -> Path:
        # TODO: plan on map.pgm from odom to goal
        return Path()
