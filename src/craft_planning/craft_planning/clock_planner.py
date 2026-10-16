"""clock_planner: called by planning_master (set_clock_pose(), get_clock_pose())."""


class ClockPlanner:
    def __init__(self):
        self.clock_pose = 0.0

    def set_clock_pose(self, pose: float):
        self.clock_pose = pose

    def get_clock_pose(self) -> float:
        # TODO: clock target from mission time
        return self.clock_pose
