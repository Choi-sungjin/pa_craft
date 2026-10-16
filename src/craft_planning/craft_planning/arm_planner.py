"""arm_planner: called by planning_master (set_arm_pose(), get_arm_pose()), sub /arm/odom."""
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class ArmPlanner:
    def __init__(self, node):
        self.arm_pose = PoseStamped()
        self.arm_odom = None
        node.create_subscription(JointState, '/arm/odom', self._on_arm_odom, 10)

    def _on_arm_odom(self, msg: JointState):
        self.arm_odom = msg

    def set_arm_pose(self, pose: PoseStamped):
        self.arm_pose = pose

    def get_arm_pose(self) -> PoseStamped:
        # TODO: arm trajectory using self.arm_odom
        return self.arm_pose
