"""Planning master and Cartesian-arm IK bridge.

Arm input: /planning/target_tcp_pose (PoseStamped, base_link frame)
IK input/output: /ik_target, /ik_solution (provided by v75_arm_ik)
Arm output: /planning/target_arm_joints (JointState)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from craft_planning.path_planner import PathPlanner

IK_OUTPUT_JOINT_ORDER = (
    'slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint'
)


class PlanningMaster(Node):
    def __init__(self):
        super().__init__('planning_master')
        map_file = self.declare_parameter('map_file', '').value

        self.path_planner = PathPlanner(map_file)

        self.mission = ''
        self.odom = None

        self.target_path_pub = self.create_publisher(Path, '/planning/target_path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/planning/odom', 10)
        self.ik_target_pub = self.create_publisher(PoseStamped, '/ik_target', 10)
        self.arm_joint_target_pub = self.create_publisher(JointState, '/planning/target_arm_joints', 10)

        self.create_subscription(String, '/planning/mission', self.on_mission, 10)
        self.create_subscription(Odometry, '/state/odom', self.on_odom, 10)
        self.create_subscription(PoseArray, '/lidar/obstacle', self.on_obstacle, 10)
        self.create_subscription(PoseStamped, '/vision/object_pose', self.on_object_pose, 10)
        self.create_subscription(PoseArray, '/vision/object_detection', self.on_object_detection, 10)
        self.create_subscription(PoseStamped, '/planning/target_tcp_pose', self.on_tcp_pose_goal, 10)
        self.create_subscription(JointState, '/ik_solution', self.on_ik_solution, 10)

        self.create_timer(0.05, self.planning_loop)

    def on_mission(self, msg: String):
        self.mission = msg.data

    def on_odom(self, msg: Odometry):
        self.odom = msg
        self.path_planner.set_odom(msg)

    def on_obstacle(self, msg: PoseArray):
        pass  # TODO

    def on_object_pose(self, msg: PoseStamped):
        pass  # TODO

    def on_object_detection(self, msg: PoseArray):
        pass  # TODO: pick goal -> self.path_planner.set_goal(...)

    def on_tcp_pose_goal(self, msg: PoseStamped):
        # TODO: transform perception/planning goals into base_link before publishing.
        if msg.header.frame_id != 'base_link':
            self.get_logger().warning('ignoring TCP goal: frame_id must be base_link')
            return
        pose_values = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in pose_values):
            self.get_logger().warning('ignoring TCP goal with non-finite pose values')
            return
        quaternion_norm = math.sqrt(sum(value * value for value in pose_values[3:]))
        if quaternion_norm < 1e-9:
            self.get_logger().warning('ignoring TCP goal with zero quaternion')
            return
        self.ik_target_pub.publish(msg)

    def on_ik_solution(self, msg: JointState):
        if len(msg.name) != len(IK_OUTPUT_JOINT_ORDER) or len(msg.name) != len(msg.position):
            self.get_logger().warning('ignoring invalid /ik_solution joint names/positions')
            return
        if any(not math.isfinite(position) for position in msg.position):
            self.get_logger().warning('ignoring non-finite /ik_solution positions')
            return
        if tuple(msg.name) != IK_OUTPUT_JOINT_ORDER:
            self.get_logger().warning('ignoring /ik_solution with unexpected arm joint names')
            return

        self.arm_joint_target_pub.publish(msg)

    def planning_loop(self):
        if self.odom is None:
            return
        self.odom_pub.publish(self.odom)
        self.target_path_pub.publish(self.path_planner.get_target_path())


def main():
    rclpy.init()
    rclpy.spin(PlanningMaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
