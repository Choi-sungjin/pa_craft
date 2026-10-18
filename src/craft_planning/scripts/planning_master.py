#!/usr/bin/env python3
"""Planning master and Cartesian-arm IK bridge.

Arm input: /planning/target_tcp_pose (PoseStamped, base_link frame)
IK input/output: /ik_target, /ik_solution (provided by v75_arm_ik)
Arm output: /planning/target_arm_joints (JointState)
"""
import math
from dataclasses import fields
from pathlib import Path as FilePath

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from craft_planning.path_planner import PathPlanner
from craft_planning.kant_astar_arc import Params

IK_OUTPUT_JOINT_ORDER = (
    'slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint'
)


class PlanningMaster(Node):
    def __init__(self):
        super().__init__('planning_master')
        map_file = self.declare_parameter('map_file', 'maps/maze.stl').value
        if not map_file:
            raise ValueError('map_file is required; the planning map is fixed (maps/maze.stl)')
        if not FilePath(map_file).expanduser().is_absolute():
            # Relative paths such as maps/maze.stl resolve against the installed package share.
            map_file = str(FilePath(get_package_share_directory('craft_planning')) / map_file)
        map_frame = self.declare_parameter('map_frame', 'map').value
        occupancy_threshold = self.declare_parameter('occupancy_threshold', 65).value
        goal_heading_required = self.declare_parameter('goal_heading_required', False).value
        stl_slice_heights = self.declare_parameter('stl_slice_heights', [0.1, 0.2, 0.3]).value
        defaults = Params()
        values = {field.name: self.declare_parameter(
            'path_planner.' + field.name, getattr(defaults, field.name)).value
            for field in fields(defaults)}
        self.path_planner = PathPlanner(map_file, Params(**values), map_frame,
                                        occupancy_threshold, goal_heading_required, stl_slice_heights)
        self._last_path_error = ''

        self.mission = ''
        self.odom = None

        self.target_path_pub = self.create_publisher(Path, '/planning/target_path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/planning/odom', 10)
        self.ik_target_pub = self.create_publisher(PoseStamped, '/ik_target', 10)
        self.arm_joint_target_pub = self.create_publisher(JointState, '/planning/target_arm_joints', 10)

        self.create_subscription(String, '/planning/mission', self.on_mission, 10)
        self.create_subscription(Odometry, '/state/odom', self.on_odom, 10)
        # The planning map is fixed to map_file; SLAM is used for localization only, so /map is unused.
        self.get_logger().info('planning map loaded from ' + self.path_planner.map_file)
        self.create_subscription(PoseStamped, '/planning/target_base_pose', self.on_base_pose_goal, 10)
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

    def on_base_pose_goal(self, msg: PoseStamped):
        if not self.path_planner.set_goal(msg):
            self.get_logger().warning('ignoring invalid base goal: ' + self.path_planner.last_error)

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
        path = self.path_planner.get_target_path()
        path.header.stamp = self.get_clock().now().to_msg()
        self.target_path_pub.publish(path)
        error = self.path_planner.last_error
        if error and error != self._last_path_error:
            self.get_logger().warning('path planning failed: ' + error)
        self._last_path_error = error


def main():
    rclpy.init()
    rclpy.spin(PlanningMaster())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
