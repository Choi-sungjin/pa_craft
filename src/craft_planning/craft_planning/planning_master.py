"""planning_master node.

sub: /planning/mission, /state/odom, /lidar/obstacle, /vision/object_pose, /vision/object_detection
pub: /planning/target_path, /planning/odom
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from craft_planning.path_planner import PathPlanner


class PlanningMaster(Node):
    def __init__(self):
        super().__init__('planning_master')
        map_file = self.declare_parameter('map_file', '').value

        self.path_planner = PathPlanner(map_file)

        self.mission = ''
        self.odom = None

        self.target_path_pub = self.create_publisher(Path, '/planning/target_path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/planning/odom', 10)

        self.create_subscription(String, '/planning/mission', self.on_mission, 10)
        self.create_subscription(Odometry, '/state/odom', self.on_odom, 10)
        self.create_subscription(PoseArray, '/lidar/obstacle', self.on_obstacle, 10)
        self.create_subscription(PoseStamped, '/vision/object_pose', self.on_object_pose, 10)
        self.create_subscription(PoseArray, '/vision/object_detection', self.on_object_detection, 10)

        self.create_timer(0.05, self.on_timer)

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

    def on_timer(self):
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
