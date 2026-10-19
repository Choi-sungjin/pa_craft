// obstacle_detector: merged scan -> obstacles
// sub: /lidar/merged    (LaserScan, frame base_link = 로봇 중앙 가상 360° LiDAR)
// pub: /lidar/obstacle  (PoseArray, frame = 입력 스캔 frame, 장애물 1개 = pose 1개)
//
// 1) 각도 순서대로 점을 훑으며, 이웃 점 사이 거리가 cluster_gap_m 이하이면 같은 장애물로 묶음
//    (NaN 칸은 건너뜀 — 0.5° bin 이라 LDS-03(0.9°) 점 사이에 빈 칸이 생김)
// 2) -180°/+180° 경계에 걸친 장애물은 하나로 합침
// 3) 점 수가 min_points 이상인 묶음만 장애물로 보고, 점들의 평균 위치를 pose.position 으로 발행
//    장애물이 없으면 빈 PoseArray 를 발행 (토픽 침묵과 "장애물 없음"을 구분)

#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct Pt { double x, y; };

class ObstacleDetector : public rclcpp::Node
{
public:
  ObstacleDetector()
  : Node("obstacle_detector")
  {
    declare_parameter<double>("max_range", 3.0);      // 이 거리 밖 점은 무시 [m]
    declare_parameter<double>("cluster_gap_m", 0.10); // 같은 장애물로 볼 이웃 점 간 최대 거리 [m]
    declare_parameter<int>("min_points", 3);          // 장애물로 인정할 최소 점 수 (잡음 제거)

    lidar_obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/lidar/obstacle", 10);
    lidar_merged_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar/merged", 10, [this](const sensor_msgs::msg::LaserScan & msg) {on_lidar_merged(msg);});
  }

private:
  void on_lidar_merged(const sensor_msgs::msg::LaserScan & msg)
  {
    const double max_range = get_parameter("max_range").as_double();
    const double gap = get_parameter("cluster_gap_m").as_double();
    const size_t min_pts = static_cast<size_t>(get_parameter("min_points").as_int());

    // 1) 각도 순서로 묶기
    std::vector<std::vector<Pt>> clusters;
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const float r = msg.ranges[i];
      if (!std::isfinite(r) || r < msg.range_min || r > max_range) {continue;}
      const double a = msg.angle_min + i * msg.angle_increment;
      const Pt p{r * std::cos(a), r * std::sin(a)};
      if (clusters.empty() || dist(clusters.back().back(), p) > gap) {
        clusters.push_back({});
      }
      clusters.back().push_back(p);
    }

    // 2) -180°/+180° 경계: 마지막 묶음 끝과 첫 묶음 시작이 가까우면 합침
    if (clusters.size() > 1 && dist(clusters.back().back(), clusters.front().front()) <= gap) {
      clusters.front().insert(clusters.front().begin(), clusters.back().begin(), clusters.back().end());
      clusters.pop_back();
    }

    // 3) 장애물 발행
    geometry_msgs::msg::PoseArray out;
    out.header = msg.header;  // stamp 는 스캔 시각 유지, frame 은 base_link
    for (const auto & c : clusters) {
      if (c.size() < min_pts) {continue;}
      double sx = 0.0, sy = 0.0;
      for (const auto & p : c) {sx += p.x; sy += p.y;}
      geometry_msgs::msg::Pose pose;
      pose.position.x = sx / c.size();
      pose.position.y = sy / c.size();
      pose.orientation.w = 1.0;  // 방향 정보 없음
      out.poses.push_back(pose);
    }
    lidar_obstacle_pub_->publish(out);
  }

  static double dist(const Pt & a, const Pt & b) {return std::hypot(a.x - b.x, a.y - b.y);}

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lidar_obstacle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_merged_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetector>());
  rclcpp::shutdown();
  return 0;
}
