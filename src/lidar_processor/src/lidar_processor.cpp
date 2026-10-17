// lidar_processor: 전/후방 LiDAR -> 로봇 중앙의 가상 360도 LiDAR 스캔 (/lidar/merged)
//
// sub: /lidar/front/scan, /lidar/rear/scan   (coin_d4_driver, BEST_EFFORT)
// pub: /lidar/merged                          (frame base_link, RELIABLE)
//
// 1) 각 LiDAR 의 FOV(로봇 바깥 방향 기준 ±fov/2) 밖이면서 로봇 중앙에서 fov_near_m 이내인 점은 버림 → 몸체 기둥 제거, 옆쪽 먼 점은 유지
//    FOV 중심은 TF 로 자동 계산하므로 LiDAR 를 어느 방향으로 달아도 같은 FOV 값을 씀
// 2) 남은 점을 base_link(로봇 중앙) 좌표로 옮겨 한 점 집합으로 합침
// 3) 로봇 중앙에서 본 각도 칸(bin)마다 가장 가까운 거리만 남김
//    → ranges[k] = 로봇 중앙에서 장애물까지 거리
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using sensor_msgs::msg::LaserScan;

struct Pose2D { double x{0}, y{0}, yaw{0}; bool ok{false}; };

class LidarProcessor : public rclcpp::Node
{
public:
  LidarProcessor()
  : Node("lidar_processor")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    bins_ = declare_parameter<int>("bins", 720);              // 0.5도 간격
    declare_parameter<double>("range_max", 8.0);
    declare_parameter<double>("max_age", 0.2);                // 전/후방 스캔 허용 시간차 [s]
    // 각 LiDAR 의 로봇 바깥 방향 기준 전체 FOV [deg]. 실행 중 ros2 param set 으로 바로 조정 가능
    declare_parameter<double>("front_fov_deg", 200.0);
    declare_parameter<double>("rear_fov_deg", 200.0);
    // FOV 밖이라도 로봇 중앙에서 이 거리 이상이면 남김 (기둥만 지우고 옆쪽은 보이게) [m]
    declare_parameter<double>("fov_near_m", 0.30);
    // 로봇 중앙에서 이 거리 안쪽 점은 FOV 와 상관없이 버림 (몸체·기둥) [m]
    declare_parameter<double>("body_radius_m", 0.25);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 팀 obstacle_detector 는 기본(Reliable) QoS 로 구독하므로 Reliable 로 발행
    lidar_processord_pub_ = create_publisher<LaserScan>("/lidar/merged", 10);
    lidar_front_scan_sub_ = create_subscription<LaserScan>(
      "/lidar/front/scan", rclcpp::SensorDataQoS(),
      [this](LaserScan::SharedPtr m) {on_lidar_front_scan(m);});
    lidar_rear_scan_sub_ = create_subscription<LaserScan>(
      "/lidar/rear/scan", rclcpp::SensorDataQoS(),
      [this](LaserScan::SharedPtr m) {rear_ = m;});
  }

private:
  // LiDAR → base_link 고정 변환 (static TF 라 처음 한 번만 조회)
  bool get_pose(const std::string & frame, Pose2D & p)
  {
    if (p.ok) {return true;}
    try {
      auto t = tf_buffer_->lookupTransform(target_frame_, frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      p.x = t.transform.translation.x;
      p.y = t.transform.translation.y;
      p.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      p.ok = true;
      RCLCPP_INFO(get_logger(), "TF %s: x=%.3f y=%.3f yaw=%.1fdeg",
        frame.c_str(), p.x, p.y, p.yaw * 180.0 / M_PI);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 대기 중 (%s): %s",
        frame.c_str(), e.what());
    }
    return p.ok;
  }

  // 한 LiDAR 의 점을 FOV 로 거른 뒤 base_link 기준 bin 에 넣음
  void add_scan(const LaserScan & s, const Pose2D & p, double fov_deg, LaserScan & out)
  {
    const double half_fov = fov_deg * M_PI / 360.0;
    // FOV 중심 = 로봇 중앙 → LiDAR 방향(바깥쪽)을 LiDAR 자기 좌표 각도로 바꾼 값
    const double fov_center = std::atan2(p.y, p.x) - p.yaw;
    const double range_max = out.range_max;
    const double fov_near = get_parameter("fov_near_m").as_double();
    const double body_r = get_parameter("body_radius_m").as_double();
    const double c = std::cos(p.yaw), sn = std::sin(p.yaw);
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const float r = s.ranges[i];
      if (!std::isfinite(r) || r < s.range_min || r > s.range_max) {continue;}
      const double a = s.angle_min + i * s.angle_increment;           // LiDAR 기준 각도
      const double lx = r * std::cos(a), ly = r * std::sin(a);
      const double x = p.x + c * lx - sn * ly;                           // base_link 좌표
      const double y = p.y + sn * lx + c * ly;
      const double d = std::hypot(x, y);                                 // 로봇 중앙에서 거리
      if (d > range_max || d < body_r) {continue;}                      // 너무 멀거나 몸체(기둥) 안쪽
      // FOV 밖이면서 로봇 가까이(기둥·몸체 거리)인 점만 버림 → 옆쪽 먼 장애물은 남김
      const bool out_of_fov = std::fabs(std::remainder(a - fov_center, 2.0 * M_PI)) > half_fov;
      if (out_of_fov && d < fov_near) {continue;}
      int k = static_cast<int>(std::floor((std::atan2(y, x) - out.angle_min) / out.angle_increment));
      if (k < 0) {k = 0;}
      if (k >= bins_) {k = bins_ - 1;}
      if (!(out.ranges[k] <= d)) {out.ranges[k] = static_cast<float>(d);}  // 비었거나 더 멀면 교체
    }
  }

  void on_lidar_front_scan(const LaserScan::SharedPtr front)
  {
    if (!get_pose(front->header.frame_id, front_pose_)) {return;}

    LaserScan out;
    out.header.stamp = front->header.stamp;
    out.header.frame_id = target_frame_;
    out.angle_min = -M_PI;
    out.angle_increment = 2.0 * M_PI / bins_;
    out.angle_max = out.angle_min + out.angle_increment * (bins_ - 1);
    out.scan_time = front->scan_time;
    out.time_increment = 0.0;   // 합친 스캔이라 점별 시간 보정 없음
    out.range_min = 0.0;
    out.range_max = static_cast<float>(get_parameter("range_max").as_double());
    out.ranges.assign(bins_, std::numeric_limits<float>::quiet_NaN());

    add_scan(*front, front_pose_, get_parameter("front_fov_deg").as_double(), out);

    if (rear_ && get_pose(rear_->header.frame_id, rear_pose_)) {
      const double age = std::fabs(
        (rclcpp::Time(front->header.stamp) - rclcpp::Time(rear_->header.stamp)).seconds());
      if (age <= get_parameter("max_age").as_double()) {
        add_scan(*rear_, rear_pose_, get_parameter("rear_fov_deg").as_double(), out);
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "후방 스캔이 오래됨 (%.2fs) → 전방만 발행", age);
      }
    } else if (!rear_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "후방 스캔 아직 없음 → 전방만 발행");
    }
    lidar_processord_pub_->publish(out);
  }

  std::string target_frame_;
  int bins_;
  Pose2D front_pose_, rear_pose_;
  LaserScan::SharedPtr rear_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<LaserScan>::SharedPtr lidar_processord_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr lidar_front_scan_sub_, lidar_rear_scan_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarProcessor>());
  rclcpp::shutdown();
  return 0;
}
