// lidar_processor: 전/후방 LiDAR -> 로봇 중앙의 가상 360도 LiDAR 스캔 (/lidar/merged)
//
// sub: /lidar/front/scan, /lidar/rear/scan   (coin_d4_driver, BEST_EFFORT)
// pub: /lidar/merged                          (frame base_link, RELIABLE)
//
// 1) 각 LiDAR 의 FOV(로봇 바깥 방향 기준 ±fov/2) 밖이면서 몸체 중앙에서 fov_near_m 이내인 점은 버림 → 몸체 기둥 제거, 옆쪽 먼 점은 유지
//    FOV 중심은 TF 로 자동 계산하므로 LiDAR 를 어느 방향으로 달아도 같은 FOV 값을 씀
// 2) 남은 점을 base_link(로봇 중앙) 좌표로 옮겨 한 점 집합으로 합침
// 3) 로봇 중앙에서 본 각도 칸(bin)마다 가장 가까운 거리만 남김
//    → ranges[k] = 로봇 중앙에서 장애물까지 거리
#include "lidar_processor/scan_merger.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using sensor_msgs::msg::LaserScan;

struct CachedPose { lidar_processing::Pose2D pose; bool ok{false}; };  // static TF, looked up once

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
    // base_link 는 실제 회전 중심(앞뒤 바퀴 축 가운데). 몸체 중앙(두 LiDAR 가운데)의 base_link x [m]
    declare_parameter<double>("body_center_x", 0.0);

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
  bool get_pose(const std::string & frame, CachedPose & p)
  {
    if (p.ok) {return true;}
    try {
      auto t = tf_buffer_->lookupTransform(target_frame_, frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      p.pose = lidar_processing::pose_from_transform(
        t.transform.translation.x, t.transform.translation.y, q.x, q.y, q.z, q.w);
      p.ok = true;
      RCLCPP_INFO(get_logger(), "TF %s: x=%.3f y=%.3f yaw=%.1fdeg%s",
        frame.c_str(), p.pose.x, p.pose.y, p.pose.yaw * 180.0 / M_PI, p.pose.flipped ? " (upside down)" : "");
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 대기 중 (%s): %s",
        frame.c_str(), e.what());
    }
    return p.ok;
  }

  void on_lidar_front_scan(const LaserScan::SharedPtr front)
  {
    if (!get_pose(front->header.frame_id, front_pose_)) {return;}

    lidar_processing::MergerConfig config;
    config.target_frame = target_frame_;
    config.bins = bins_;
    config.range_max = get_parameter("range_max").as_double();
    config.max_age = get_parameter("max_age").as_double();
    config.front_fov_deg = get_parameter("front_fov_deg").as_double();
    config.rear_fov_deg = get_parameter("rear_fov_deg").as_double();
    config.fov_near_m = get_parameter("fov_near_m").as_double();
    config.body_radius_m = get_parameter("body_radius_m").as_double();
    config.body_center_x = get_parameter("body_center_x").as_double();
    if (!merger_.configure(config)) {
      RCLCPP_ERROR(get_logger(), "Invalid lidar merger parameters");
      return;
    }
    const lidar_processing::Pose2D fp = front_pose_.pose;
    lidar_processing::Pose2D rp;
    const LaserScan *rear = nullptr;
    if (rear_ && get_pose(rear_->header.frame_id, rear_pose_)) {
      rp = rear_pose_.pose;
      const double age = std::fabs((rclcpp::Time(front->header.stamp) -
                                    rclcpp::Time(rear_->header.stamp)).seconds());
      if (age <= config.max_age) rear = rear_.get();
    }
    if (!rear) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "No usable rear scan; publishing front only");
    const auto out = merger_.merge(*front, fp, rear, rear ? &rp : nullptr);
    if (out) lidar_processord_pub_->publish(*out);
  }

  lidar_processing::ScanMerger merger_;
  std::string target_frame_;
  int bins_;
  CachedPose front_pose_, rear_pose_;
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
