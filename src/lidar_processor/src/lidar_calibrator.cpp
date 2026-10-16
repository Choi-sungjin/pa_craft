// lidar_calibrator: 기준 물체(퍽)로 전/후방 LiDAR 상대 위치를 한 번 보정해 TF 값을 출력 (실행 후 자동 종료)
//
// 사용: lidar_processor.launch.py 실행 중, 로봇 정지
//   로봇 주변 roi_max(기본 0.8 m) 안에는 퍽만 두고 나머지 물건은 치움
//   퍽 2개 이상(권장 3개)을 두 LiDAR 가 함께 보는 로봇 양옆에, 서로 다른 앞뒤 위치로 세움
//   ros2 run lidar_processor lidar_calibrator --ros-args -p puck_radius:=0.035
//
// 1) /lidar/front/scan, /lidar/rear/scan 원본 스캔을 samples 개씩 모아 1cm 격자 평균
// 2) 현재 TF 로 base_link 에 옮겨 roi_min~roi_max 고리 안의 점만 남김 (몸체 제외)
// 3) 점 덩어리(클러스터) 중 퍽 크기인 것만 골라 중심 계산
//    LiDAR 는 퍽의 앞면 반원만 보므로, 점 평균을 LiDAR 반대 방향으로 0.637*반지름 만큼 보정
// 4) 같은 퍽을 두 LiDAR 에서 짝지어, 두 중심이 겹치도록 후방 LiDAR 위치·각도를 최소제곱으로 계산
//    (잔차가 outlier_mm 보다 크면, 빼서 가장 잘 맞아지는 퍽을 하나씩 제외, 최소 3쌍)
// 5) 전방 LiDAR 방향을 기준으로, 두 LiDAR 가운데를 base_link 로 다시 잡아 출력
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using sensor_msgs::msg::LaserScan;

struct Pt { double x, y; };
struct Pose2D { double x{0}, y{0}, yaw{0}; };

static Pt apply(const Pose2D & p, const Pt & q)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y};
}
static Pose2D compose(const Pose2D & a, const Pose2D & b)   // a * b
{
  const Pt t = apply(a, {b.x, b.y});
  return {t.x, t.y, std::remainder(a.yaw + b.yaw, 2.0 * M_PI)};
}
static Pose2D inverse(const Pose2D & p)
{
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  return {-(c * p.x + s * p.y), -(-s * p.x + c * p.y), -p.yaw};
}

// 점 쌍 (src -> dst) 을 가장 잘 맞추는 2D 회전+이동
static Pose2D fit_rigid(const std::vector<Pt> & src, const std::vector<Pt> & dst)
{
  const size_t n = src.size();
  double sx = 0, sy = 0, tx = 0, ty = 0;
  for (size_t i = 0; i < n; ++i) {sx += src[i].x; sy += src[i].y; tx += dst[i].x; ty += dst[i].y;}
  sx /= n; sy /= n; tx /= n; ty /= n;
  double num = 0, den = 0;
  for (size_t i = 0; i < n; ++i) {
    const double ax = src[i].x - sx, ay = src[i].y - sy, bx = dst[i].x - tx, by = dst[i].y - ty;
    num += ax * by - ay * bx; den += ax * bx + ay * by;
  }
  const double th = std::atan2(num, den);
  return {tx - (std::cos(th) * sx - std::sin(th) * sy), ty - (std::sin(th) * sx + std::cos(th) * sy), th};
}

struct Target { Pt center_lidar; Pt center_base; size_t n; double size; };

// LiDAR 좌표 점(lp)에서 퍽 후보 찾기 (P: 현재 TF, base_link 좌표 계산용)
static std::vector<Target> detect_targets(
  const std::vector<Pt> & lp, const Pose2D & P, double puck_radius, double cluster_gap)
{
    // LiDAR 기준 각도 순으로 정렬 → 인접 점 간격이 cluster_gap 이하이면 같은 덩어리
    std::vector<size_t> idx(lp.size());
    for (size_t i = 0; i < idx.size(); ++i) {idx[i] = i;}
    std::sort(idx.begin(), idx.end(), [&](size_t a, size_t b) {
      return std::atan2(lp[a].y, lp[a].x) < std::atan2(lp[b].y, lp[b].x);});
    std::vector<std::vector<size_t>> clusters;
    for (size_t k = 0; k < idx.size(); ++k) {
      const size_t i = idx[k];
      if (clusters.empty() ||
        std::hypot(lp[i].x - lp[clusters.back().back()].x, lp[i].y - lp[clusters.back().back()].y) > cluster_gap)
      {
        clusters.push_back({});
      }
      clusters.back().push_back(i);
    }
    // 첫 덩어리와 마지막 덩어리가 각도 경계(±180도)에서 이어지면 합침
    if (clusters.size() > 1) {
      const auto & f = lp[clusters.front().front()];
      const auto & l = lp[clusters.back().back()];
      if (std::hypot(f.x - l.x, f.y - l.y) <= cluster_gap) {
        clusters.front().insert(clusters.front().end(), clusters.back().begin(), clusters.back().end());
        clusters.pop_back();
      }
    }

    std::vector<Target> out;
    for (const auto & c : clusters) {
      double mx = 0, my = 0;
      for (size_t i : c) {mx += lp[i].x; my += lp[i].y;}
      mx /= c.size(); my /= c.size();
      double size = 0;
      for (size_t i : c) {
        for (size_t j : c) {size = std::max(size, std::hypot(lp[i].x - lp[j].x, lp[i].y - lp[j].y));}
      }
      if (c.size() < 3 || size > 2.5 * puck_radius + 0.02) {continue;}   // 퍽보다 크면 제외
      // 보이는 반원 평균 → 원 중심: LiDAR 에서 먼 쪽으로 (2/pi)*R
      const double r = std::hypot(mx, my);
      const double k = 2.0 / M_PI * puck_radius;
      const Pt cl{mx + mx / r * k, my + my / r * k};
      out.push_back({cl, apply(P, cl), c.size(), size});
    }
  return out;
}

class LidarCalibrator : public rclcpp::Node
{
public:
  LidarCalibrator()
  : Node("lidar_calibrator")
  {
    samples_ = declare_parameter<int>("samples", 30);          // LiDAR 당 스캔 수 (약 3초)
    puck_radius_ = declare_parameter<double>("puck_radius", 0.035);
    roi_min_ = declare_parameter<double>("roi_min", 0.25);     // 로봇 중앙에서 이보다 가까우면 몸체로 보고 제외
    roi_max_ = declare_parameter<double>("roi_max", 0.80);     // 이보다 멀면 제외 (벽 등)
    cluster_gap_ = declare_parameter<double>("cluster_gap", 0.03);
    match_dist_ = declare_parameter<double>("match_dist", 0.15);  // 두 LiDAR 의 같은 퍽 짝짓기 허용 거리
    declare_parameter<double>("outlier_mm", 15.0);              // 이보다 잔차가 큰 퍽은 제외

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    front_sub_ = create_subscription<LaserScan>("/lidar/front/scan", rclcpp::SensorDataQoS(),
      [this](LaserScan::SharedPtr m) {collect(*m, front_grid_, front_n_, front_frame_);});
    rear_sub_ = create_subscription<LaserScan>("/lidar/rear/scan", rclcpp::SensorDataQoS(),
      [this](LaserScan::SharedPtr m) {collect(*m, rear_grid_, rear_n_, rear_frame_);});
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {check_done();});
    RCLCPP_INFO(get_logger(), "로봇을 움직이지 마세요. 스캔 %d개씩 수집 중... (퍽 반지름 %.3f m)",
      samples_, puck_radius_);
  }

private:
  using Cells = std::map<std::pair<long, long>, std::pair<Pt, int>>;   // 1cm 칸: (합, 개수)

  void collect(const LaserScan & s, Cells & grid, int & n, std::string & frame)
  {
    if (n >= samples_) {return;}
    frame = s.header.frame_id;
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const double r = s.ranges[i];
      if (!std::isfinite(r) || r < s.range_min || r > roi_max_ + 0.4) {continue;}
      const double a = s.angle_min + i * s.angle_increment;
      const Pt p{r * std::cos(a), r * std::sin(a)};
      auto & c = grid[{static_cast<long>(std::floor(p.x / 0.01)), static_cast<long>(std::floor(p.y / 0.01))}];
      c.first.x += p.x; c.first.y += p.y; c.second += 1;
    }
    ++n;
  }

  bool lookup(const std::string & frame, Pose2D & p)
  {
    try {
      auto t = tf_buffer_->lookupTransform("base_link", frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      p = {t.transform.translation.x, t.transform.translation.y,
        std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "TF 대기 중 (%s): %s", frame.c_str(), e.what());
      return false;
    }
  }

  // 한 LiDAR 의 점에서 퍽 후보 찾기
  std::vector<Target> find_targets(const Cells & grid, const Pose2D & P, const char * name)
  {
    const int min_hits = std::max(3, samples_ / 10);   // 3번 이상 찍힌 칸만 (튀는 점 제거)
    std::vector<Pt> lp;                                // LiDAR 좌표
    for (const auto & kv : grid) {
      if (kv.second.second < min_hits) {continue;}
      const Pt l{kv.second.first.x / kv.second.second, kv.second.first.y / kv.second.second};
      const Pt b = apply(P, l);
      const double d = std::hypot(b.x, b.y);
      if (d < roi_min_ || d > roi_max_) {continue;}
      lp.push_back(l);
    }
    auto out = detect_targets(lp, P, puck_radius_, cluster_gap_);
    for (const auto & t : out) {
      RCLCPP_INFO(get_logger(), "  %s 퍽 후보: base(x=%+.3f, y=%+.3f) 점 %zu개 크기 %.0f mm",
        name, t.center_base.x, t.center_base.y, t.n, t.size * 1000);
    }
    return out;
  }

  void check_done()
  {
    if (done_ || front_n_ < samples_ || rear_n_ < samples_) {return;}
    Pose2D Pf, Pr;
    if (!lookup(front_frame_, Pf) || !lookup(rear_frame_, Pr)) {return;}
    done_ = true;

    const auto F = find_targets(front_grid_, Pf, "front");
    const auto R = find_targets(rear_grid_, Pr, "rear ");

    // 짝짓기: 현재 TF 기준 base_link 에서 가장 가까운 것끼리 (서로 유일하게)
    std::vector<Pt> src, dst;   // src: 후방 LiDAR 좌표, dst: 전방 LiDAR 좌표
    std::vector<bool> used(R.size(), false);
    double before = 0;
    for (const auto & f : F) {
      long best = -1; double bd = match_dist_;
      for (size_t j = 0; j < R.size(); ++j) {
        if (used[j]) {continue;}
        const double d = std::hypot(f.center_base.x - R[j].center_base.x, f.center_base.y - R[j].center_base.y);
        if (d < bd) {bd = d; best = static_cast<long>(j);}
      }
      if (best < 0) {continue;}
      used[best] = true;
      dst.push_back(f.center_lidar); src.push_back(R[best].center_lidar);
      before += bd * bd;
    }
    RCLCPP_INFO(get_logger(), "두 LiDAR 가 함께 찾은 퍽: %zu개", src.size());
    if (src.size() < 2) {
      RCLCPP_ERROR(get_logger(),
        "퍽이 2개 이상 짝지어져야 합니다. roi_max(%.2f m) 안에 퍽만 두고, 로봇 양옆에 서로 다른 앞뒤 위치로 세우세요.",
        roi_max_);
      rclcpp::shutdown(); return;
    }

    // 후방 LiDAR → 전방 LiDAR 변환 계산
    // 잔차가 outlier_mm 보다 큰 짝이 있으면 빼고 다시 계산 (가려짐·잘못된 짝 제거, 최소 3쌍 유지)
    const double outlier = get_parameter("outlier_mm").as_double() / 1000.0;
    auto rms_of = [](const Pose2D & T, const std::vector<Pt> & s, const std::vector<Pt> & d) {
        double sum = 0, mx = 0;
        for (size_t i = 0; i < s.size(); ++i) {
          const Pt p = apply(T, s[i]);
          const double e = std::hypot(p.x - d[i].x, p.y - d[i].y);
          sum += e * e; mx = std::max(mx, e);
        }
        return std::make_pair(std::sqrt(sum / s.size()), mx);
      };
    Pose2D T = fit_rigid(src, dst);
    // 가장 큰 잔차가 기준보다 크면, 하나씩 빼 보며 남은 퍽의 RMS 가 가장 작아지는 퍽을 제외
    while (src.size() > 3 && rms_of(T, src, dst).second > outlier) {
      size_t drop = 0; double best = 1e9;
      for (size_t k = 0; k < src.size(); ++k) {
        auto s2 = src, d2 = dst;
        s2.erase(s2.begin() + k); d2.erase(d2.begin() + k);
        const double r = rms_of(fit_rigid(s2, d2), s2, d2).first;
        if (r < best) {best = r; drop = k;}
      }
      const Pt wb = apply(Pf, dst[drop]);
      RCLCPP_WARN(get_logger(), "  퍽(base x=%+.3f, y=%+.3f) 제외 후 다시 계산 (남은 퍽 RMS %.1f mm)",
        wb.x, wb.y, best * 1000);
      src.erase(src.begin() + drop); dst.erase(dst.begin() + drop);
      T = fit_rigid(src, dst);
    }
    double after = 0;
    for (size_t i = 0; i < src.size(); ++i) {
      const Pt p = apply(T, src[i]);
      const double e = std::hypot(p.x - dst[i].x, p.y - dst[i].y);
      after += e * e;
      RCLCPP_INFO(get_logger(), "  퍽 %zu 잔차: %.1f mm", i + 1, e * 1000);
    }
    before = std::sqrt(before / src.size()); after = std::sqrt(after / src.size());

    // 결과: 전방 방향은 그대로, 후방 = 전방 * T, 두 LiDAR 가운데를 base_link 로
    Pose2D Rn = compose(Pf, T);
    Pose2D Fn = Pf;
    const double mx = (Fn.x + Rn.x) / 2.0, my = (Fn.y + Rn.y) / 2.0;
    Fn.x -= mx; Fn.y -= my; Rn.x -= mx; Rn.y -= my;
    const double deg = 180.0 / M_PI;

    RCLCPP_INFO(get_logger(), "보정 전: 같은 퍽 위치 차이 RMS %.1f mm (짝지은 퍽 전체)", before * 1000);
    RCLCPP_INFO(get_logger(), "보정 후: 같은 퍽 위치 차이 RMS %.1f mm (사용한 퍽 %zu개)", after * 1000, src.size());
    RCLCPP_INFO(get_logger(), "LiDAR 중심 간 거리: %.3f m (기존 TF %.3f m)",
      std::hypot(Fn.x - Rn.x, Fn.y - Rn.y), std::hypot(Pf.x - Pr.x, Pf.y - Pr.y));
    RCLCPP_INFO(get_logger(),
      "\n==== launch 인자 (lidar_processor.launch.py 기본값에 반영) ====\n"
      "front_x:=%.4f front_y:=%.4f front_yaw_deg:=%.2f rear_x:=%.4f rear_y:=%.4f rear_yaw_deg:=%.2f",
      Fn.x, Fn.y, Fn.yaw * deg, Rn.x, Rn.y, Rn.yaw * deg);
    if (src.size() == 2) {
      RCLCPP_WARN(get_logger(), "퍽 2개는 잔차가 항상 작게 나옵니다. 오차 확인을 위해 3개 이상을 권장합니다.");
    }
    if (after > outlier) {
      RCLCPP_WARN(get_logger(), "보정 후 잔차가 큽니다. 퍽 배치를 바꿔 다시 실행하세요.");
    }
    rclcpp::shutdown();
  }

  int samples_;
  double puck_radius_, roi_min_, roi_max_, cluster_gap_, match_dist_;
  Cells front_grid_, rear_grid_;
  int front_n_{0}, rear_n_{0};
  std::string front_frame_, rear_frame_;
  bool done_{false};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<LaserScan>::SharedPtr front_sub_, rear_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarCalibrator>());
  return 0;
}
