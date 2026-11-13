// Check of the ScanMerger motion compensation: scans of a robot turning / driving in a 3 x 2.4 m room, merged with
// and without it; prints how far the merged points lie from the walls seen from the robot at the reference time.
// Not in CMake. Build and run (from ~/craft):
//   g++ -std=c++17 -O1 -Isrc/lidar_processor/include $(for d in /opt/ros/lyrical/include/*/; do printf -- "-I%s " $d; done) \
//     src/lidar_processor/test/merge_motion_check.cpp src/lidar_processor/src/scan_merger.cpp -o /tmp/merge_motion_check && /tmp/merge_motion_check
// Expected: with compensation 0.3-0.4 cm on average in every case (the 0.5 deg bins); without it 2-7 cm in the turns.
#include "lidar_processor/scan_merger.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
using namespace lidar_processing;
using sensor_msgs::msg::LaserScan;
struct P2 { double x, y, yaw; };
static const double W = 1.5, H = 1.2;          // 방: 가로 3 m, 세로 2.4 m, 로봇은 가운데 근처
P2 pose_at(double t, double v, double w) { t -= 10.0;    // t = 10 에서 (0.1, -0.1, 0.3 rad), 일정한 속도·회전속도
  const double y0 = 0.3; if (std::fabs(w) < 1e-9) return {0.1 + v * t * std::cos(y0), -0.1 + v * t * std::sin(y0), y0};
  return {0.1 + v / w * (std::sin(y0 + w * t) - std::sin(y0)), -0.1 - v / w * (std::cos(y0 + w * t) - std::cos(y0)), y0 + w * t};
}
double ray(double ox, double oy, double th) {
  const double c = std::cos(th), s = std::sin(th); double best = 1e9;
  if (std::fabs(c) > 1e-9) { const double r = ((c > 0 ? W : -W) - ox) / c; if (r > 0) best = std::min(best, r); }
  if (std::fabs(s) > 1e-9) { const double r = ((s > 0 ? H : -H) - oy) / s; if (r > 0) best = std::min(best, r); }
  return best;
}
LaserScan make(const Pose2D &lp, double stamp, double v, double w, const char *frame) {   // 인덱스 0 이 마지막 빔
  LaserScan s; s.header.frame_id = frame; s.header.stamp.sec = int(stamp); s.header.stamp.nanosec = uint32_t((stamp - int(stamp)) * 1e9);
  const int n = 400; s.angle_min = 0; s.angle_increment = 2 * M_PI / n; s.angle_max = 2 * M_PI; s.range_min = 0.1; s.range_max = 100;
  s.scan_time = 0.1; s.time_increment = 0.1 / (n - 1); s.ranges.resize(n);
  for (int i = 0; i < n; ++i) {
    const P2 b = pose_at(stamp - i * s.time_increment, v, w);
    const double ox = b.x + std::cos(b.yaw) * lp.x - std::sin(b.yaw) * lp.y, oy = b.y + std::sin(b.yaw) * lp.x + std::cos(b.yaw) * lp.y;
    const double a = s.angle_min + i * s.angle_increment;
    s.ranges[i] = float(ray(ox, oy, b.yaw + lp.yaw + (lp.flipped ? -a : a)));
  }
  return s;
}
double rms_to_room(const LaserScan &m, const P2 &ref, int *n_out, double *worst) {
  double sum = 0; int n = 0; *worst = 0;
  for (size_t k = 0; k < m.ranges.size(); ++k) {
    const float r = m.ranges[k]; if (!std::isfinite(r)) continue;
    const double a = m.angle_min + k * m.angle_increment, bx = r * std::cos(a), by = r * std::sin(a);
    const double wx = ref.x + std::cos(ref.yaw) * bx - std::sin(ref.yaw) * by, wy = ref.y + std::sin(ref.yaw) * bx + std::cos(ref.yaw) * by;
    const double d = std::min(std::min(std::fabs(W - wx), std::fabs(-W - wx)), std::min(std::fabs(H - wy), std::fabs(-H - wy)));
    sum += d * d; ++n; *worst = std::max(*worst, d);
  }
  *n_out = n; return std::sqrt(sum / std::max(n, 1));
}
int main() {
  MergerConfig c; c.front_fov_deg = 180; c.rear_fov_deg = 180; c.fov_near_m = 9.0; c.body_radius_m = 0.25; c.body_center_x = 0.01275; c.pose_time_offset = 0.05;
  ScanMerger merger; if (!merger.configure(c)) { std::puts("configure failed"); return 1; }
  const Pose2D fp{0.155, 0.0, M_PI, true}, rp{-0.1295, 0.0, 0.0, true};
  std::puts("상황                         뒤 스캔 시각   | 보정 없음: 벽에서 평균/최대 cm | 보정 있음: 평균/최대 cm | 점 수");
  struct Case { const char *name; double v, w, rear_dt; };
  for (const Case &k : {Case{"서 있음", 0, 0, -0.07}, Case{"직진 0.26 m/s", 0.26, 0, -0.07}, Case{"제자리 회전 0.6 rad/s", 0, 0.6, 0.0}, Case{"제자리 회전 0.6 rad/s", 0, 0.6, -0.1},
                        Case{"제자리 회전 0.6 rad/s", 0, 0.6, -0.2}, Case{"제자리 회전 -0.6 rad/s", 0, -0.6, 0.15}, Case{"곡선 0.2 m/s, 0.5 rad/s", 0.2, 0.5, -0.12}}) {
    const double ts = 10.0; const LaserScan f = make(fp, ts, k.v, k.w, "laser_front"), r = make(rp, ts + k.rear_dt, k.v, k.w, "laser_rear");
    const Motion m{k.v, k.w}; int n0, n1; double w0, w1;
    const auto plain = merger.merge(f, fp, &r, &rp), comp = merger.merge(f, fp, &r, &rp, &m);
    const double e0 = rms_to_room(*plain, pose_at(ts, k.v, k.w), &n0, &w0);                       // 보정 없음: 스캔 시각의 로봇 기준으로 평가
    const double e1 = rms_to_room(*comp, pose_at(ts - c.pose_time_offset, k.v, k.w), &n1, &w1);   // 보정 있음: 기준 시각(스캔 시각 - 0.05초)
    std::printf("%-26s %+5.2f s  |        %5.2f / %5.2f          |      %5.2f / %5.2f      | %d, %d\n", k.name, k.rear_dt, e0 * 100, w0 * 100, e1 * 100, w1 * 100, n0, n1);
  }
}
