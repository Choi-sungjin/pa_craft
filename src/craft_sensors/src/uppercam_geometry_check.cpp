// Offline check of the C++ upper-cam geometry against recorded frames (no camera, no ROS).
//   uppercam_geometry_check <urdf> <frames dir> <out.jsonl>
// Frames as written by boxgap code/read_bag.py: <prefix>_depth.u16 (u16 LE, 1 mm units) + <prefix>_meta.txt
// ("w h fx fy ppx ppy depth_scale serial"). One JSON line per frame, the keys of tools py_ref (ok, error, ms, ...).
#include "uppercam_geometry.hpp"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>

int main(int argc, char ** argv)
{
  if (argc != 4) { std::cerr << "usage: uppercam_geometry_check <urdf> <frames dir> <out.jsonl>\n"; return 2; }
  const auto mount = uppercam::load_camera_mount(argv[1]);
  std::set<std::string> prefixes;
  for (const auto & e : std::filesystem::directory_iterator(argv[2])) {
    const std::string p = e.path().string();
    if (p.size() > 9 && p.compare(p.size() - 9, 9, "_meta.txt") == 0) prefixes.insert(p.substr(0, p.size() - 9));
  }
  std::ofstream out(argv[3]);
  for (const auto & p : prefixes) {
    std::ifstream meta(p + "_meta.txt");
    int w, h;
    double fx, fy, cx, cy, scale;
    meta >> w >> h >> fx >> fy >> cx >> cy >> scale;
    cv::Mat raw(h, w, CV_16U);
    std::ifstream(p + "_depth.u16", std::ios::binary).read(reinterpret_cast<char *>(raw.data), static_cast<long>(w) * h * 2);
    cv::Mat d;
    raw.convertTo(d, CV_64F, scale);
    Eigen::Matrix3d K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    const auto t0 = std::chrono::steady_clock::now();
    char line[512];
    try {
      const auto g = uppercam::square_far20_geometry(d, K, mount);
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      std::snprintf(line, sizeof(line),
                    "{\"ok\": true, \"raw_gap_mm\": %.9g, \"lateral_m\": %.9g, \"box_yaw_rad\": %.9g, \"near_x\": %.9g, "
                    "\"far_inliers\": %d, \"side_inliers\": %d, \"plane_rmse_mm\": %.9g, \"top_area_m2\": %.9g, \"nflags\": %zu, "
                    "\"ms\": %.3f, \"frame\": \"%s\"}",
                    g.raw_gap_mm, g.lateral_m, g.box_yaw_rad, g.near_x, g.far_inliers, g.side_inliers, g.plane_rmse_mm,
                    g.top_area_m2, g.flags.size(), ms, std::filesystem::path(p).filename().c_str());
    } catch (const std::exception & ex) {
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      std::snprintf(line, sizeof(line), "{\"ok\": false, \"error\": \"%s\", \"ms\": %.3f, \"frame\": \"%s\"}", ex.what(), ms,
                    std::filesystem::path(p).filename().c_str());
    }
    out << line << '\n';
  }
}
