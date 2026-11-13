// Box finder check on recorded /perception/lidar/obstacle messages (dump_obstacles.py: "robot_x robot_y n x y ...").
//   uppercam_find_box_check <dump> <expected_x> <expected_y> <expected_yaw_deg> [true_x true_y]
// Prints the box found in each message; with the true centre, fails (exit 1) if a message's box is over 2 cm off.
#include "uppercam_geometry.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 5 && argc != 7) { std::fprintf(stderr, "usage: uppercam_find_box_check <dump> <x> <y> <yaw_deg> [true_x true_y]\n"); return 2; }
  const Eigen::Vector3d expected(std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]) * M_PI / 180);
  std::ifstream in(argv[1]);
  std::string line;
  int found = 0, total = 0, bad = 0;
  while (std::getline(in, line)) {
    std::istringstream s(line);
    Eigen::Vector2d robot;
    int n;
    s >> robot.x() >> robot.y() >> n;
    std::vector<Eigen::Vector2d> pts(n);
    for (auto & q : pts) s >> q.x() >> q.y();
    ++total;
    const auto b = uppercam::find_box(pts, expected, robot, 0.205);
    if (!b) { std::printf("none\n"); continue; }
    ++found;
    std::printf("box (%.3f, %.3f) yaw %.1f deg, %d points, extent %.3f x %.3f\n", b->x, b->y, b->yaw * 180 / M_PI, b->points,
                b->extent_u, b->extent_v);
    if (argc == 7 && std::hypot(b->x - std::stod(argv[5]), b->y - std::stod(argv[6])) > 0.02) ++bad;
  }
  std::printf("found in %d of %d messages, %d over 2 cm off\n", found, total, bad);
  return bad || (argc == 7 && found == 0) ? 1 : 0;
}
