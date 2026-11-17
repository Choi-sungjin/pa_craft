// Parity helper for test/map_tools_parity.py: map yaml + scan points file (x y per line) -> localize result.
//   map_tools_cli MAP_YAML POINTS_TXT [near_x near_y yaw_deg]
#include <cstdio>
#include <fstream>

#include "../src/map_tools.hpp"

int main(int argc, char ** argv) {
  const map_tools::MapGrid grid(argv[1]);
  const map_tools::ScanMatcher m(grid);
  std::vector<map_tools::Pt> pts;
  std::ifstream in(argv[2]);
  for (double x, y; in >> x >> y;) pts.push_back({x, y});
  std::optional<map_tools::Pt> near;
  std::optional<double> yaw;
  if (argc > 5) {
    near = map_tools::Pt{std::stod(argv[3]), std::stod(argv[4])};
    yaw = std::stod(argv[5]) * M_PI / 180;
  }
  const auto r = m.localize(pts, 0.18, 0.92, near, 0.5, yaw);
  if (!r.pose) return std::puts("none"), 0;
  std::printf("%.6f %.6f %.6f %.6f %.6f %d\n", (*r.pose)[0], (*r.pose)[1], (*r.pose)[2], r.score, r.second, r.ambiguous);
}
