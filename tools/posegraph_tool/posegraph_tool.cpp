// slam_toolbox pose graph (<base>.posegraph + <base>.data) tool, using slam_toolbox's own Karto library.
//   info   <base>                       one CSV line per scan: id, time, corrected pose, range stats
//   trim   <base> <out> <first_drop_id> drop every scan with unique id >= first_drop_id, save as <out>.*
//   render <base> <out> [resolution]    occupancy grid of all scans -> <out>.pgm + <out>.yaml (trinary)
// Removal follows lifelong_slam_toolbox::removeFromSlamGraph (graph, sensor manager, dataset).
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "karto_sdk/Mapper.h"

namespace {
// RemoveNodeFromGraph() also tells the optimizer; nothing is optimized here, so a do-nothing solver.
class NullSolver : public karto::ScanSolver {
public:
  void Compute() override {}
  void Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr) override {}
  const IdPoseVector & GetCorrections() const override {return corrections_;}
private:
  IdPoseVector corrections_;
};

bool exists(const std::string & f) {struct stat b; return stat(f.c_str(), &b) == 0;}

std::vector<karto::LocalizedRangeScan *> sorted_scans(karto::Mapper & mapper) {
  auto scans = mapper.GetAllProcessedScans();
  std::sort(scans.begin(), scans.end(), [](auto a, auto b) {return a->GetUniqueId() < b->GetUniqueId();});
  return scans;
}

int info(karto::Mapper & mapper) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  const auto scans = sorted_scans(mapper);
  std::size_t vertices = 0;
  for (const auto & [name, map] : mapper.GetGraph()->GetVertices()) vertices += map.size();
  std::cerr << scans.size() << " scans, " << vertices << " vertices, " << mapper.GetGraph()->GetEdges().size()
            << " edges\n";
  std::printf("id,time,x,y,theta,valid,median_range,p90_range\n");
  for (auto * s : scans) {
    karto::RangeReadingsVector r;
    try {r = s->GetRangeReadingsVector();} catch (const karto::Exception & e) {
      std::printf("%d,,,,,0,,  # range readings: %s\n", s->GetUniqueId(), e.GetErrorMessage().c_str());
      continue;
    } catch (...) {std::printf("%d,,,,,0,,  # range readings: unknown error\n", s->GetUniqueId()); continue;}
    std::vector<double> v;
    for (double x : r) if (std::isfinite(x) && x > 0.0) v.push_back(x);
    std::sort(v.begin(), v.end());
    const double med = v.empty() ? NAN : v[v.size() / 2], p90 = v.empty() ? NAN : v[v.size() * 9 / 10];
    const auto & p = s->GetCorrectedPose();
    std::printf("%d,%.3f,%.4f,%.4f,%.4f,%zu,%.3f,%.3f\n", s->GetUniqueId(), s->GetTime(), p.GetX(), p.GetY(),
                p.GetHeading(), v.size(), med, p90);
  }
  return 0;
}

int trim(karto::Mapper & mapper, karto::Dataset & dataset, const std::string & out, int first_drop) {
  if (exists(out + ".posegraph") || exists(out + ".data")) {std::cerr << out << ".* exists\n"; return 1;}
  NullSolver solver;
  mapper.SetScanSolver(&solver);
  std::vector<karto::Vertex<karto::LocalizedRangeScan> *> drop;
  for (const auto & [name, map] : mapper.GetGraph()->GetVertices())
    for (const auto & [id, vertex] : map) if (id >= first_drop) drop.push_back(vertex);
  std::sort(drop.begin(), drop.end(), [](auto a, auto b) {return a->GetObject()->GetUniqueId() > b->GetObject()->GetUniqueId();});
  for (auto * vertex : drop) {
    mapper.RemoveNodeFromGraph(vertex);
    mapper.GetMapperSensorManager()->RemoveScan(vertex->GetObject());
    dataset.RemoveData(vertex->GetObject());
    vertex->RemoveObject();
    delete vertex;
  }
  mapper.SetScanSolver(nullptr);
  std::cerr << "dropped " << drop.size() << " scans (id >= " << first_drop << "), "
            << sorted_scans(mapper).size() << " left\n";
  mapper.SaveToFile(out + ".posegraph");
  dataset.SaveToFile(out + ".data");
  std::cerr << "saved " << out << ".posegraph / .data\n";
  return 0;
}

int render(karto::Mapper & mapper, const std::string & out, double resolution) {
  if (exists(out + ".pgm") || exists(out + ".yaml")) {std::cerr << out << ".pgm/.yaml exists\n"; return 1;}
  const auto scans = sorted_scans(mapper);
  karto::LocalizedRangeScanVector v(scans.begin(), scans.end());
  // slam_toolbox defaults: min_pass_through 2, occupancy_threshold 0.1
  std::unique_ptr<karto::OccupancyGrid> grid(karto::OccupancyGrid::CreateFromScans(v, resolution, 2, 0.1));
  if (!grid) {std::cerr << "no scans\n"; return 1;}
  const int w = grid->GetWidth(), h = grid->GetHeight();
  const auto offset = grid->GetCoordinateConverter()->GetOffset();
  std::ofstream pgm(out + ".pgm", std::ios::binary);
  pgm << "P5\n# posegraph_tool render\n" << w << " " << h << "\n255\n";
  for (int y = h - 1; y >= 0; --y)
    for (int x = 0; x < w; ++x) {
      const auto c = grid->GetValue(karto::Vector2<kt_int32s>(x, y));
      pgm.put(static_cast<char>(c == karto::GridStates_Occupied ? 0 : c == karto::GridStates_Free ? 254 : 205));
    }
  std::ofstream yaml(out + ".yaml");
  const std::string image = out.substr(out.find_last_of('/') + 1) + ".pgm";
  yaml << "image: " << image << "\nmode: trinary\nresolution: " << resolution << "\norigin: [" << offset.GetX()
       << ", " << offset.GetY() << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  std::cerr << "rendered " << scans.size() << " scans -> " << out << ".pgm (" << w << "x" << h << ")\n";
  return 0;
}
}  // namespace

int main(int argc, char ** argv) {
  if (argc < 3) {std::cerr << "usage: info <base> | trim <base> <out> <first_drop_id> | render <base> <out> [res]\n"; return 2;}
  const std::string cmd = argv[1], base = argv[2];
  karto::Mapper mapper;
  karto::Dataset dataset;
  mapper.LoadFromFile(base + ".posegraph");
  dataset.LoadFromFile(base + ".data");
  // Scans look their laser up by name (angles for point readings): register it as slam_toolbox does on load.
  for (auto * object : dataset.GetLasers())
    if (auto * sensor = dynamic_cast<karto::Sensor *>(object)) karto::SensorManager::GetInstance()->RegisterSensor(sensor);
  int rc = 2;
  if (cmd == "info") rc = info(mapper);
  else if (cmd == "trim" && argc >= 5) rc = trim(mapper, dataset, argv[3], std::stoi(argv[4]));
  else if (cmd == "render" && argc >= 4) rc = render(mapper, argv[3], argc >= 5 ? std::stod(argv[4]) : 0.02);
  else std::cerr << "bad arguments\n";
  // Karto's Mapper/Dataset destructors throw on this data; everything is written by now.
  std::cout.flush(); std::fflush(nullptr); std::_Exit(rc);
}
