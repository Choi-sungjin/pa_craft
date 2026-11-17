// slam_toolbox pose graph (<base>.posegraph + <base>.data) tool, using slam_toolbox's own Karto library.
//   info   <base>                       one CSV line per scan: id, time, corrected pose, range stats
//   trim   <base> <out> <first_drop_id> drop every scan with unique id >= first_drop_id, save as <out>.*
//   render <base> <out> [resolution]    occupancy grid of all scans -> <out>.pgm + <out>.yaml (trinary)
//   transform <base> <out> <x> <y> <theta_deg>  move the whole map rigidly: p' = R(theta) p + (x, y), save as <out>.*
// Removal follows lifelong_slam_toolbox::removeFromSlamGraph (graph, sensor manager, dataset).
#include <algorithm>
#include <cmath>
#include <set>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "karto_sdk/Mapper.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

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

// Rigid move of every pose: scans (odometric + corrected, point readings recomputed) and the absolute poses of
// each link. Link pose differences and covariances are in the first pose's frame, so they stay as they are.
int transform(karto::Mapper & mapper, karto::Dataset & dataset, const std::string & out, double tx, double ty,
              double theta) {
  if (exists(out + ".posegraph") || exists(out + ".data")) {std::cerr << out << ".* exists\n"; return 1;}
  const double c = std::cos(theta), s = std::sin(theta);
  auto move = [&](const karto::Pose2 & p) {
    return karto::Pose2(c * p.GetX() - s * p.GetY() + tx, s * p.GetX() + c * p.GetY() + ty,
                        karto::math::NormalizeAngle(p.GetHeading() + theta));
  };
  auto move_scan = [&](karto::LocalizedRangeScan * scan) {
    scan->SetOdometricPose(move(scan->GetOdometricPose()));
    scan->SetCorrectedPoseAndUpdate(move(scan->GetCorrectedPose()));
  };
  const auto scans = sorted_scans(mapper);
  std::set<const karto::Object *> moved(scans.begin(), scans.end());
  for (auto * scan : scans) move_scan(scan);
  std::size_t dataset_scans = 0;
  for (const auto & [id, object] : dataset.GetData())
    if (auto * scan = dynamic_cast<karto::LocalizedRangeScan *>(object); scan && !moved.count(scan)) {
      move_scan(scan); moved.insert(scan); ++dataset_scans;
    }
  std::size_t links = 0;
  for (auto * edge : mapper.GetGraph()->GetEdges()) {
    auto * link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    if (!link) continue;
    const karto::Pose2 p1 = move(link->GetPose1()), p2 = move(link->GetPose2());
    karto::Matrix3 r; r.FromAxisAngle(0, 0, 1, p1.GetHeading());
    link->Update(p1, p2, r * link->GetCovariance() * r.Transpose());  // Update rotates it back by -p1 heading
    ++links;
  }
  std::cerr << "moved " << scans.size() << " graph scans, " << dataset_scans << " other dataset scans, " << links
            << " links by (" << tx << ", " << ty << ", " << theta * 180.0 / M_PI << " deg)\n";
  mapper.SaveToFile(out + ".posegraph");
  dataset.SaveToFile(out + ".data");
  std::cerr << "saved " << out << ".posegraph / .data\n";
  return 0;
}

// One full optimization of the stored graph with slam_toolbox's own Ceres plugin (its default parameters, as
// slam.yaml sets none), saved under a new name. A saved graph is not always at the optimum of its constraints:
// slam_toolbox re-optimizes it at the first loop closure in localization mode, and arena5_legend then turned 2.26 deg
// about its first node (2026-11-08), away from the map A* had been rendered from. Optimize, then render/register.
int optimize(karto::Mapper & mapper, karto::Dataset & dataset, const std::string & out) {
  if (exists(out + ".posegraph") || exists(out + ".data")) {std::cerr << out << ".* exists\n"; return 1;}
  pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox", "karto::ScanSolver");
  auto solver = loader.createSharedInstance("solver_plugins::CeresSolver");
  // The plugin reads these from the slam_toolbox node; mode as at runtime (localization only enables fast removal).
  solver->Configure(std::make_shared<rclcpp_lifecycle::LifecycleNode>("posegraph_tool_solver", rclcpp::NodeOptions()
    .parameter_overrides({{"debug_logging", false}, {"mode", std::string("localization")}})
    .automatically_declare_parameters_from_overrides(true)));
  solver->Reset();
  std::size_t nodes = 0, constraints = 0;
  for (const auto & [name, vertices] : mapper.GetGraph()->GetVertices())
    for (const auto & [id, vertex] : vertices) {solver->AddNode(vertex); ++nodes;}
  for (auto * edge : mapper.GetGraph()->GetEdges()) {solver->AddConstraint(edge); ++constraints;}
  solver->Compute();
  double moved = 0, turned = 0;
  std::size_t applied = 0;
  for (const auto & [id, pose] : solver->GetCorrections()) {
    auto * scan = mapper.GetMapperSensorManager()->GetScan(id);
    if (!scan) continue;
    const auto before = scan->GetCorrectedPose();
    moved = std::max(moved, before.GetPosition().Distance(pose.GetPosition()));
    turned = std::max(turned, std::abs(karto::math::NormalizeAngle(pose.GetHeading() - before.GetHeading())));
    scan->SetCorrectedPoseAndUpdate(pose);
    ++applied;
  }
  std::cerr << "optimized " << nodes << " nodes, " << constraints << " constraints: " << applied
            << " poses corrected, largest move " << moved << " m, " << turned * 180.0 / M_PI << " deg\n";
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
  if (argc < 3) {std::cerr << "usage: info <base> | trim <base> <out> <first_drop_id> | render <base> <out> [res] | "
                                 "transform <base> <out> <x> <y> <theta_deg> | optimize <base> <out>\n"; return 2;}
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
  else if (cmd == "transform" && argc >= 7)
    rc = transform(mapper, dataset, argv[3], std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6]) * M_PI / 180.0);
  else if (cmd == "optimize" && argc >= 4) {rclcpp::init(argc, argv); rc = optimize(mapper, dataset, argv[3]);}
  else std::cerr << "bad arguments\n";
  // Karto's Mapper/Dataset destructors throw on this data; everything is written by now.
  std::cout.flush(); std::fflush(nullptr); std::_Exit(rc);
}
