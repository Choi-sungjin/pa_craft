// Erase the arena boxes from a slam_toolbox map (pose graph + data), so localization matches the fixed structure only.
//
//   erase_boxes <map base> <out base> <erase_mask.yaml> [--dump before.csv after.csv]
//
// <map base>.posegraph/.data are slam_toolbox's save (karto Mapper + Dataset). <erase_mask.yaml> is a ROS map YAML
// whose black cells are the places to erase (make_erase_mask.py: every box footprint grown by a margin, walls left
// out). Every scan beam ending in a black cell gets a range past the laser's range threshold: karto then leaves it
// out of the matched point readings (scan matching, localization) and keeps it only as a no-hit ray in the rendered
// grid. Writes <out base>.posegraph/.data; the input is not changed. --dump writes the matched points of every scan
// (map frame) before and after, for plot_erase.py.
// 2026-11-10: the localization pulled the robot 30 mm sideways near box L1, which had moved 36 mm since mapping.
#include <karto_sdk/Mapper.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Mask
{
  int width = 0, height = 0;
  double resolution = 0, ox = 0, oy = 0;
  std::vector<unsigned char> pixels;  // row 0 = top (the PGM's order)
  bool erase(double x, double y) const
  {
    const int ix = static_cast<int>(std::floor((x - ox) / resolution));
    const int iy = height - 1 - static_cast<int>(std::floor((y - oy) / resolution));
    return ix >= 0 && iy >= 0 && ix < width && iy < height && pixels[iy * width + ix] < 128;
  }
};

static Mask read_mask(const std::string & yaml_path)
{
  std::ifstream in(yaml_path); std::stringstream ss; ss << in.rdbuf(); const std::string text = ss.str();
  Mask m;
  std::smatch g;
  if (!std::regex_search(text, g, std::regex("image:\\s*(\\S+)"))) throw std::runtime_error("mask yaml: no image");
  std::string image = g[1];
  if (image.front() != '/') {
    const auto slash = yaml_path.find_last_of('/');
    image = (slash == std::string::npos ? "" : yaml_path.substr(0, slash + 1)) + image;
  }
  if (!std::regex_search(text, g, std::regex("resolution:\\s*([-0-9.eE+]+)"))) throw std::runtime_error("mask yaml: no resolution");
  m.resolution = std::stod(g[1]);
  // origin as a block list (- x / - y / - 0) or a flow list ([x, y, 0])
  if (std::regex_search(text, g, std::regex("origin:\\s*\\[\\s*([-0-9.eE+]+)\\s*,\\s*([-0-9.eE+]+)")) ||
      std::regex_search(text, g, std::regex("origin:\\s*-\\s*([-0-9.eE+]+)\\s*-\\s*([-0-9.eE+]+)"))) {
    m.ox = std::stod(g[1]); m.oy = std::stod(g[2]);
  } else {
    throw std::runtime_error("mask yaml: no origin");
  }
  std::ifstream pgm(image, std::ios::binary);
  std::string magic; int maxval = 0;
  pgm >> magic >> m.width >> m.height >> maxval; pgm.get();
  if (magic != "P5" || maxval != 255) throw std::runtime_error("mask: 8-bit binary PGM (P5) expected: " + image);
  m.pixels.resize(static_cast<std::size_t>(m.width) * m.height);
  pgm.read(reinterpret_cast<char *>(m.pixels.data()), static_cast<std::streamsize>(m.pixels.size()));
  if (!pgm) throw std::runtime_error("mask: short PGM " + image);
  return m;
}

static void dump(const karto::LocalizedRangeScanVector & scans, const std::string & path)
{
  std::ofstream out(path);
  out << "scan,x,y\n";
  for (const auto * scan : scans) {
    for (const auto & p : scan->GetPointReadings(true)) out << scan->GetUniqueId() << ',' << p.GetX() << ',' << p.GetY() << '\n';
  }
}

static int run(int argc, char ** argv)
{
  if (argc < 4) {
    std::cerr << "usage: erase_boxes <map base> <out base> <erase_mask.yaml> [--dump before.csv after.csv]\n";
    return 2;
  }
  const std::string in = argv[1], out = argv[2];
  const Mask mask = read_mask(argv[3]);
  std::string dump_before, dump_after;
  for (int i = 4; i < argc; ++i) {
    if (std::string(argv[i]) == "--dump" && i + 2 < argc) {dump_before = argv[++i]; dump_after = argv[++i];}
  }
  std::cout << "mask " << mask.width << "x" << mask.height << " @ " << mask.resolution << " m" << std::endl;

  karto::Mapper mapper;
  karto::Dataset dataset;
  mapper.LoadFromFile(in + ".posegraph");
  dataset.LoadFromFile(in + ".data");
  // Loading does not register the lasers (slam_toolbox does it itself after loading): the scans look theirs up.
  for (auto * object : dataset.GetLasers()) {
    if (auto * sensor = dynamic_cast<karto::Sensor *>(object)) karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);
  }
  auto scans = mapper.GetMapperSensorManager()->GetAllScans();
  std::cout << scans.size() << " scans in the pose graph, " << dataset.GetData().size() << " in the data" << std::endl;
  if (!dump_before.empty()) dump(scans, dump_before);

  // The pose graph's scans and the data's are separate objects after loading: edit both.
  std::vector<karto::LocalizedRangeScan *> all(scans.begin(), scans.end());
  for (const auto & [id, data] : dataset.GetData()) {
    if (auto * scan = dynamic_cast<karto::LocalizedRangeScan *>(data)) all.push_back(scan);
  }
  long beams = 0, erased = 0;
  for (auto * scan : all) {
    const auto * laser = scan->GetLaserRangeFinder();
    const double threshold = laser->GetRangeThreshold(), minimum = laser->GetMinimumRange();
    const karto::Pose2 pose = scan->GetSensorPose();
    auto ranges = scan->GetRangeReadingsVector();
    bool changed = false;
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      const double r = ranges[i];
      if (!(r >= minimum && r <= threshold)) continue;
      ++beams;
      const double a = pose.GetHeading() + laser->GetMinimumAngle() + i * laser->GetAngularResolution();
      if (mask.erase(pose.GetX() + r * std::cos(a), pose.GetY() + r * std::sin(a))) {
        ranges[i] = threshold + 1.0; changed = true; ++erased;
      }
    }
    if (changed) {
      scan->SetRangeReadings(ranges);
      kt_bool dirty = true; scan->SetIsDirty(dirty);
      scan->GetPointReadings(true);  // recompute points, barycenter and bounding box now
    }
  }
  std::cout << "erased " << erased << " of " << beams << " beams (pose graph and data together)" << std::endl;
  if (!dump_after.empty()) dump(scans, dump_after);
  mapper.SaveToFile(out + ".posegraph");
  dataset.SaveToFile(out + ".data");
  std::cout << "wrote " << out << ".posegraph / .data" << std::endl;
  return 0;
}

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const karto::Exception & e) {
    std::cerr << "karto: " << e.GetErrorMessage() << std::endl;
  } catch (const std::exception & e) {
    std::cerr << e.what() << std::endl;
  }
  return 1;
}
