// puck_axis_offline: run the production PuckAxis on a recorded session (demo/puck_compare/record.py)
// so it sees exactly the frames another method (CenterPose) sees.
// usage: puck_axis_offline <session_dir> <out.csv> [--box apriltag|yolo] [--tags 23,24]
//          [--family tag36h11] [--tag-scale 2.0] [--model-dir <ncnn dir>] [--confidence 0.5]
//          [--grow 2.0] [--standing-height 0.045]
// apriltag box = tag box * tag-scale around the tag center (same as D435Perception::puck_axis);
// no --tags -> every detected tag is a puck. yolo box = puck_blue detections.
#include "craft_perception/apriltag_reader.hpp"
#include "craft_perception/puck_axis.hpp"
#include "craft_perception/yolo_detector.hpp"
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>

using namespace craft_perception;
namespace {
double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}
}  // namespace

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "usage: puck_axis_offline <session_dir> <out.csv> [--box apriltag|yolo] [--tags a,b]"
                 " [--family f] [--tag-scale s] [--model-dir d] [--confidence c] [--grow g]"
                 " [--standing-height h]\n";
    return 2;
  }
  const std::string session = argv[1], out_path = argv[2];
  std::map<std::string, std::string> opt{{"--box", "apriltag"}, {"--tags", ""}, {"--family", "tag36h11"},
    {"--tag-scale", "2.0"}, {"--model-dir", ""}, {"--confidence", "0.5"}, {"--grow", "2.0"},
    {"--standing-height", "0.045"}};
  for (int i = 3; i + 1 < argc; i += 2) {
    if (!opt.count(argv[i])) { std::cerr << "unknown option " << argv[i] << "\n"; return 2; }
    opt[argv[i]] = argv[i + 1];
  }
  std::set<int> tags;
  std::stringstream tag_list(opt["--tags"]);
  for (std::string id; std::getline(tag_list, id, ',');) if (!id.empty()) tags.insert(std::stoi(id));
  const double tag_scale = std::stod(opt["--tag-scale"]);
  const bool use_yolo = opt["--box"] == "yolo";
  if (!use_yolo && opt["--box"] != "apriltag") { std::cerr << "--box must be apriltag|yolo\n"; return 2; }

  cv::FileStorage meta(session + "/meta.json", cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
  if (!meta.isOpened()) { std::cerr << "cannot open " << session << "/meta.json\n"; return 1; }
  const Intrinsics in{meta["fx"].real(), meta["fy"].real(), meta["cx"].real(), meta["cy"].real()};
  const double depth_unit_m = meta["depth_unit_m"].real();

  PuckAxisConfig puck;
  puck.grow = std::stod(opt["--grow"]);
  puck.standing_height_m = std::stod(opt["--standing-height"]);
  PuckAxis axis;
  if (!axis.configure(puck)) { std::cerr << "bad puck_axis config\n"; return 1; }
  AprilTagReader apriltag;
  YoloDetector yolo;
  if (use_yolo) {
    DetectorConfig c;
    c.model_dir = opt["--model-dir"];
    c.classes = 1;  // puck_blue: target_blue
    c.confidence = std::stof(opt["--confidence"]);
    if (c.model_dir.empty() || !yolo.configure(c)) { std::cerr << "yolo configure failed\n"; return 1; }
  } else if (!apriltag.configure(opt["--family"])) {
    std::cerr << "bad apriltag family\n"; return 1;
  }

  std::ifstream frames(session + "/frames.csv");
  if (!frames) { std::cerr << "cannot open " << session << "/frames.csv\n"; return 1; }
  std::ofstream out(out_path);
  out << "frame,stamp_ns,box_src,id,bx,by,bw,bh,score,axis_valid,cx,cy,cz,ax,ay,az,nx,ny,nz,"
         "standing,height_m,plane_rms_m,inlier_ratio,points,detect_ms,axis_ms\n";
  std::string line;
  std::getline(frames, line);  // header: frame,stamp_ns,sequence
  int frame_count = 0, rows = 0, valid = 0;
  while (std::getline(frames, line)) {
    std::stringstream fields(line);
    std::string frame, stamp;
    std::getline(fields, frame, ',');
    std::getline(fields, stamp, ',');
    const cv::Mat bgr = cv::imread(session + "/frames/" + frame + "_color.png", cv::IMREAD_COLOR);
    const cv::Mat raw = cv::imread(session + "/frames/" + frame + "_depth.png", cv::IMREAD_ANYDEPTH);
    if (bgr.empty() || raw.empty() || raw.type() != CV_16UC1) {
      std::cerr << "frame " << frame << ": missing/bad png, skipped\n";
      continue;
    }
    ++frame_count;
    const DepthImage depth{raw, depth_unit_m, {}};

    const auto t_detect = std::chrono::steady_clock::now();
    std::vector<Detection> boxes;
    if (use_yolo) {
      boxes = yolo.compute_result(bgr);
    } else {
      for (const auto &tag : apriltag.compute_result(bgr)) {
        if (!tags.empty() && !tags.count(tag.id)) continue;
        boxes.push_back({cv::Rect2f(tag.center.x - tag.box.width * tag_scale / 2,
          tag.center.y - tag.box.height * tag_scale / 2,
          tag.box.width * tag_scale, tag.box.height * tag_scale), tag.id, 1.0f, {}});
      }
    }
    const double detect_ms = ms_since(t_detect);

    for (const auto &box : boxes) {
      const auto t_axis = std::chrono::steady_clock::now();
      std::optional<PuckResult> r;
      if (axis.is_valid_input(depth, box.box, in)) r = axis.compute_result(depth, box.box, in);
      const double axis_ms = ms_since(t_axis);
      char buffer[768];
      if (r) {
        std::snprintf(buffer, sizeof(buffer),
          "%s,%s,%s,%d,%.1f,%.1f,%.1f,%.1f,%.3f,1,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.4f,%.5f,%.3f,%d,%.2f,%.2f\n",
          frame.c_str(), stamp.c_str(), opt["--box"].c_str(), box.class_id, box.box.x, box.box.y,
          box.box.width, box.box.height, box.score, r->center[0], r->center[1], r->center[2],
          r->axis[0], r->axis[1], r->axis[2], r->normal[0], r->normal[1], r->normal[2],
          r->standing ? 1 : 0, r->height_m, r->plane_rms_m, r->ring_inlier_ratio, r->points,
          detect_ms, axis_ms);
        ++valid;
      } else {
        std::snprintf(buffer, sizeof(buffer), "%s,%s,%s,%d,%.1f,%.1f,%.1f,%.1f,%.3f,0,,,,,,,,,,,,,,,%.2f,%.2f\n",
          frame.c_str(), stamp.c_str(), opt["--box"].c_str(), box.class_id, box.box.x, box.box.y,
          box.box.width, box.box.height, box.score, detect_ms, axis_ms);
      }
      out << buffer;
      ++rows;
    }
    if (boxes.empty()) out << frame << "," << stamp << "," << opt["--box"] << ",-1,,,,,,0,,,,,,,,,,,,,,," << detect_ms << ",0\n";
  }
  std::cout << frame_count << " frames, " << rows << " puck boxes, " << valid << " axis valid -> " << out_path << "\n";
  return 0;
}
