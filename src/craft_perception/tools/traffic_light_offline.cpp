// Traffic light offline, the robot's own path: ncnn YOLO (YoloDetector) + TrafficLightCounter on recorded frames.
//   traffic_light_offline <model_dir> <yolo|pixels> <index.csv>   (index.csv lines: <jpg path>,<stamp s>)
// Prints every accepted blink, rejections, discarded cycles, the final count, and per-class detection counts.
#include "craft_perception/traffic_light.hpp"
#include "craft_perception/yolo_detector.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

using namespace craft_perception;

int main(int argc, char **argv) {
  if (argc != 4) { std::fprintf(stderr, "usage: %s model_dir yolo|pixels index.csv\n", argv[0]); return 2; }
  DetectorConfig yolo;
  yolo.model_dir = argv[1]; yolo.classes = 5; yolo.confidence = 0.25f;
  YoloDetector detector;
  if (!detector.configure(yolo)) { std::fprintf(stderr, "model load failed\n"); return 2; }
  TrafficLightConfig light;  // perception.yaml uppercam.traffic_light
  light.red_class_id = 1; light.yellow_class_id = 2; light.green_class_id = 3; light.off_class_id = 4;
  light.min_score = 0.5f; light.yellow_min_score = 0.75f;
  light.color_source = std::string(argv[2]) == "pixels" ? ColorSource::kPixels : ColorSource::kYolo;
  auto &p = light.pixels;
  p.cap_fraction = 0.2; p.on_fraction = 0.10; p.use_ring_hue = false; p.red_cut = 0.49; p.yellow_cut = 0.72;
  TrafficLightCounter counter;
  if (!counter.configure(light)) return 2;
  std::ifstream index(argv[3]);
  std::string line;
  std::map<int, int> hist, hist_strong;
  int frames = 0, discarded = 0;
  std::size_t rejected = 0;
  while (std::getline(index, line)) {
    std::stringstream ss(line); std::string path, stamp;
    std::getline(ss, path, ','); std::getline(ss, stamp, ',');
    const cv::Mat bgr = cv::imread(path, cv::IMREAD_COLOR);
    if (!detector.is_valid_input(bgr)) continue;
    ++frames;
    const auto boxes = detector.compute_result(bgr);
    for (const auto &b : boxes) { ++hist[b.class_id]; if (b.score >= 0.5f) ++hist_strong[b.class_id]; }
    const double t = std::stod(stamp);
    if (const auto blink = counter.update(t, boxes, bgr)) {
      std::printf("blink %s %.2fs %d/%d\n", to_string(blink->color), blink->end_s - blink->start_s, blink->votes, blink->frames);
    }
    for (; rejected < counter.rejected().size(); ++rejected) std::printf("  rejected %s\n", counter.rejected()[rejected].c_str());
    if (counter.discarded_cycles() != discarded) {
      discarded = counter.discarded_cycles(); rejected = 0;
      std::printf("  DISCARDED: %s\n", counter.last_discard().c_str());
    }
  }
  std::string seq;
  for (const auto &b : counter.blinks()) seq += to_string(b.color);
  const auto c = counter.counts();
  std::printf("frames=%d counted=%s R%d Y%d G%d complete=%d discarded=%d\n", frames, seq.c_str(), c.red(), c.yellow(),
    c.green(), counter.complete(), counter.discarded_cycles());
  std::printf("detections (>=0.25 / >=0.5):");
  for (const auto &[k, n] : hist) std::printf(" c%d=%d/%d", k, n, hist_strong[k]);
  std::printf("\n");
  return 0;
}
