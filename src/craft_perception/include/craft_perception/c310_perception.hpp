#pragma once
#include "craft_perception/knob_lever.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/yolo_detector.hpp"
#include <optional>
#include <string>
#include <vector>

namespace craft_perception {
// Filled by PerceptionMaster::configure() from perception.yaml.
struct C310Config {
  bool use_knob_lever = true;
  DetectorConfig yolo;  // knob_sign model; its own instance, independent of the D435
  int knob_class_id = 0;
  std::string lever_source = "hsv";  // hsv | yolo_seg
  int lever_class_id = -1;           // yolo_seg: class of the lever masks in yolo
};

struct LeverObservation {
  Detection detection;
  std::optional<double> angle;  // image-plane rad, 0 up, clockwise
};
struct KnobLeverResult {
  std::vector<Detection> detections;  // every class, for /perception/vision/c310/detections
  std::vector<LeverObservation> levers;
};

// C310 color only. knob_lever runs its own YOLO + lever estimate per frame. Lever pixels come
// from the red-HSV heuristic or from the seg model's lever mask inside each knob box.
// nullopt = invalid input (master skips, no publish).
class C310Perception {
public:
  bool configure(const C310Config &config);
  std::optional<KnobLeverResult> knob_lever(const SharedFrame &frame);
private:
  std::optional<double> seg_lever(const Detection &knob, const std::vector<Detection> &detections,
    cv::Size image) const;
  int knob_class_id_ = 0, lever_class_id_ = -1;
  bool use_seg_ = false;
  YoloDetector yolo_;
  KnobLever lever_;
};
}
