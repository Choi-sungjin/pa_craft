#pragma once
#include "craft_perception/knob_lever.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/yolo_detector.hpp"
#include <optional>
#include <vector>

namespace craft_perception {
// Filled by PerceptionMaster::configure() from perception.yaml.
struct C310Config {
  bool use_knob_lever = true;
  DetectorConfig yolo;  // knob_sign model; its own instance, independent of the D435
  int knob_class_id = 0;
};

struct LeverObservation {
  Detection detection;
  std::optional<double> angle;  // image-plane rad, 0 up, clockwise
};
struct KnobLeverResult {
  std::vector<Detection> detections;  // every class, for /perception/vision/c310/detections
  std::vector<LeverObservation> levers;
};

// C310 color only. knob_lever runs its own YOLO + lever estimate per frame.
// nullopt = invalid input (master skips, no publish).
class C310Perception {
public:
  bool configure(const C310Config &config);
  std::optional<KnobLeverResult> knob_lever(const SharedFrame &frame);
private:
  int knob_class_id_ = 0;
  YoloDetector yolo_;
  KnobLever lever_;
};
}
