#pragma once
#include "craft_perception/apriltag_reader.hpp"
#include "craft_perception/knob_depth.hpp"
#include "craft_perception/object_detector.hpp"
#include "craft_perception/obstacle_detector.hpp"
#include "craft_perception/puck_axis.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/yolo_detector.hpp"
#include <optional>
#include <string>
#include <vector>

namespace craft_perception {
// Filled by PerceptionMaster::configure() from perception.yaml.
struct D435Config {
  bool use_yolo = true, use_apriltag = true, use_puck = true;  // disabled tasks load nothing
  DetectorConfig yolo;      // knob_sign model, shared by knob_depth/object/obstacle
  int knob_class_id = 0;
  std::string apriltag_family = "tag36h11";
  double knob_grow = 2.0, knob_tolerance_m = 0.03;
  int knob_min_points = 50;
  double object_inner_fraction = 0.5;
  std::vector<int> obstacle_class_ids;
  double obstacle_max_range_m = 1.5, obstacle_near_quantile = 0.1;
  DetectorConfig puck_yolo;  // separate puck model, run on its own by puck_axis
  PuckAxisConfig puck;
};

struct KnobDepthObservation {
  Detection detection;
  std::optional<DepthResult> depth;
};
struct PuckObservation {
  Detection detection;
  std::optional<PuckResult> axis;
};

// D435 color + aligned depth. One YOLO inference per frame (detect) feeds
// knob_depth / detect_objects / detect_obstacles through the same boxes; AprilTag and
// puck_axis read the frame directly. nullopt = invalid input (master skips, no publish),
// empty vector = valid frame with nothing found.
// Each function is called from one master task at a time; different functions may run
// concurrently (each owns its own model/state).
class D435Perception {
public:
  bool configure(const D435Config &config);
  std::optional<std::vector<Detection>> detect(const SharedFrame &frame);
  std::optional<std::vector<int>> read_apriltag(const SharedFrame &frame) const;
  std::optional<std::vector<KnobDepthObservation>> knob_depth(const SharedFrame &frame,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
  std::optional<std::vector<ObjectPoint>> detect_objects(const SharedFrame &frame,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
  std::optional<std::vector<ObjectPoint>> detect_obstacles(const SharedFrame &frame,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
  std::optional<std::vector<PuckObservation>> puck_axis(
    const SharedFrame &frame, const Intrinsics &intrinsics);
private:
  int knob_class_id_ = 0;
  YoloDetector yolo_, puck_yolo_;
  AprilTagReader apriltag_;
  KnobDepth knob_depth_;
  ObjectDetector object_detector_;
  ObstacleDetector obstacle_detector_;
  PuckAxis puck_axis_;
};
}
