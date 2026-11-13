#pragma once
#include <unordered_map>
#include "craft_perception/apriltag_reader.hpp"
#include "craft_perception/object_detector.hpp"
#include "craft_perception/obstacle_detector.hpp"
#include "craft_perception/puck_axis.hpp"
#include "craft_perception/puck_tag_axis.hpp"
#include "craft_perception/shared_frame.hpp"
#include "craft_perception/tag_registry.hpp"
#include "craft_perception/yolo_detector.hpp"
#include <optional>
#include <string>
#include <vector>

namespace craft_perception {
// Filled by PerceptionMaster::configure() from perception.yaml.
struct D435Config {
  bool use_yolo = true, use_apriltag = true, use_puck = true;  // disabled tasks load nothing
  DetectorConfig yolo;      // sign model, shared by object/obstacle
  std::string apriltag_family = "tag36h11";
  TagRegistry tag_registry;
  double puck_tag_box_scale = 1.0;
  double object_inner_fraction = 0.5;
  std::vector<int> obstacle_class_ids;
  double obstacle_max_range_m = 1.5, obstacle_near_quantile = 0.1;
  PuckAxisConfig puck;
  std::string puck_method = "tag";  // tag: AprilTag faces only (PuckTagAxis) | donut: tag ROI + depth (PuckAxis)
  PuckTagAxisConfig puck_tag;
};

struct PuckObservation {
  Detection detection;
  std::optional<PuckResult> axis;
};

// D435 color + aligned depth. One YOLO inference per frame (detect) feeds
// detect_objects / detect_obstacles through the same boxes. AprilTag reads
// mission and puck IDs; puck_axis: method tag groups the visible tags of each puck and solves the
// axis from their poses (one observation per puck), method donut expands each puck-tag box and uses depth.
// nullopt = invalid input (master skips, no publish),
// empty vector = valid frame with nothing found.
// Each function is called from one master task at a time; different functions may run
// concurrently (each owns its own model/state).
class D435Perception {
public:
  bool configure(const D435Config &config);
  std::optional<std::vector<Detection>> detect(const SharedFrame &frame);
  std::optional<std::vector<AprilTagDetection>> read_apriltag(const SharedFrame &frame) const;
  std::optional<std::vector<ObjectPoint>> detect_objects(const SharedFrame &frame,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
  std::optional<std::vector<ObjectPoint>> detect_obstacles(const SharedFrame &frame,
    const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const;
  // shapes: puck survey result by tag id (tag method only; missing = unknown).
  std::optional<std::vector<PuckObservation>> puck_axis(const SharedFrame &frame, const Intrinsics &intrinsics,
    const std::unordered_map<int, PuckShape> &shapes = {});
private:
  YoloDetector yolo_;
  AprilTagReader apriltag_;
  ObjectDetector object_detector_;
  ObstacleDetector obstacle_detector_;
  PuckAxis puck_axis_;
  PuckTagAxis puck_tag_axis_;
  bool puck_by_tag_ = true;
  TagRegistry tag_registry_;
  double puck_tag_box_scale_ = 1.0;
};
}
