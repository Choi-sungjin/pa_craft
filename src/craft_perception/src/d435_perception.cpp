#include "craft_perception/d435_perception.hpp"

namespace craft_perception {
bool D435Perception::configure(const D435Config &c) {
  if (c.knob_class_id < 0 || c.knob_class_id >= c.yolo.classes) return false;
  if (c.use_yolo && !yolo_.configure(c.yolo)) return false;
  if (c.use_aruco && !aruco_.configure(c.aruco_dictionary)) return false;
  if (c.use_puck && (!puck_yolo_.configure(c.puck_yolo) || !puck_axis_.configure(c.puck))) return false;
  if (!knob_depth_.configure(c.knob_grow, c.knob_tolerance_m, c.knob_min_points) ||
      !object_detector_.configure(c.object_inner_fraction) ||
      !obstacle_detector_.configure(c.obstacle_class_ids, c.obstacle_max_range_m, c.obstacle_near_quantile)) {
    return false;
  }
  knob_class_id_ = c.knob_class_id;
  return true;
}

std::optional<std::vector<Detection>> D435Perception::detect(const SharedFrame &frame) {
  if (!yolo_.is_valid_input(frame.bgr)) return {};
  return yolo_.compute_result(frame.bgr);
}

std::optional<std::vector<int>> D435Perception::read_aruco(const SharedFrame &frame) const {
  if (!aruco_.is_valid_input(frame.bgr)) return {};
  return aruco_.compute_result(frame.bgr);
}

std::optional<std::vector<KnobDepthObservation>> D435Perception::knob_depth(const SharedFrame &frame,
  const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const {
  if (!frame.depth.is_valid()) return {};
  std::vector<KnobDepthObservation> output;
  for (const auto &box : boxes) {
    if (box.class_id != knob_class_id_) continue;
    KnobDepthObservation observation{box, std::nullopt};
    if (knob_depth_.is_valid_input(frame.depth, box.box, intrinsics)) {
      observation.depth = knob_depth_.compute_result(frame.depth, box.box, intrinsics);
    }
    output.push_back(observation);
  }
  return output;
}

std::optional<std::vector<ObjectPoint>> D435Perception::detect_objects(const SharedFrame &frame,
  const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const {
  if (!object_detector_.is_valid_input(frame.depth, intrinsics)) return {};
  return object_detector_.compute_result(frame.depth, boxes, intrinsics);
}

std::optional<std::vector<ObjectPoint>> D435Perception::detect_obstacles(const SharedFrame &frame,
  const std::vector<Detection> &boxes, const Intrinsics &intrinsics) const {
  if (!obstacle_detector_.is_valid_input(frame.depth, intrinsics)) return {};
  return obstacle_detector_.compute_result(frame.depth, boxes, intrinsics);
}

std::optional<std::vector<PuckObservation>> D435Perception::puck_axis(
  const SharedFrame &frame, const Intrinsics &intrinsics) {
  if (!puck_yolo_.is_valid_input(frame.bgr) || !frame.depth.is_valid()) return {};
  std::vector<PuckObservation> output;
  for (const auto &box : puck_yolo_.compute_result(frame.bgr)) {
    PuckObservation observation{box, std::nullopt};
    if (puck_axis_.is_valid_input(frame.depth, box.box, intrinsics)) {
      observation.axis = puck_axis_.compute_result(frame.depth, box.box, intrinsics);
    }
    output.push_back(observation);
  }
  return output;
}
}
