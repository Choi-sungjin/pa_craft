#include "craft_perception/d435_perception.hpp"
#include <algorithm>
#include <cmath>

namespace craft_perception {
bool D435Perception::configure(const D435Config &c) {
  if (c.use_yolo && !yolo_.configure(c.yolo)) return false;
  if ((c.use_apriltag || c.use_puck) && !apriltag_.configure(c.apriltag_family)) return false;
  if (!std::isfinite(c.puck_tag_box_scale) || c.puck_tag_box_scale <= 0) return false;
  if (c.puck_method != "tag" && c.puck_method != "donut") return false;
  puck_by_tag_ = c.puck_method == "tag";
  if (c.use_puck && !(puck_by_tag_ ? puck_tag_axis_.configure(c.puck_tag) : puck_axis_.configure(c.puck))) {
    return false;
  }
  if (!object_detector_.configure(c.object_inner_fraction) ||
      !obstacle_detector_.configure(c.obstacle_class_ids, c.obstacle_max_range_m, c.obstacle_near_quantile)) {
    return false;
  }
  tag_registry_ = c.tag_registry;
  puck_tag_box_scale_ = c.puck_tag_box_scale;
  return true;
}

std::optional<std::vector<Detection>> D435Perception::detect(const SharedFrame &frame) {
  if (!yolo_.is_valid_input(frame.bgr)) return {};
  return yolo_.compute_result(frame.bgr);
}

std::optional<std::vector<AprilTagDetection>> D435Perception::read_apriltag(const SharedFrame &frame) const {
  if (!apriltag_.is_valid_input(frame.bgr)) return {};
  return apriltag_.compute_result(frame.bgr);
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
  const SharedFrame &frame, const Intrinsics &intrinsics, const std::unordered_map<int, PuckShape> &shapes) {
  if (!apriltag_.is_valid_input(frame.bgr)) return {};
  if (!puck_by_tag_ && !frame.depth.is_valid()) return {};
  std::vector<PuckObservation> output;
  if (puck_by_tag_) {
    // All visible faces of one puck together, in order of first appearance (tags come sorted left -> right).
    std::vector<int> order;
    std::unordered_map<int, std::vector<AprilTagDetection>> by_puck;
    for (const auto &tag : apriltag_.compute_result(frame.bgr)) {
      const auto target = tag_registry_.find(tag.id);
      if (target == tag_registry_.end() || target->second.role != TagRole::Puck) continue;
      auto &group = by_puck[target->second.puck_id];
      if (group.empty()) order.push_back(target->second.puck_id);
      group.push_back(tag);
    }
    for (const int puck_id : order) {
      const auto &group = by_puck[puck_id];
      cv::Rect2f box = group.front().box;
      std::vector<PuckTagAxis::Corners> corners;
      std::optional<PuckShape> shape;
      for (const auto &tag : group) {
        box |= tag.box;
        corners.push_back(tag.corners);
        const auto found = shapes.find(tag.id);
        if (found != shapes.end()) shape = found->second;
      }
      PuckObservation observation{{box, puck_id, 1.0f, {}}, std::nullopt};
      if (puck_tag_axis_.is_valid_input(corners, intrinsics)) {
        observation.axis = puck_tag_axis_.compute_result(corners, intrinsics, shape);
      }
      output.push_back(observation);
    }
    return output;
  }
  for (const auto &tag : apriltag_.compute_result(frame.bgr)) {
    const auto target = tag_registry_.find(tag.id);
    if (target == tag_registry_.end() || target->second.role != TagRole::Puck) continue;
    const auto center = tag.center;
    cv::Rect2f puck_box(
      center.x - tag.box.width * puck_tag_box_scale_ / 2,
      center.y - tag.box.height * puck_tag_box_scale_ / 2,
      tag.box.width * puck_tag_box_scale_, tag.box.height * puck_tag_box_scale_);
    Detection box{puck_box, target->second.puck_id, 1.0f, {}};
    PuckObservation observation{box, std::nullopt};
    if (puck_axis_.is_valid_input(frame.depth, box.box, intrinsics)) {
      observation.axis = puck_axis_.compute_result(frame.depth, box.box, intrinsics);
    }
    output.push_back(observation);
  }
  return output;
}
}
