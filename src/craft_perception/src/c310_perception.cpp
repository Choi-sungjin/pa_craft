#include "craft_perception/c310_perception.hpp"
#include "geometry_utils.hpp"

namespace craft_perception {
bool C310Perception::configure(const C310Config &c) {
  if (c.knob_class_id < 0 || c.knob_class_id >= c.yolo.classes) return false;
  if (c.lever_source != "hsv" && c.lever_source != "yolo_seg") return false;
  use_seg_ = c.lever_source == "yolo_seg";
  if (use_seg_ && (c.yolo.proto_blob.empty() || c.lever_class_id < 0 ||
                   c.lever_class_id >= c.yolo.classes || c.lever_class_id == c.knob_class_id)) {
    return false;
  }
  if (c.use_knob_lever && !yolo_.configure(c.yolo)) return false;
  knob_class_id_ = c.knob_class_id;
  lever_class_id_ = c.lever_class_id;
  return true;
}

std::optional<KnobLeverResult> C310Perception::knob_lever(const SharedFrame &frame) {
  if (!yolo_.is_valid_input(frame.bgr)) return {};
  KnobLeverResult result;
  result.detections = yolo_.compute_result(frame.bgr);
  for (const auto &box : result.detections) {
    if (box.class_id != knob_class_id_) continue;
    LeverObservation observation{box, std::nullopt};
    if (use_seg_) {
      observation.angle = seg_lever(box, result.detections, frame.bgr.size());
    } else if (lever_.is_valid_input(frame.bgr, box.box)) {
      observation.angle = lever_.compute_result(frame.bgr, box.box);
    }
    result.levers.push_back(observation);
  }
  return result;
}

// Best-scoring lever whose center lies in the knob box; its mask is copied into the knob crop
// so the knob center stays the pivot.
std::optional<double> C310Perception::seg_lever(const Detection &knob,
  const std::vector<Detection> &detections, cv::Size image) const {
  const auto knob_roi = detail::clipped(knob.box, image);
  if (knob_roi.empty()) return {};
  const Detection *best = nullptr;
  for (const auto &d : detections) {
    const cv::Point2f center(d.box.x + d.box.width / 2, d.box.y + d.box.height / 2);
    if (d.class_id == lever_class_id_ && !d.mask.empty() && knob.box.contains(center) &&
        (!best || d.score > best->score)) best = &d;
  }
  if (!best) return {};
  cv::Mat crop(knob_roi.size(), CV_8U, cv::Scalar(0));
  const auto lever_roi = detail::clipped(best->box, image);
  const auto overlap = lever_roi & knob_roi;
  if (overlap.empty()) return {};
  best->mask(overlap - lever_roi.tl()).copyTo(crop(overlap - knob_roi.tl()));
  if (!lever_.is_valid_input(crop)) return {};
  return lever_.compute_result(crop);
}
}
