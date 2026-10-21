#include "craft_perception/c310_perception.hpp"

namespace craft_perception {
bool C310Perception::configure(const C310Config &c) {
  if (c.knob_class_id < 0 || c.knob_class_id >= c.yolo.classes) return false;
  if (c.use_knob_lever && !yolo_.configure(c.yolo)) return false;
  knob_class_id_ = c.knob_class_id;
  return true;
}

std::optional<KnobLeverResult> C310Perception::knob_lever(const SharedFrame &frame) {
  if (!yolo_.is_valid_input(frame.bgr)) return {};
  KnobLeverResult result;
  result.detections = yolo_.compute_result(frame.bgr);
  for (const auto &box : result.detections) {
    if (box.class_id != knob_class_id_) continue;
    LeverObservation observation{box, std::nullopt};
    if (lever_.is_valid_input(frame.bgr, box.box)) observation.angle = lever_.compute_result(frame.bgr, box.box);
    result.levers.push_back(observation);
  }
  return result;
}
}
