#pragma once
#include "craft_perception/perception_types.hpp"
#include "craft_perception/traffic_light.hpp"  // PuckShape
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace craft_perception {
struct PuckTagAxisConfig {
  double tag_size_m = 0.016;                    // printed tag side (black border)
  double length_m = 0.060, width_m = 0.030;     // 30x30x60 cuboid / Ø30x60 cylinder
  cv::Vec3d up{0.0, -0.7234, -0.6904};          // world up in the camera optical frame (URDF camera_top: 43.6 deg down)
  double standing_max_tilt_deg = 45.0;          // axis within this of up -> standing
  double max_fit_error_deg = 15.0;              // worst disagreement between tags of one puck
  double max_center_spread_m = 0.008;           // tags' back-projected puck centers must agree within this...
  double center_spread_per_m = 0.02;            // ...or within this x the puck distance, if larger (PnP of a small
                                                // tag gets worse with distance: at 0.78 m the true reading was 8.5 mm)
  double center_weight_deg_per_mm = 2.0;        // cost of that spread, in deg of axis disagreement per mm
  double box_iou_weight_deg = 60.0;             // with a YOLO box: cost per (1 - IoU) of the projected puck vs the box
  double min_box_iou = 0.6;                     // with a YOLO box: answers projecting below this IoU are dropped
  double box_border_px = 3.0;                   // a box this close to the image edge may be cut: IoU term not used
  double ambiguity_margin_deg = 5.0;            // a different axis this close in cost -> no answer
  double max_reprojection_px = 2.0;             // IPPE solutions above this are dropped
  double shelf_prior_weight = 1.0;              // cost per deg the axis is off both up and level
  int max_tags = 4;
};

// Puck axis from the AprilTags on its faces alone (no depth, no shelf plane).
// Cuboid: tags on all six faces; cylinder: on the two end faces only. Tags are stuck on in any of the
// four 90-degree turns (user, 2026-11-12), but their edges are always parallel or perpendicular to the
// puck's edges, so a side-face tag puts the long axis along its x OR its y axis. Per tag, IPPE_SQUARE
// gives up to two poses; every (pose, face = end | side-x | side-y) combination of the visible tags
// proposes an axis (end: tag normal, side: that tag axis), and the combination where all tags agree and
// distinct faces are orthogonal wins (ties broken by a shelf prior: axis up or level). With tags turned at
// random, the orientation alone cannot tell which of three orthogonal faces is the 30x30 end; the tag
// positions do: a puck center back-projected from each tag (end tag 30 mm in, side tag 15 mm in) only
// agrees for the true face assignment (wrong one: ~15 mm apart), so the center spread is part of the cost.
// One cuboid tag alone is ambiguous (end, or which side axis) -> nullopt, unless a YOLO box is given.
// YOLO box (optional): each combination fixes the whole puck pose, so its 8 corners are projected and the
// bounding box compared with the YOLO box (IoU). A standing and a lying puck project very differently
// (60 vs 30 mm tall): on the 2026-11-12 upper-cam sets the true reading had IoU 0.90-0.95, the wrong ones
// 0.43-0.65, which settles two-tag prisms the center spread could not (PnP noise at 0.7 m) and single-tag ones.
class PuckTagAxis {
public:
  using Corners = std::array<cv::Point2f, 4>;  // aruco order: TL, TR, BR, BL of the tag image
  bool configure(const PuckTagAxisConfig &config);
  bool is_valid_input(const std::vector<Corners> &tags, const Intrinsics &intrinsics) const;
  // shape: from the puck survey; nullopt = unknown (treated as cuboid, the superset).
  // box: the puck's YOLO box [px] in an image of image_size (both or neither); nullopt = tags only.
  std::optional<PuckResult> compute_result(const std::vector<Corners> &tags, const Intrinsics &intrinsics,
    std::optional<PuckShape> shape = std::nullopt, std::optional<cv::Rect2f> box = std::nullopt,
    cv::Size image_size = {}) const;
private:
  PuckTagAxisConfig config_;
};
}  // namespace craft_perception
