#include "craft_perception/puck_tag_axis.hpp"
#include "geometry_utils.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace craft_perception {
using namespace detail;
namespace {
constexpr double kDeg = 180.0 / M_PI;
constexpr double kDistinctAxisDeg = 20.0;   // two answers further apart than this are different answers
constexpr double kReprojectionWeight = 2.0; // deg of cost per px of extra IPPE reprojection error

struct TagPose {
  cv::Vec3d normal, along_x, along_y, position;  // face normal (toward camera), tag x/y axes, tag center
  double extra_px;                    // reprojection error above this tag's best solution
};

double angle_deg(const cv::Vec3d &a, const cv::Vec3d &b) {  // sign-free line angle
  return std::acos(std::clamp(std::abs(a.dot(b)), 0.0, 1.0)) * kDeg;
}

cv::Vec3d principal_axis(const std::vector<cv::Vec3d> &lines) {
  cv::Matx33d scatter = cv::Matx33d::zeros();
  for (const auto &l : lines) scatter += l * l.t();
  cv::Mat values, vectors;
  cv::eigen(cv::Mat(scatter), values, vectors);
  return {vectors.at<double>(0, 0), vectors.at<double>(0, 1), vectors.at<double>(0, 2)};
}

struct Combination {
  double cost = std::numeric_limits<double>::infinity();  // fit + IPPE reprojection + shelf prior
  double fit = 0;                                          // worst tag disagreement [deg]
  double spread = 0;                                       // worst tag center off the mean [m]
  double iou = -1;                                         // projected puck vs YOLO box (-1 = not used)
  cv::Vec3d axis, center;
};
}  // namespace

bool PuckTagAxis::configure(const PuckTagAxisConfig &c) {
  const double positive[] = {c.tag_size_m, c.length_m, c.width_m, c.standing_max_tilt_deg,
                             c.max_fit_error_deg, c.max_reprojection_px, c.max_center_spread_m};
  for (const double value : positive) if (!std::isfinite(value) || value <= 0) return false;
  if (!std::isfinite(c.ambiguity_margin_deg) || c.ambiguity_margin_deg < 0) return false;
  if (!std::isfinite(c.shelf_prior_weight) || c.shelf_prior_weight < 0) return false;
  if (!std::isfinite(c.center_weight_deg_per_mm) || c.center_weight_deg_per_mm < 0) return false;
  if (!std::isfinite(c.center_spread_per_m) || c.center_spread_per_m < 0) return false;
  if (!std::isfinite(c.box_iou_weight_deg) || c.box_iou_weight_deg < 0) return false;
  if (!std::isfinite(c.min_box_iou) || c.min_box_iou < 0 || c.min_box_iou > 1) return false;
  if (!std::isfinite(c.box_border_px) || c.box_border_px < 0) return false;
  if (c.standing_max_tilt_deg >= 90 || c.max_tags < 1 || c.max_tags > 6) return false;
  const double up = cv::norm(c.up);
  if (!std::isfinite(up) || up < 1e-6) return false;
  config_ = c;
  config_.up = c.up / up;
  return true;
}

bool PuckTagAxis::is_valid_input(const std::vector<Corners> &tags, const Intrinsics &intrinsics) const {
  return !tags.empty() && is_valid_intrinsics(intrinsics);
}

std::optional<PuckResult> PuckTagAxis::compute_result(const std::vector<Corners> &input,
  const Intrinsics &in, std::optional<PuckShape> shape, std::optional<cv::Rect2f> box, cv::Size image_size) const {
  // The IoU term only for a box that is not cut by the image border (a cut box is smaller than the puck).
  const bool use_box = box && box->area() > 0 && image_size.width > 0 && image_size.height > 0 &&
    box->x > config_.box_border_px && box->y > config_.box_border_px &&
    box->x + box->width < image_size.width - config_.box_border_px &&
    box->y + box->height < image_size.height - config_.box_border_px;
  // Largest tags first: they give the steadiest poses.
  std::vector<Corners> tags = input;
  std::sort(tags.begin(), tags.end(), [](const Corners &a, const Corners &b) {
    return std::abs(cv::contourArea(std::vector<cv::Point2f>(a.begin(), a.end()))) >
           std::abs(cv::contourArea(std::vector<cv::Point2f>(b.begin(), b.end())));
  });
  if (static_cast<int>(tags.size()) > config_.max_tags) tags.resize(config_.max_tags);

  const double h = config_.tag_size_m / 2;
  const std::vector<cv::Point3f> object{{float(-h), float(h), 0}, {float(h), float(h), 0},
                                        {float(h), float(-h), 0}, {float(-h), float(-h), 0}};
  const cv::Matx33d camera(in.fx, 0, in.cx, 0, in.fy, in.cy, 0, 0, 1);
  std::vector<std::vector<TagPose>> poses;
  for (const auto &corners : tags) {
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat errors;
    const std::vector<cv::Point2f> image(corners.begin(), corners.end());
    try {
      cv::solvePnPGeneric(object, image, camera, cv::noArray(), rvecs, tvecs, false,
        cv::SOLVEPNP_IPPE_SQUARE, cv::noArray(), cv::noArray(), errors);
    } catch (const cv::Exception &) { continue; }
    std::vector<TagPose> solutions;
    double best_px = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < rvecs.size(); ++k) best_px = std::min(best_px, errors.at<double>(int(k)));
    for (size_t k = 0; k < rvecs.size(); ++k) {
      const double px = errors.at<double>(int(k));
      if (!(px <= config_.max_reprojection_px)) continue;
      cv::Matx33d R;
      cv::Rodrigues(rvecs[k], R);
      const cv::Vec3d t(tvecs[k].at<double>(0), tvecs[k].at<double>(1), tvecs[k].at<double>(2));
      if (!(t[2] > 0)) continue;
      cv::Vec3d normal(R(0, 2), R(1, 2), R(2, 2));
      if (normal.dot(t) > 0) normal = -normal;  // a visible face points at the camera
      solutions.push_back({normal, cv::Vec3d(R(0, 0), R(1, 0), R(2, 0)), cv::Vec3d(R(0, 1), R(1, 1), R(2, 1)), t,
                           px - best_px});
    }
    if (!solutions.empty()) poses.push_back(std::move(solutions));
  }
  if (poses.empty()) return {};

  // Every (pose, face) choice per tag: 0 = end, 1 = side with the long axis along tag x, 2 = along tag y.
  // A cylinder has end faces only.
  const bool cylinder = shape == PuckShape::kCylinder;
  const int faces = cylinder ? 1 : 3;
  size_t total = 1;
  for (const auto &p : poses) total *= p.size() * faces;
  std::vector<Combination> results;
  for (size_t index = 0; index < total; ++index) {
    size_t rest = index;
    std::vector<const TagPose *> chosen;
    std::vector<bool> end;
    std::vector<int> face;
    int ends = 0;
    for (const auto &p : poses) {
      const size_t options = p.size() * faces;
      const size_t pick = rest % options;
      rest /= options;
      chosen.push_back(&p[pick / faces]);
      face.push_back(cylinder ? 0 : static_cast<int>(pick % faces));
      end.push_back(face.back() == 0);
      ends += end.back();
    }
    if (!cylinder && ends > 1) continue;  // the two ends of a cuboid are never both visible
    std::vector<cv::Vec3d> lines, centers;
    cv::Vec3d center(0, 0, 0);
    double extra_px = 0;
    for (size_t i = 0; i < chosen.size(); ++i) {
      const auto &p = *chosen[i];
      lines.push_back(face[i] == 0 ? p.normal : face[i] == 1 ? p.along_x : p.along_y);
      centers.push_back(p.position - p.normal * ((end[i] ? config_.length_m : config_.width_m) / 2));
      center += centers.back();
      extra_px += p.extra_px;
    }
    Combination c;
    c.axis = principal_axis(lines);
    c.center = center / static_cast<double>(chosen.size());
    for (const auto &q : centers) c.spread = std::max(c.spread, cv::norm(q - c.center));
    const double spread_gate = std::max(config_.max_center_spread_m, config_.center_spread_per_m * c.center[2]);
    if (c.spread > spread_gate) continue;  // these faces cannot belong to one puck
    if (use_box) {
      // Whole pose of this reading: the axis, and a cross-section side from a side face (its normal) or
      // from the end tag (its x lies along a cross-section side: tag edges follow the puck edges).
      cv::Vec3d side = chosen.front()->along_x;
      for (size_t i = 0; i < chosen.size(); ++i) if (!end[i]) { side = chosen[i]->normal; break; }
      cv::Vec3d v = side - c.axis * side.dot(c.axis);
      if (cv::norm(v) < 1e-6) continue;
      v /= cv::norm(v);
      const cv::Vec3d w = c.axis.cross(v);
      float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
      bool in_front = true;
      for (const double a : {-1.0, 1.0}) for (const double b : {-1.0, 1.0}) for (const double d : {-1.0, 1.0}) {
        const cv::Vec3d p = c.center + c.axis * (a * config_.length_m / 2) + v * (b * config_.width_m / 2) +
          w * (d * config_.width_m / 2);
        if (!(p[2] > 1e-3)) { in_front = false; continue; }
        const float u = float(in.fx * p[0] / p[2] + in.cx), vv = float(in.fy * p[1] / p[2] + in.cy);
        x0 = std::min(x0, u); y0 = std::min(y0, vv); x1 = std::max(x1, u); y1 = std::max(y1, vv);
      }
      if (!in_front) continue;
      const cv::Rect2f projected(x0, y0, x1 - x0, y1 - y0);
      const double inter = (projected & *box).area();
      c.iou = inter / (projected.area() + box->area() - inter);
      if (c.iou < config_.min_box_iou) continue;  // this pose does not fill the detected box
    }
    double worst = 0;
    for (const auto &l : lines) worst = std::max(worst, angle_deg(c.axis, l));
    for (size_t i = 0; i < chosen.size(); ++i) {
      for (size_t j = i + 1; j < chosen.size(); ++j) {
        if (end[i] && end[j]) continue;  // cylinder: both ends, normals antiparallel (axis check covers it)
        worst = std::max(worst, 90.0 - angle_deg(chosen[i]->normal, chosen[j]->normal));  // distinct faces: 90 deg
      }
    }
    // On a shelf a puck stands (axis up) or lies (axis level): breaks the IPPE mirror ambiguity of a
    // lone small tag. It cannot separate a cuboid's end from its side face (both stay on-shelf).
    const double tilt = angle_deg(c.axis, config_.up);
    c.fit = worst;
    c.cost = worst + kReprojectionWeight * extra_px + config_.shelf_prior_weight * std::min(tilt, 90.0 - tilt) +
      config_.center_weight_deg_per_mm * c.spread * 1000.0 + (c.iou >= 0 ? config_.box_iou_weight_deg * (1.0 - c.iou) : 0.0);
    results.push_back(c);
  }
  if (results.empty()) return {};
  const auto best = *std::min_element(results.begin(), results.end(),
    [](const Combination &a, const Combination &b) { return a.cost < b.cost; });
  if (best.fit > config_.max_fit_error_deg) return {};
  for (const auto &other : results) {
    if (other.cost <= best.cost + config_.ambiguity_margin_deg &&
        angle_deg(other.axis, best.axis) > kDistinctAxisDeg) return {};  // e.g. one cuboid tag: end or side
  }

  PuckResult result;
  result.axis = best.axis;
  result.normal = config_.up;
  result.standing = angle_deg(result.axis, config_.up) < config_.standing_max_tilt_deg;
  if (result.standing ? result.axis.dot(config_.up) < 0 : result.axis[0] < 0) result.axis = -result.axis;
  result.center = best.center;
  result.height_m = result.standing ? config_.length_m : config_.width_m;
  result.tags = static_cast<int>(poses.size());
  result.points = result.tags;
  result.fit_error_deg = best.fit;
  result.box_iou = best.iou;
  return result;
}
}  // namespace craft_perception
