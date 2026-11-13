#include "craft_perception/traffic_light.hpp"
#include "geometry_utils.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cstdio>
#include <opencv2/imgproc.hpp>
#include <string>

namespace craft_perception {
const char *to_string(LightColor color) {
  switch (color) {
    case LightColor::kRed: return "R";
    case LightColor::kYellow: return "Y";
    case LightColor::kGreen: return "G";
  }
  return "?";
}

bool TrafficLightCounter::configure(const TrafficLightConfig &c) {
  const int a = c.red_class_id, b = c.yellow_class_id, g = c.green_class_id;
  if (a < 0 || b < 0 || g < 0 || a == b || a == g || b == g) return false;
  if (c.min_on_s <= 0 || c.max_on_s <= c.min_on_s || c.max_gap_s <= 0) return false;
  if (c.min_vote_ratio <= 0.5 || c.min_vote_ratio > 1.0) return false;  // > 0.5 = unique winner
  if (c.expected_blinks <= 0) return false;
  if (c.end_off_s <= c.max_gap_s) return false;
  config_ = c;
  reset();
  return true;
}

void TrafficLightCounter::reset() {
  active_ = false;
  complete_ = false;
  cycle_open_ = false;
  discarded_cycles_ = 0;
  last_discard_.clear();
  votes_ = {0, 0, 0};
  blinks_.clear();
  rejected_.clear();
}

// The best-scoring lit detection of this frame; one light, so one color per frame.
std::optional<LightColor> measure_light_color(const cv::Mat &bgr, const cv::Rect2f &box,
  const PixelColorConfig &c) {
  if (bgr.empty() || bgr.type() != CV_8UC3) return {};
  const cv::Rect full = cv::Rect(cv::Point(std::lround(box.x), std::lround(box.y)),
    cv::Point(std::lround(box.x + box.width), std::lround(box.y + box.height))) & cv::Rect(0, 0, bgr.cols, bgr.rows);
  if (full.width < 3 || full.height < 6) return {};
  // Below the cap; positions stay measured along the whole box (0 = its top).
  const int skip = std::clamp(static_cast<int>(std::lround(c.cap_fraction * full.height)), 0, full.height - 3);
  const cv::Rect roi(full.x, full.y + skip, full.width, full.height - skip);
  cv::Mat hsv;
  cv::cvtColor(bgr(roi), hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> ch;
  cv::split(hsv, ch);
  const cv::Mat bright = ch[2] >= c.bright_v;
  const int core = cv::countNonZero(bright);
  if (core < c.on_fraction * roi.area()) return {};  // unlit (or too little glow)
  if (full.height < c.upright_ratio * full.width) {
    // Not an upright tower in the image: vote over the saturated glow (package lamp_color()).
    const cv::Mat lit = (ch[2] >= 170) & (ch[1] >= 90);
    if (cv::countNonZero(lit) < 15) return {};
    const cv::Mat halo = (ch[1] >= 150) & (ch[2] >= 100);
    const cv::Mat &use = cv::countNonZero(halo) >= 15 ? halo : lit;
    int votes[3] = {0, 0, 0}, n = 0;
    for (int y = 0; y < use.rows; ++y) {
      for (int x = 0; x < use.cols; ++x) {
        if (!use.at<uchar>(y, x)) continue;
        const int h = ch[0].at<uchar>(y, x);
        ++n;
        if (h <= 10 || h >= 165) ++votes[0];
        else if (h >= 15 && h <= 35) ++votes[1];
        else if (h >= 40 && h <= 95) ++votes[2];
      }
    }
    const int best = static_cast<int>(std::max_element(votes, votes + 3) - votes);
    if (votes[best] < 0.5 * n) return {};
    return static_cast<LightColor>(best);
  }
  // Hue of the ring right around the clipped core.
  const int w = std::max(3, static_cast<int>(std::lround(c.ring_fraction * roi.height))) | 1;
  cv::Mat grown;
  cv::dilate(bright, grown, cv::Mat::ones(w, w, CV_8U));
  const cv::Mat ring = grown & ~bright & (ch[1] >= c.ring_min_saturation);
  std::vector<uchar> hues;
  hues.reserve(cv::countNonZero(ring));
  double core_y = 0;
  for (int y = 0; y < ring.rows; ++y) {
    for (int x = 0; x < ring.cols; ++x) {
      if (ring.at<uchar>(y, x)) hues.push_back(ch[0].at<uchar>(y, x));
      if (bright.at<uchar>(y, x)) core_y += y;
    }
  }
  const double yc = (core_y / core + skip) / full.height;  // 0 = top of the whole box
  if (!c.use_ring_hue) {
    return yc < c.red_cut ? LightColor::kRed : yc < c.yellow_cut ? LightColor::kYellow : LightColor::kGreen;
  }
  if (c.red_by_position && yc < c.red_cut) return LightColor::kRed;
  if (static_cast<int>(hues.size()) >= c.ring_min_pixels) {
    auto mid = hues.begin() + hues.size() / 2;
    std::nth_element(hues.begin(), mid, hues.end());
    double hue = *mid;
    if (hues.size() % 2 == 0) {  // numpy median: mean of the two middle values
      hue = (hue + *std::max_element(hues.begin(), mid)) / 2.0;
    }
    if (hue >= c.green_hue_min && hue <= c.green_hue_max) return LightColor::kGreen;
    if (hue < c.red_hue_below || hue >= c.red_hue_from) return LightColor::kRed;
    if (hue <= c.yellow_hue_max) return LightColor::kYellow;
  }
  return yc < c.red_cut ? LightColor::kRed : yc < c.yellow_cut ? LightColor::kYellow : LightColor::kGreen;
}

std::optional<LightColor> TrafficLightCounter::frame_color(
  const std::vector<Detection> &detections, const cv::Mat &bgr) const {
  if (config_.color_source == ColorSource::kPixels) {
    // The tower: the best box of any sign class; its color comes from the pixels, not the class.
    const Detection *tower = nullptr;
    for (const auto &d : detections) {
      const bool sign = d.class_id == config_.red_class_id || d.class_id == config_.yellow_class_id ||
        d.class_id == config_.green_class_id || d.class_id == config_.off_class_id;
      if (sign && d.score >= config_.locate_min_score && (!tower || d.score > tower->score)) tower = &d;
    }
    if (!tower) return {};
    return measure_light_color(bgr, tower->box, config_.pixels);
  }
  const Detection *best = nullptr;
  for (const auto &d : detections) {
    const float min_score = d.class_id == config_.yellow_class_id
      ? config_.yellow_min_score : config_.min_score;
    if (d.score < min_score) continue;
    if (d.class_id != config_.red_class_id && d.class_id != config_.yellow_class_id &&
        d.class_id != config_.green_class_id) continue;
    if (!best || d.score > best->score) best = &d;
  }
  if (!best) return {};
  if (best->class_id == config_.red_class_id) return LightColor::kRed;
  if (best->class_id == config_.yellow_class_id) return LightColor::kYellow;
  return LightColor::kGreen;
}

std::optional<BlinkEvent> TrafficLightCounter::update(double stamp_s,
  const std::vector<Detection> &detections, const cv::Mat &bgr) {
  if (complete()) return {};
  std::optional<BlinkEvent> closed;
  if (active_ && stamp_s - last_lit_s_ > config_.max_gap_s) closed = close_blink();
  if (const auto color = frame_color(detections, bgr)) {
    if (!active_) {
      active_ = true;
      start_s_ = stamp_s;
      votes_ = {0, 0, 0};
    }
    cycle_open_ = true;
    last_lit_s_ = stamp_s;
    ++votes_[static_cast<int>(*color)];
  } else if (cycle_open_ && !active_ && stamp_s - last_lit_s_ >= config_.end_off_s) {
    end_cycle();
  }
  return closed;
}

// The light went dark for end_off_s: accept the cycle, or drop it and wait for the next one.
void TrafficLightCounter::end_cycle() {
  cycle_open_ = false;
  if (valid()) {
    complete_ = true;
    return;
  }
  const auto c = counts();
  char reason[120];
  std::snprintf(reason, sizeof(reason), "cycle ended with %d blinks (R%d Y%d G%d), need %d with each 1..6",
    c.total(), c.red(), c.yellow(), c.green(), config_.expected_blinks);
  last_discard_ = reason;
  ++discarded_cycles_;
  blinks_.clear();
  rejected_.clear();
}

std::optional<BlinkEvent> TrafficLightCounter::flush() {
  if (!active_) return {};
  return close_blink();
}

std::optional<BlinkEvent> TrafficLightCounter::close_blink() {
  active_ = false;
  const int frames = votes_[0] + votes_[1] + votes_[2];
  const int winner = static_cast<int>(std::max_element(votes_.begin(), votes_.end()) - votes_.begin());
  const double duration = last_lit_s_ - start_s_;
  char reason[160];
  if (duration < config_.min_on_s) {
    std::snprintf(reason, sizeof(reason), "t=%.2f short blink %.2fs (%d frames)", start_s_,
      duration, frames);
    rejected_.emplace_back(reason);
    return {};
  }
  if (duration > config_.max_on_s) {
    std::snprintf(reason, sizeof(reason), "t=%.2f long blink %.2fs (merged blinks?)", start_s_,
      duration);
    rejected_.emplace_back(reason);
    return {};
  }
  if (votes_[winner] < config_.min_vote_ratio * frames) {
    std::snprintf(reason, sizeof(reason), "t=%.2f ambiguous color R%d Y%d G%d", start_s_,
      votes_[0], votes_[1], votes_[2]);
    rejected_.emplace_back(reason);
    return {};
  }
  BlinkEvent event{static_cast<LightColor>(winner), start_s_, last_lit_s_, frames, votes_[winner]};
  blinks_.push_back(event);
  return event;
}

TrafficLightCounts TrafficLightCounter::counts() const {
  TrafficLightCounts out;
  for (const auto &b : blinks_) ++out.count[static_cast<int>(b.color)];
  return out;
}

bool TrafficLightCounter::valid() const {
  const auto c = counts();
  if (c.total() != config_.expected_blinks) return false;
  return std::all_of(c.count.begin(), c.count.end(), [](int n) { return n >= 1 && n <= 6; });
}

// ---------------------------------------------------------------------------------------------
std::optional<SortOrder> parse_barcode(const std::string &code) {
  if (code == "P2-01" || code == "P2-03") return SortOrder::kAscending;
  if (code == "P2-02" || code == "P2-04") return SortOrder::kDescending;
  return {};
}

void PuckSurvey::reset() {
  slots_.fill(std::nullopt);
  carrier_.fill(-1);
}

bool PuckSurvey::load(int puck_id, int carrier_slot) {
  if (puck_id < 0 || puck_id >= 12 || carrier_slot < 0 || carrier_slot >= 12) return false;
  if (!slots_[puck_id] || carrier_[puck_id] >= 0) return false;  // not surveyed, or loaded twice
  if (std::find(carrier_.begin(), carrier_.end(), carrier_slot) != carrier_.end()) return false;
  carrier_[puck_id] = carrier_slot;
  return true;
}

bool PuckSurvey::all_loaded() const {
  return std::all_of(carrier_.begin(), carrier_.end(), [](int slot) { return slot >= 0; });
}

std::optional<int> PuckSurvey::carrier_slot(int puck_id) const {
  if (puck_id < 0 || puck_id >= 12 || carrier_[puck_id] < 0) return {};
  return carrier_[puck_id];
}

bool PlacementCheck::ok() const {
  return duplicate_ids.empty() && unseen_ids.empty() &&
    std::all_of(locations.begin(), locations.end(), [](const LocationCheck &l) { return l.ok; });
}

PlacementCheck verify_placement(const PuckPlacement &plan,
  const std::array<std::vector<PlacedPuck>, 3> &observed, double tolerance_m) {
  PlacementCheck check;
  std::array<int, 12> seen{};
  for (int l = 0; l < 3; ++l) {
    const auto &want = plan.locations[l];
    auto &result = check.locations[l];
    std::vector<bool> dot_used(want.dots.size(), false);
    std::vector<int> here;
    for (const auto &puck : observed[l]) {
      if (puck.id >= 0 && puck.id < 12) ++seen[puck.id];
      here.push_back(puck.id);
      if (std::find(want.puck_ids.begin(), want.puck_ids.end(), puck.id) == want.puck_ids.end()) {
        result.wrong_ids.push_back(puck.id);
      }
      // Nearest free dot within tolerance; dots are 60 mm apart, so at most one can match.
      bool on_dot = false;
      for (std::size_t d = 0; d < want.dots.size() && !on_dot; ++d) {
        const double dx = puck.offset[0] - want.dots[d][0], dy = puck.offset[1] - want.dots[d][1];
        if (!dot_used[d] && dx * dx + dy * dy <= tolerance_m * tolerance_m) dot_used[d] = on_dot = true;
      }
      if (!on_dot) ++result.extra_pucks;
    }
    for (const int id : want.puck_ids) {
      if (std::find(here.begin(), here.end(), id) == here.end()) result.missing_ids.push_back(id);
    }
    result.empty_dots = static_cast<int>(std::count(dot_used.begin(), dot_used.end(), false));
    result.ok = result.wrong_ids.empty() && result.missing_ids.empty() && result.empty_dots == 0 &&
      result.extra_pucks == 0;
  }
  for (int id = 0; id < 12; ++id) {
    if (seen[id] == 0) check.unseen_ids.push_back(id);
    if (seen[id] > 1) check.duplicate_ids.push_back(id);
  }
  return check;
}

const char *to_string(MissionPhase phase) {
  switch (phase) {
    case MissionPhase::kObserveSignal: return "observe_signal";
    case MissionPhase::kLoadPucks: return "load_pucks";
    case MissionPhase::kSolve: return "solve";
    case MissionPhase::kPlacePucks: return "place_pucks";
    case MissionPhase::kVerify: return "verify";
    case MissionPhase::kReturn: return "return";
    case MissionPhase::kDone: return "done";
    case MissionPhase::kError: return "error";
  }
  return "?";
}

bool PuckSurvey::add(const PuckSighting &s) {
  if (s.box < 1 || s.box > 6 || s.slot < 0 || s.slot > 1) return false;
  auto &slot = slots_[puck_id(s.box, s.slot)];
  if (!slot || s.score > slot->score) slot = s;  // TODO: vote over several looks instead of best score?
  return true;
}

bool PuckSurvey::add(int puck_id, PuckColor color, PuckShape shape, float score, bool shape_confirmed) {
  if (puck_id < 0 || puck_id >= 12) return false;
  auto &slot = slots_[puck_id];
  const bool better = !slot || (shape_confirmed && !slot->shape_confirmed) ||
    (shape_confirmed == slot->shape_confirmed && score > slot->score);
  if (better) {
    PuckSighting sighting;
    sighting.box = puck_id / 2 + 1;  // placeholder slot, unused for tagged pucks
    sighting.slot = puck_id % 2;
    sighting.color = color;
    sighting.shape = shape;
    sighting.score = score;
    sighting.shape_confirmed = shape_confirmed;
    slot = sighting;
  }
  return true;
}

bool tag_face_on(const std::array<cv::Point2f, 4> &c, const PuckShapeRule &rule) {
  double shortest = 1e18, longest = 0;
  for (int i = 0; i < 4; ++i) {
    const double side = cv::norm(c[(i + 1) % 4] - c[i]);
    shortest = std::min(shortest, side);
    longest = std::max(longest, side);
  }
  if (!(longest > 1e-6) || shortest / longest < rule.face_on_side_ratio) return false;
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f a = c[(i + 3) % 4] - c[i], b = c[(i + 1) % 4] - c[i];
    const double cosine = a.dot(b) / (cv::norm(a) * cv::norm(b));
    const double angle = std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / CV_PI;
    if (std::abs(angle - 90.0) > rule.face_on_skew_deg) return false;
  }
  return true;
}

bool tag_face_on(const PuckTag &tag, const PuckShapeRule &rule) {
  if (tag.normal && tag.position && cv::norm(*tag.position) > 1e-9) {
    const double cosine = std::abs(tag.normal->dot(*tag.position)) / (cv::norm(*tag.normal) * cv::norm(*tag.position));
    return std::acos(std::clamp(cosine, -1.0, 1.0)) * 180.0 / CV_PI <= rule.face_on_max_angle_deg;
  }
  return tag_face_on(tag.corners, rule);
}

std::optional<TagDepth> tag_depth(const DepthImage &depth, const std::array<cv::Point2f, 4> &corners,
  const Intrinsics &intrinsics, int min_points) {
  if (!depth.is_valid() || !detail::is_valid_intrinsics(intrinsics) || min_points < 3) return {};
  std::vector<cv::Point> quad;
  for (const auto &p : corners) quad.emplace_back(cvRound(p.x), cvRound(p.y));
  const cv::Rect roi = cv::boundingRect(quad) & cv::Rect(0, 0, depth.raw.cols, depth.raw.rows);
  if (roi.area() <= 0) return {};
  cv::Mat inside(roi.size(), CV_8U, cv::Scalar(0));
  std::vector<cv::Point> local;
  for (const auto &p : quad) local.push_back(p - roi.tl());
  cv::fillConvexPoly(inside, local, cv::Scalar(255));
  std::vector<double> zs;
  std::vector<cv::Vec3d> points;
  for (int y = 0; y < roi.height; ++y) {
    for (int x = 0; x < roi.width; ++x) {
      if (!inside.at<uint8_t>(y, x)) continue;
      const double z = depth.at(roi.y + y, roi.x + x);
      if (!std::isfinite(z) || z <= 0) continue;
      zs.push_back(z);
      points.push_back(detail::deproject(roi.x + x, roi.y + y, z, intrinsics));
    }
  }
  if (static_cast<int>(points.size()) < min_points) return {};
  auto mid = zs.begin() + zs.size() / 2;
  std::nth_element(zs.begin(), mid, zs.end());
  cv::Point2f c(0, 0);
  for (const auto &p : corners) c += p * 0.25f;
  TagDepth out;
  out.center = detail::deproject(c.x, c.y, *mid, intrinsics);
  out.points = static_cast<int>(points.size());
  // Plane through the points within 2 cm of the median (drops edge pixels from the background).
  cv::Vec3d mean(0, 0, 0);
  int used = 0;
  for (const auto &p : points) if (std::abs(p[2] - *mid) < 0.02) { mean += p; ++used; }
  if (used < min_points) return {};
  mean /= used;
  cv::Matx33d cov = cv::Matx33d::zeros();
  for (const auto &p : points) {
    if (std::abs(p[2] - *mid) >= 0.02) continue;
    const cv::Vec3d d = p - mean;
    cov += d * d.t();
  }
  cv::Mat values, vectors;
  cv::eigen(cv::Mat(cov), values, vectors);  // descending: last row = smallest = plane normal
  cv::Vec3d n(vectors.at<double>(2, 0), vectors.at<double>(2, 1), vectors.at<double>(2, 2));
  n /= cv::norm(n);
  if (n.dot(mean) > 0) n = -n;  // toward the camera at the origin
  out.normal = n;
  out.view_angle_deg = std::acos(std::clamp(-n.dot(mean) / cv::norm(mean), -1.0, 1.0)) * 180.0 / CV_PI;
  return out;
}

namespace {
bool is_puck_class(int k, const PuckClassMap &classes) {
  return k == classes.red_cylinder || k == classes.red_prism || k == classes.green_cylinder || k == classes.green_prism;
}
}  // namespace

std::vector<cv::Rect2f> puck_boxes(const std::vector<Detection> &yolo, const PuckClassMap &classes, float min_score) {
  std::vector<cv::Rect2f> out;
  for (const auto &d : yolo) {
    if (is_puck_class(d.class_id, classes) && d.score >= min_score) out.push_back(d.box);
  }
  return out;
}

std::vector<TaggedPuck> associate_pucks(const std::vector<Detection> &yolo,
  const std::vector<PuckTag> &tags, const PuckClassMap &classes, float min_score, const PuckShapeRule &rule, bool every_box) {
  struct Candidate { const Detection *box; int id = -1; bool ambiguous = false; std::vector<const PuckTag *> tags; };
  std::vector<Candidate> candidates;
  for (const auto &d : yolo) {
    if (is_puck_class(d.class_id, classes) && d.score >= min_score) candidates.push_back({&d});
  }
  for (const auto &tag : tags) {
    if (tag.id < 0 || tag.id >= 12) continue;
    Candidate *best = nullptr;
    for (auto &c : candidates) {
      if (c.box->box.contains(tag.center) && (!best || c.box->box.area() < best->box->box.area())) best = &c;
    }
    if (!best) continue;
    if (best->id >= 0 && best->id != tag.id) best->ambiguous = true;
    best->id = tag.id;
    best->tags.push_back(&tag);
  }
  std::vector<TaggedPuck> out;
  for (const auto &c : candidates) {
    if (!every_box && (c.id < 0 || c.ambiguous)) continue;
    const int k = c.box->class_id;
    const bool red = k == classes.red_cylinder || k == classes.red_prism;
    PuckShape shape = k == classes.red_cylinder || k == classes.green_cylinder ? PuckShape::kCylinder : PuckShape::kCuboid;
    bool from_tags = false;
    if (rule.enabled && (red || !rule.red_only)) {
      const auto &b = c.box->box;
      const double aspect = std::max(b.width, b.height) / std::max(1e-6f, std::min(b.width, b.height));
      if (c.tags.size() >= 2 ||
          (c.tags.size() == 1 && aspect >= rule.side_face_aspect && tag_face_on(*c.tags.front(), rule))) {
        shape = PuckShape::kCuboid;
        from_tags = true;
      }
    }
    std::optional<cv::Vec3d> position;
    int located = 0;
    cv::Vec3d sum(0, 0, 0);
    for (const auto *t : c.tags) if (t->position) { sum += *t->position; ++located; }
    if (located) position = sum / located;
    std::vector<std::array<cv::Point2f, 4>> corners;
    for (const auto *t : c.tags) corners.push_back(t->corners);
    out.push_back({c.ambiguous ? -1 : c.id, red ? PuckColor::kRed : PuckColor::kGreen, shape, c.box->score, c.box->box,
                   static_cast<int>(c.tags.size()), position, corners, std::nullopt, from_tags});
  }
  if (every_box) return out;
  // The same id on two boxes (a tag misread, or one puck split in two): keep the confirmed, then better box.
  std::sort(out.begin(), out.end(), [](const TaggedPuck &a, const TaggedPuck &b) {
    if (a.id != b.id) return a.id < b.id;
    if (a.shape_from_tags != b.shape_from_tags) return a.shape_from_tags;
    return a.score > b.score;
  });
  out.erase(std::unique(out.begin(), out.end(), [](const TaggedPuck &a, const TaggedPuck &b) { return a.id == b.id; }),
    out.end());
  return out;
}

bool PuckSurvey::complete() const {
  return std::all_of(slots_.begin(), slots_.end(), [](const auto &s) { return s.has_value(); });
}

std::vector<int> PuckSurvey::missing_ids() const {
  std::vector<int> out;
  for (int i = 0; i < 12; ++i) if (!slots_[i]) out.push_back(i);
  return out;
}

std::optional<std::vector<PuckInfo>> PuckSurvey::pucks() const {
  if (!complete()) return {};
  std::vector<PuckInfo> out;
  for (int i = 0; i < 12; ++i) out.push_back({i, slots_[i]->color, slots_[i]->shape});
  return out;
}

std::vector<std::array<double, 2>> dice_dots(int n) {
  constexpr double d = 0.06;
  const std::array<double, 2> c{0, 0}, tl{-d, d}, tr{d, d}, bl{-d, -d}, br{d, -d}, ml{-d, 0}, mr{d, 0};
  switch (n) {
    case 1: return {c};
    case 2: return {tl, br};
    case 3: return {tl, c, br};
    case 4: return {tl, tr, bl, br};
    case 5: return {tl, tr, c, bl, br};
    case 6: return {tl, ml, bl, tr, mr, br};
  }
  return {};
}

std::array<LightColor, 3> location_groups(const TrafficLightCounts &counts, SortOrder order) {
  std::array<LightColor, 3> groups{LightColor::kRed, LightColor::kYellow, LightColor::kGreen};
  std::stable_sort(groups.begin(), groups.end(), [&](LightColor a, LightColor b) {
    const int ca = counts.count[static_cast<int>(a)], cb = counts.count[static_cast<int>(b)];
    return order == SortOrder::kAscending ? ca < cb : ca > cb;
  });
  return groups;
}

std::optional<PuckPlacement> solve_traffic_light(const TrafficLightCounts &counts, SortOrder order,
  const std::vector<PuckInfo> &input, std::string *error) {
  const auto fail = [&](const std::string &why) -> std::optional<PuckPlacement> {
    if (error) *error = why;
    return {};
  };
  for (const int n : counts.count) {
    if (n < 1 || n > 6) return fail("each of R/Y/G must be 1..6");
  }
  if (counts.total() != 12) return fail("R+Y+G must be 12");
  if (input.size() != 12) return fail("need exactly 12 pucks");
  auto pucks = input;
  std::sort(pucks.begin(), pucks.end(), [](const PuckInfo &a, const PuckInfo &b) { return a.id < b.id; });
  for (std::size_t i = 1; i < pucks.size(); ++i) {
    if (pucks[i].id == pucks[i - 1].id) return fail("duplicate puck id " + std::to_string(pucks[i].id));
  }
  const auto count_of = [&](PuckColor color, std::optional<PuckShape> shape) {
    return static_cast<int>(std::count_if(pucks.begin(), pucks.end(), [&](const PuckInfo &p) {
      return p.color == color && (!shape || p.shape == *shape);
    }));
  };
  const int y_red = count_of(PuckColor::kRed, {}) - counts.red();
  const int y_green = count_of(PuckColor::kGreen, {}) - counts.green();
  if (y_red < 0) return fail("fewer red pucks than R");
  if (y_green < 0) return fail("fewer green pucks than G");
  // y_red + y_green == Y holds because pucks and blinks both total 12.
  std::optional<PuckShape> shape;
  for (const auto s : {PuckShape::kCuboid, PuckShape::kCylinder}) {
    if (count_of(PuckColor::kRed, s) >= y_red && count_of(PuckColor::kGreen, s) >= y_green) {
      shape = s;
      break;
    }
  }
  if (!shape) return fail("no shape has " + std::to_string(y_red) + " red + " + std::to_string(y_green) +
    " green left over for Y");

  std::array<std::vector<int>, 3> group_ids;  // indexed by LightColor
  int need_red = y_red, need_green = y_green;
  for (const auto &p : pucks) {  // Y first: lowest ids of the chosen shape
    int &need = p.color == PuckColor::kRed ? need_red : need_green;
    if (p.shape == *shape && need > 0) {
      group_ids[static_cast<int>(LightColor::kYellow)].push_back(p.id);
      --need;
    } else {
      group_ids[static_cast<int>(p.color == PuckColor::kRed ? LightColor::kRed : LightColor::kGreen)].push_back(p.id);
    }
  }

  PuckPlacement plan;
  plan.yellow_shape = *shape;
  const auto groups = location_groups(counts, order);
  for (int l = 0; l < 3; ++l) {
    auto &location = plan.locations[l];
    location.group = groups[l];
    location.count = counts.count[static_cast<int>(groups[l])];
    location.puck_ids = group_ids[static_cast<int>(groups[l])];
    location.dots = dice_dots(location.count);
    if (location.puck_ids.size() != location.dots.size()) return fail("internal: group size mismatch");
  }
  return plan;
}
}
