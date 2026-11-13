#pragma once
#include "craft_perception/perception_types.hpp"
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace craft_perception {
// Traffic-light mission (발제 04): the light blinks 12 times, 1 s on / 1 s off, one color per
// blink. Counting the color of every ON phase gives (R, Y, G), each 1..6, sum 12.
enum class LightColor { kRed = 0, kYellow = 1, kGreen = 2 };
const char *to_string(LightColor color);

// How the lit color of a frame is decided (perception.yaml uppercam.traffic_light.color_source):
//   yolo:   the YOLO class of the best sign box (sign_red / sign_yellow / sign_green).
//   pixels: YOLO only finds the tower (any sign class, incl. sign_off, score >= locate_min_score);
//           the color is measured in its pixels (port of the perception-v2 package tower_state()):
//           lit = near-white clipped core (V >= bright_v) covering >= on_fraction of the box, else off;
//           color = median hue of the saturated ring right around that core; no usable ring ->
//           position of the core along the tower (top red, middle yellow, bottom green).
//           An unlit tower never counts, whatever color the model sees (2026-11-12: off -> yellow).
enum class ColorSource { kYolo, kPixels };
struct PixelColorConfig {
  int bright_v = 235;            // V of the clipped lamp core
  double on_fraction = 0.03;     // share of the box that must be core to be lit (off towers: < 0.004)
  double ring_fraction = 0.06;   // ring width / tower height
  int ring_min_saturation = 80;
  int ring_min_pixels = 10;
  double green_hue_min = 45, green_hue_max = 100;  // OpenCV hue 0..179
  double red_hue_below = 9.5, red_hue_from = 165;  // red: < red_hue_below or >= red_hue_from
  double yellow_hue_max = 40;                      // yellow: red_hue_below .. yellow_hue_max
  double red_cut = 0.47, yellow_cut = 0.68;        // fallback: core centroid along the tower, 0 = top
  // true: a core above red_cut is red before any hue rule (our upper cam: lit red reads hue 13-15, inside
  // yellow's 11-21, but its core sits at 0.27-0.33 of the tower vs yellow 0.42-0.60; bags 2026-11-11).
  bool red_by_position = false;
  // Top share of the box left out of every measurement: our tower has a white cap that reads as a clipped
  // core even when unlit (3-13 % of the box) and pulls the core position up (2026-11-12). 0 = whole box.
  double cap_fraction = 0.0;
  // false: the color is the core position alone (red above red_cut, yellow above yellow_cut, else green);
  // true: package order (ring hue first, position when no ring).
  bool use_ring_hue = true;
  double upright_ratio = 1.3;    // box taller than this x its width = an upright tower
};
// Lit color of the tower in `box` (pixels method), nullopt = unlit or not measurable.
std::optional<LightColor> measure_light_color(const cv::Mat &bgr, const cv::Rect2f &box,
  const PixelColorConfig &config = {});

struct TrafficLightConfig {
  // YOLO class id of each lit color; detections of other classes are ignored.
  int red_class_id = 0, yellow_class_id = 1, green_class_id = 2;
  float min_score = 0.5f;
  float yellow_min_score = 0.5f;
  // Filter on the 1 s ON phase. A blink is a run of frames with a lit detection; frames with
  // no detection shorter than max_gap_s are bridged (missed detections). The blink is accepted
  // only if it lasted at least min_on_s (rejects false spikes) and its majority color holds at
  // least min_vote_ratio of the lit frames (rejects color flicker).
  double min_on_s = 0.6;
  double max_on_s = 1.6;      // longer = two blinks merged or a stuck light: rejected, warned
  double max_gap_s = 0.4;     // must stay well below the 1 s OFF phase
  double min_vote_ratio = 0.6;
  int expected_blinks = 12;
  // End of the sequence: the light stays dark this long after a blink (rule: 3 s off after the 12th;
  // 2.8 s leaves margin for frame timing). Must exceed the 1 s OFF phase between blinks.
  double end_off_s = 2.8;
  ColorSource color_source = ColorSource::kYolo;
  // pixels: the tower box (best of the sign classes above + off_class_id, score >= locate_min_score).
  int off_class_id = 4;
  float locate_min_score = 0.3f;
  PixelColorConfig pixels;
};

struct BlinkEvent {
  LightColor color;
  double start_s, end_s;
  int frames, votes;  // lit frames in the blink, frames that voted for `color`
};

struct TrafficLightCounts {
  std::array<int, 3> count{0, 0, 0};  // indexed by LightColor
  int red() const { return count[0]; }
  int yellow() const { return count[1]; }
  int green() const { return count[2]; }
  int total() const { return count[0] + count[1] + count[2]; }
};

// Pure, time-driven: feed one call per camera frame with that frame's detections, in stamp
// order. No ROS, no clock of its own, so it replays from bags and tests deterministically.
class TrafficLightCounter {
public:
  bool configure(const TrafficLightConfig &config);
  int expected_blinks() const { return config_.expected_blinks; }
  void reset();
  // Returns the blink this frame closed, if any (the gap after it exceeded max_gap_s).
  // bgr: the frame the detections came from; needed by ColorSource::kPixels (without it no frame is lit).
  std::optional<BlinkEvent> update(double stamp_s, const std::vector<Detection> &detections,
    const cv::Mat &bgr = cv::Mat());
  // Closes a pending blink at end of stream (recorded data that stops right after a blink).
  // Does not end the cycle: only end_off_s of dark frames does.
  std::optional<BlinkEvent> flush();

  const std::vector<BlinkEvent> &blinks() const { return blinks_; }
  const std::vector<std::string> &rejected() const { return rejected_; }  // reasons, for logs
  TrafficLightCounts counts() const;
  // true once a cycle ended (dark for end_off_s) with exactly expected_blinks and each color 1..6.
  // A cycle that ends any other way is discarded and counting starts over on the next cycle.
  bool complete() const { return complete_; }
  int discarded_cycles() const { return discarded_cycles_; }
  const std::string &last_discard() const { return last_discard_; }  // why the last cycle was dropped
  // Mission rule check: total 12 and each color 1..6.
  bool valid() const;

private:
  std::optional<LightColor> frame_color(const std::vector<Detection> &detections, const cv::Mat &bgr) const;
  std::optional<BlinkEvent> close_blink();
  void end_cycle();

  TrafficLightConfig config_;
  bool active_ = false;
  bool complete_ = false;
  bool cycle_open_ = false;  // a lit frame was seen since the last cycle end
  int discarded_cycles_ = 0;
  std::string last_discard_;
  double start_s_ = 0, last_lit_s_ = 0;
  std::array<int, 3> votes_{0, 0, 0};
  std::vector<BlinkEvent> blinks_;
  std::vector<std::string> rejected_;
};

// ---------------------------------------------------------------------------------------------
// Barcode + placement puzzle (발제 v5, 02~04).

// Code 128 next to the light: P2-01 / P2-03 ascending, P2-02 / P2-04 descending (sorts the R/Y/G counts).
enum class SortOrder { kAscending, kDescending };
// nullopt = unregistered code (mission error: stop safely).
std::optional<SortOrder> parse_barcode(const std::string &code);

// Decoding the image is in barcode_reader.hpp (needs zxing-cpp).

enum class PuckColor { kRed, kGreen };
enum class PuckShape { kCuboid, kCylinder };  // 30x30x60 cuboid or Ø30x60 cylinder
struct PuckInfo {
  int id;  // 0..11 (placement.csv); start boxes hold 2 each
  PuckColor color;
  PuckShape shape;
};

// ---- Puck survey (skeleton) -------------------------------------------------------------------
// The robot visits start boxes S1..S6 (2 pucks each, box center X +-40 mm, Y 0) and reads every
// puck's color and shape before solving. Detector: CenterPose model (in training, 2026-11-11);
// this only collects its results. TODO: detector output -> PuckSighting, ROS wiring, re-look policy.
struct PuckSighting {
  int box = 0;          // start box 1..6
  int slot = 0;         // 0 = -X side, 1 = +X side of the box (x -40 / +40 mm)
  PuckColor color = PuckColor::kRed;
  PuckShape shape = PuckShape::kCuboid;
  float score = 0;      // detector confidence
  std::array<double, 3> position{0, 0, 0};  // [m] map frame, for the pick planner (TODO)
  bool shape_confirmed = false;  // shape from the AprilTag rule (associate_pucks), not YOLO alone
};

class PuckSurvey {
public:
  PuckSurvey() { reset(); }
  void reset();
  // Keeps the best-scoring sighting per (box, slot). false = rejected (box/slot out of range).
  bool add(const PuckSighting &sighting);
  // Same, for a puck identified by its AprilTag (id 0..11) instead of its start slot. A sighting whose
  // shape the tags confirmed replaces an unconfirmed one whatever the scores, and is never replaced by one.
  bool add(int puck_id, PuckColor color, PuckShape shape, float score, bool shape_confirmed = false);
  bool complete() const;               // all 12 slots seen
  std::vector<int> missing_ids() const;
  // 12 pucks for solve_traffic_light once complete(); nullopt before.
  std::optional<std::vector<PuckInfo>> pucks() const;
  // Puck id of a start slot. TODO: take the real table from placement.csv; placeholder (box-1)*2+slot.
  static int puck_id(int box, int slot) { return (box - 1) * 2 + slot; }

  // Loading: the robot carries all 12 before solving, so each pick records which carrier slot
  // (0..11, layout TODO) the puck went into. false = unknown puck, bad slot or slot already used.
  bool load(int puck_id, int carrier_slot);
  bool all_loaded() const;
  std::optional<int> carrier_slot(int puck_id) const;  // where to take the puck from when placing
  // Best sighting of one puck so far (nullopt = not seen).
  const std::optional<PuckSighting> &sighting(int puck_id) const { return slots_.at(puck_id); }

private:
  std::array<std::optional<PuckSighting>, 12> slots_;
  std::array<int, 12> carrier_{};  // puck id -> carrier slot, -1 = not loaded
};

// ---- Puck identity from one frame: YOLO box (color + shape) + AprilTag (id) ---------------------
// Port of the perception-v2 package's combine() association (tagged_perception.py), without pose.
// Tags: cuboids carry one on every face, cylinders only on the two end faces, so a cylinder is
// only identified when an end face is visible (standing, seen from above).
struct PuckClassMap {  // YOLO class ids of the puck model (pucks_v8b order)
  int red_cylinder = 0, red_prism = 1, green_cylinder = 2, green_prism = 3;
};
struct PuckTag {
  int id;               // AprilTag id; 0..11 are pucks (12..14 are the locations)
  cv::Point2f center;   // [px]
  std::array<cv::Point2f, 4> corners{};  // image corners (AprilTagDetection order); all zero = unknown
  std::optional<cv::Vec3d> position;     // [m] optical frame, from aligned depth (tag_depth); D435 only
  std::optional<cv::Vec3d> normal;       // unit face normal toward the camera, from depth
};
// A tag on the D435's color-aligned depth (bgr(v,u) and depth(v,u) are the same point, no lookup table):
// center = median depth inside the tag quad deprojected at the quad center; normal = plane fitted to the
// quad's depth points, turned toward the camera; view_angle_deg = angle between normal and line of sight.
struct TagDepth { cv::Vec3d center, normal; double view_angle_deg = 0; int points = 0; };
std::optional<TagDepth> tag_depth(const DepthImage &depth, const std::array<cv::Point2f, 4> &corners,
  const Intrinsics &intrinsics, int min_points = 12);
struct TaggedPuck {
  int id;
  PuckColor color;
  PuckShape shape;
  float score;          // YOLO confidence
  cv::Rect2f box;
  int tags = 0;                  // puck-tag detections inside the box
  std::optional<cv::Vec3d> position;  // [m] optical frame: mean of its tags' depth positions (D435)
  std::vector<std::array<cv::Point2f, 4>> tag_corners;  // its tags' image corners (for PuckTagAxis)
  std::optional<PuckResult> axis;  // filled by the caller: PuckTagAxis on tag_corners + box (puck survey)
  bool shape_from_tags = false;  // shape set by PuckShapeRule, not by the YOLO class
};
// Shape from the tags inside a box, because YOLO scores red prism and red cylinder alike (2026-11-11 bag).
// A patch for that confusion only: applied to red boxes (red_only), green keeps the YOLO shape.
// A cuboid has a tag on all 6 faces, a cylinder only on its 2 end faces, which are never both in view:
//   >= 2 tags in one box                        -> cuboid
//   1 face-on tag, box long/short >= side_face_aspect -> cuboid (its 30x60 side face; a face-on cylinder
//                                                  end or cuboid end is 30x30, a near-square box)
//   anything else                               -> the YOLO shape, shape_from_tags false
// Tags never prove a cylinder: one tag may be a cuboid whose other faces were too oblique to read.
// Face-on: with depth (D435), the tag normal within face_on_max_angle_deg of the line of sight; without
// depth (upper-cam JPEG), from the image corners alone: a near-square quad.
struct PuckShapeRule {
  bool enabled = true;
  bool red_only = true;  // only red boxes (red prism / red cylinder mix-up)
  double face_on_side_ratio = 0.85;  // shortest / longest tag side >= this
  double face_on_skew_deg = 10.0;    // every tag corner within 90 +- this
  double side_face_aspect = 1.6;     // YOLO box long / short side >= this
  double face_on_max_angle_deg = 20.0;  // with depth: tag normal vs line of sight
};
bool tag_face_on(const std::array<cv::Point2f, 4> &corners, const PuckShapeRule &rule);  // image corners
bool tag_face_on(const PuckTag &tag, const PuckShapeRule &rule);  // depth normal if known, else corners
// Each puck tag (id 0..11) goes to the smallest puck box containing its center; a box keeps one
// tag only (a cuboid shows several faces: same id, counted once; two different ids = ambiguous, dropped).
// every_box: also return boxes without a usable id (no tag, or two different ids: id -1) and do not merge boxes
// that share an id, so each YOLO puck box gets its own tags and shape (the axis step works per box, ids aside).
std::vector<TaggedPuck> associate_pucks(const std::vector<Detection> &yolo,
  const std::vector<PuckTag> &tags, const PuckClassMap &classes = {}, float min_score = 0.4f,
  const PuckShapeRule &rule = {}, bool every_box = false);
// The puck boxes associate_pucks considers (puck class, score >= min_score): where to look for tags.
std::vector<cv::Rect2f> puck_boxes(const std::vector<Detection> &yolo, const PuckClassMap &classes, float min_score);

// One location (L1..L3): which signal group it holds and which puck goes on which dice dot.
struct LocationPlan {
  LightColor group;   // R: red pucks, G: green pucks, Y: one shape, any color
  int count = 0;      // dice face = that group's blink count
  std::vector<int> puck_ids;                // in dot order, same size as dots
  std::vector<std::array<double, 2>> dots;  // [m] offsets from the location center, +Y = map up
};
struct PuckPlacement {
  std::array<LocationPlan, 3> locations;  // L1, L2, L3
  PuckShape yellow_shape = PuckShape::kCuboid;
};

// Dice dot offsets [m] for face n (1..6) on the 3x3 grid of pitch 0.06 m: 2 and 3 on the
// top-left/bottom-right diagonal, 6 as the left and right columns (발제 03, no rotation).
std::vector<std::array<double, 2>> dice_dots(int n);

// ---- Re-verification after placing (발제 05 step 7, 채점 "이동 후 검증") --------------------------
// What the camera saw at one location after all pucks were placed (new observation, not the plan).
struct PlacedPuck {
  int id;                         // puck AprilTag id
  std::array<double, 2> offset;   // [m] from the location center, +Y = map up
};
struct LocationCheck {
  bool ok = false;                // ids, count and every dot within tolerance, no extra puck
  std::vector<int> wrong_ids;     // seen here but planned elsewhere (or unknown)
  std::vector<int> missing_ids;   // planned here but not seen
  int empty_dots = 0;             // planned dots with no puck within tolerance
  int extra_pucks = 0;            // pucks not on a planned dot
};
struct PlacementCheck {
  std::array<LocationCheck, 3> locations;  // L1..L3
  std::vector<int> duplicate_ids;          // seen at more than one location / twice
  std::vector<int> unseen_ids;             // 0..11 never seen
  bool ok() const;
};
// Which puck sits on which dot is free; only the group (ids) and the dot pattern count.
// tolerance_m: 발제 03 success = center error within 10 mm.
PlacementCheck verify_placement(const PuckPlacement &plan,
  const std::array<std::vector<PlacedPuck>, 3> &observed, double tolerance_m = 0.010);

// ---- Mission phases (skeleton) ----------------------------------------------------------------
// 1 signal + barcode at the start zone -> 2 load all 12 from S1..S6 (survey color/shape) ->
// 3 solve -> 4 place on L1..L3 -> 5 re-verify by camera -> 6 return (whole body in the start zone, 3 s).
// TODO: the state machine itself lives in the mission node; this is the shared vocabulary.
enum class MissionPhase { kObserveSignal, kLoadPucks, kSolve, kPlacePucks, kVerify, kReturn, kDone, kError };
const char *to_string(MissionPhase phase);

// L1..L3 order: groups sorted by count (barcode direction), ties keep R -> Y -> G.
std::array<LightColor, 3> location_groups(const TrafficLightCounts &counts, SortOrder order);

// Whole assignment before any pick (발제 03: R/G must not use up the Y shape): every leftover red
// (red - R) and green (green - G) goes to Y, so Y needs one shape with at least that many of each
// color. Fixed choices for a deterministic plan: cuboid before cylinder when both work; within a
// group, lowest ids first, and ids that Y needs are taken for Y before R/G.
// nullopt + error = no valid assignment (bad counts, puck list, or no shape fits).
std::optional<PuckPlacement> solve_traffic_light(const TrafficLightCounts &counts, SortOrder order,
  const std::vector<PuckInfo> &pucks, std::string *error = nullptr);
}
