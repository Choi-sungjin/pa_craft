#include "uppercam_geometry.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>

namespace uppercam
{
namespace
{

using Mask = cv::Mat;  // CV_8U, 0 / 1

// RANSAC gates: uppercam_box_range.RANSAC with edge_drop 0.020, as SquareFar20Estimator passes it.
constexpr double kBand = 0.030, kInlier = 0.005, kTiltDeg = 8.0, kHeightTol = 0.030, kAreaLo = 0.4, kAreaHi = 1.4;
constexpr double kEdgeDrop = 0.020;
constexpr int kIters = 500, kSample = 4000;

const cv::Mat & cross()
{
  static const cv::Mat k = cv::getStructuringElement(cv::MORPH_CROSS, {3, 3});
  return k;
}

Mask morph(const Mask & m, int op, int iterations)
{
  Mask out;
  cv::morphologyEx(m, out, op, cross(), {-1, -1}, iterations, cv::BORDER_CONSTANT, cv::Scalar(0));
  return out;
}
Mask erode(const Mask & m, int iterations)
{
  Mask out;
  cv::erode(m, out, cross(), {-1, -1}, iterations, cv::BORDER_CONSTANT, cv::Scalar(0));
  return out;
}
Mask dilate(const Mask & m, int iterations)
{
  Mask out;
  cv::dilate(m, out, cross(), {-1, -1}, iterations, cv::BORDER_CONSTANT, cv::Scalar(0));
  return out;
}

// Holes = background not 4-connected to the outside.
Mask fill_holes(const Mask & m)
{
  Mask padded;
  cv::copyMakeBorder(m, padded, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
  cv::floodFill(padded, cv::Point(0, 0), cv::Scalar(2), nullptr, cv::Scalar(), cv::Scalar(), 4);
  Mask out = padded(cv::Rect(1, 1, m.cols, m.rows)) != 2;
  return out / 255;
}

double percentile(std::vector<double> v, double p)  // numpy linear
{
  std::sort(v.begin(), v.end());
  const double pos = p / 100.0 * (v.size() - 1);
  const auto i = static_cast<std::size_t>(std::floor(pos));
  if (i + 1 >= v.size()) return v.back();
  return v[i] + (pos - i) * (v[i + 1] - v[i]);
}

Eigen::Matrix3d rpy(double r, double p, double y)
{
  return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Vector3d forward(const Eigen::Vector3d & up)
{
  Eigen::Vector3d f = Eigen::Vector3d::UnitZ() - up.z() * up;
  return f.normalized();
}

struct Pixels  // depth lifted to camera points
{
  cv::Mat P;      // CV_64FC3
  Mask valid;     // d > 0
};

Pixels lift(const cv::Mat & d, const Eigen::Matrix3d & K)
{
  const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
  Pixels px{cv::Mat(d.size(), CV_64FC3), Mask::zeros(d.size(), CV_8U)};
  for (int v = 0; v < d.rows; ++v) {
    const double * z = d.ptr<double>(v);
    auto * P = px.P.ptr<cv::Vec3d>(v);
    auto * ok = px.valid.ptr<uchar>(v);
    for (int u = 0; u < d.cols; ++u) {
      P[u] = {(u - cx) * z[u] / fx, (v - cy) * z[u] / fy, z[u]};
      ok[u] = z[u] > 0;
    }
  }
  return px;
}

inline Eigen::Vector3d at(const cv::Mat & P, int r, int c)
{
  const auto & p = P.at<cv::Vec3d>(r, c);
  return {p[0], p[1], p[2]};
}

// Plane through points: centroid and the normal of least spread.
void plane(const std::vector<Eigen::Vector3d> & X, Eigen::Vector3d & cen, Eigen::Vector3d & n)
{
  cen.setZero();
  for (const auto & x : X) cen += x;
  cen /= static_cast<double>(X.size());
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (const auto & x : X) C += (x - cen) * (x - cen).transpose();
  n = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(C).eigenvectors().col(0);
}

struct Top
{
  Mask top;  // full image
  Eigen::Vector3d cen, n;
  double area{}, rms{};
};

// Bounding box of the set pixels of m (offset by org) grown by margin, clipped to bounds.
cv::Rect window(const Mask & m, cv::Point org, int margin, const cv::Rect & bounds)
{
  cv::Rect bb = cv::boundingRect(m);
  bb.x += org.x - margin; bb.y += org.y - margin; bb.width += 2 * margin; bb.height += 2 * margin;
  return bb & bounds;
}

// box_top / box_top_cropped (same output): height-aware RANSAC for the box top, blob, plane refit, edge refine.
// Like box_top_cropped, the morphology runs on windows wider than its reach, so the result is the whole-image one.
Top box_top(const cv::Mat & d, const Eigen::Matrix3d & K, const Eigen::Vector3d & up)
{
  const int H = d.rows, W = d.cols, r0 = static_cast<int>(H * 0.5);  // lower half only
  const cv::Rect roi(0, r0, W, H - r0);
  const double fx = K(0, 0), fy = K(1, 1);
  Pixels px = lift(d, K);
  px.valid.rowRange(0, r0).setTo(0);
  std::vector<cv::Point> idx;
  std::vector<Eigen::Vector3d> pts;
  for (int r = r0; r < H; ++r)
    for (int c = 0; c < W; ++c) {
      if (!px.valid.at<uchar>(r, c)) continue;
      const Eigen::Vector3d p = at(px.P, r, c);
      if (std::abs(kCamHeight + p.dot(up) - kBoxHeight) < kBand) { idx.emplace_back(c, r); pts.push_back(p); }
    }
  if (idx.size() < 300) throw std::runtime_error("no points near the box-top height (check --cam-height / --cam-pitch)");

  // Hypotheses scored on a sample of the candidates; only the best six get the blob check.
  std::mt19937 rng(0);
  std::vector<int> order(pts.size());
  std::iota(order.begin(), order.end(), 0);
  const int ns = std::min<int>(kSample, pts.size());
  for (int i = 0; i < ns; ++i) std::swap(order[i], order[i + std::uniform_int_distribution<int>(0, pts.size() - 1 - i)(rng)]);
  std::vector<Eigen::Vector3d> samp(ns);
  for (int i = 0; i < ns; ++i) samp[i] = pts[order[i]];

  struct Hyp { int score; Eigen::Vector3d a, n; };
  std::vector<Hyp> hyps;
  const double cos_tilt = std::cos(kTiltDeg * M_PI / 180.0);
  std::uniform_int_distribution<int> pick(0, ns - 1);
  for (int it = 0; it < kIters; ++it) {
    int i = pick(rng), j, k;
    do j = pick(rng); while (j == i);
    do k = pick(rng); while (k == i || k == j);
    const Eigen::Vector3d a = samp[i];
    Eigen::Vector3d n = (samp[j] - a).cross(samp[k] - a);
    const double len = n.norm();
    if (len < 1e-9) continue;
    n /= len;
    if (n.dot(up) < 0) n = -n;
    if (n.dot(up) < cos_tilt || std::abs(kCamHeight + a.dot(up) - kBoxHeight) > kHeightTol) continue;
    const double off = a.dot(n);
    int score = 0;
    for (const auto & s : samp) score += std::abs(s.dot(n) - off) < kInlier;
    hyps.push_back({score, a, n});
  }
  std::stable_sort(hyps.begin(), hyps.end(), [](const Hyp & x, const Hyp & y) { return x.score > y.score; });

  int best_size = 0;
  double best_area = 0;
  Mask best_blob;      // in best_rect
  cv::Rect best_rect;
  for (std::size_t h = 0; h < std::min<std::size_t>(6, hyps.size()); ++h) {
    const auto & hy = hyps[h];
    std::vector<cv::Point> inl;
    for (std::size_t i = 0; i < pts.size(); ++i) if (std::abs((pts[i] - hy.a).dot(hy.n)) < kInlier) inl.push_back(idx[i]);
    if (inl.empty()) continue;
    cv::Rect w = cv::boundingRect(inl);
    w = cv::Rect(w.x - 4, w.y - 4, w.width + 8, w.height + 8) & roi;  // open (2 x cross) stays inside the inliers
    Mask on = Mask::zeros(w.size(), CV_8U);
    for (const auto & p : inl) on.at<uchar>(p - w.tl()) = 1;
    cv::Mat labels, stats, centroids;
    const int count = cv::connectedComponentsWithStats(morph(on, cv::MORPH_OPEN, 2), labels, stats, centroids, 4, CV_32S,
                                                       cv::CCL_SAUF) - 1;
    if (count <= 0) continue;
    int id = 1;
    for (int l = 2; l <= count; ++l) if (stats.at<int>(l, cv::CC_STAT_AREA) > stats.at<int>(id, cv::CC_STAT_AREA)) id = l;
    Mask blob = labels == id;
    blob /= 255;
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    double area_px = 0;
    for (int r = 0; r < w.height; ++r)
      for (int c = 0; c < w.width; ++c)
        if (blob.at<uchar>(r, c)) {
          mean += at(px.P, r + w.y, c + w.x);
          const double z = d.at<double>(r + w.y, c + w.x);
          area_px += (z / fx) * (z / fy);
        }
    const int size = stats.at<int>(id, cv::CC_STAT_AREA);
    mean /= size;
    const double cosine = std::max(std::abs(hy.n.dot(mean / mean.norm())), 0.2);
    const double area = area_px / cosine;
    if (kAreaLo * kSide * kSide < area && area < kAreaHi * kSide * kSide && (best_blob.empty() || size > best_size)) {
      best_size = size; best_area = area; best_blob = blob; best_rect = w;
    }
  }
  if (best_blob.empty()) throw std::runtime_error("no level blob of box-top size at box-top height");

  Top out;
  out.area = best_area;
  out.top = Mask::zeros(d.size(), CV_8U);
  {
    const cv::Rect w = window(best_blob, best_rect.tl(), 8, roi);  // close (3 x cross) + hole fill
    Mask blob = Mask::zeros(w.size(), CV_8U);
    const cv::Rect both = best_rect & w;  // every blob pixel is inside w
    best_blob(both - best_rect.tl()).copyTo(blob(both - w.tl()));
    fill_holes(morph(blob, cv::MORPH_CLOSE, 3)).copyTo(out.top(w));
  }
  // Plane refit on the inner pixels (erode 8 x cross), outliers trimmed.
  const cv::Rect w = window(out.top, {0, 0}, 20, roi);  // reach of erode 8, dilate 15 + close 2
  const Mask top_w = out.top(w).clone(), valid_w = px.valid(w);
  const Mask inner = erode(top_w, 8) & valid_w;
  std::vector<Eigen::Vector3d> X;
  for (int r = 0; r < w.height; ++r)
    for (int c = 0; c < w.width; ++c)
      if (inner.at<uchar>(r, c)) X.push_back(at(px.P, r + w.y, c + w.x));
  if (X.size() < 3) throw std::runtime_error("box top too small for a plane refit");
  auto spread = [&](std::vector<double> & res) {
    double m = 0, s2 = 0;
    res.resize(X.size());
    for (std::size_t i = 0; i < X.size(); ++i) { res[i] = std::abs((X[i] - out.cen).dot(out.n)); m += res[i]; }
    m /= X.size();
    for (double r : res) s2 += (r - m) * (r - m);
    return std::sqrt(s2 / X.size());
  };
  plane(X, out.cen, out.n);
  std::vector<double> res;
  for (int it = 0; it < 3; ++it) {
    const double lim = 3 * spread(res) + 1e-4;
    std::vector<Eigen::Vector3d> kept;
    for (std::size_t i = 0; i < X.size(); ++i) if (res[i] < lim) kept.push_back(X[i]);
    X.swap(kept);
    if (X.size() < 3) throw std::runtime_error("box top too small for a plane refit");
    plane(X, out.cen, out.n);
  }
  if (out.n.dot(-out.cen) < 0) out.n = -out.n;
  out.rms = spread(res);
  // Refine: edges at the depth drop (edge_drop below the top, not 20 mm above it), around the blob, connected to it.
  const Mask near = dilate(top_w, 15);
  Mask on = Mask::zeros(w.size(), CV_8U);
  for (int r = 0; r < w.height; ++r)
    for (int c = 0; c < w.width; ++c) {
      if (!valid_w.at<uchar>(r, c) || !near.at<uchar>(r, c)) continue;
      const double h = (at(px.P, r + w.y, c + w.x) - out.cen).dot(out.n);
      on.at<uchar>(r, c) = h > -kEdgeDrop && h < 0.02;
    }
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(on, labels, stats, centroids, 4, CV_32S, cv::CCL_SAUF) - 1;
  std::vector<char> in_top(count + 1, 0);
  for (int r = 0; r < w.height; ++r)
    for (int c = 0; c < w.width; ++c)
      if (top_w.at<uchar>(r, c) && labels.at<int>(r, c) > 0) in_top[labels.at<int>(r, c)] = 1;
  int pick_id = 0;  // label inside the top with the most pixels (first on ties)
  for (int i = 1; i <= count; ++i)
    if (in_top[i] && (pick_id == 0 || stats.at<int>(i, cv::CC_STAT_AREA) > stats.at<int>(pick_id, cv::CC_STAT_AREA))) pick_id = i;
  if (pick_id) {
    Mask sel = labels == pick_id;
    out.top.setTo(0);
    fill_holes(morph(sel / 255, cv::MORPH_CLOSE, 2)).copyTo(out.top(w));
  }
  return out;
}

Mask boundary(const Mask & m) { return m & ~erode(m, 1) & 1; }

std::vector<Eigen::Vector2d> convex_hull(std::vector<Eigen::Vector2d> p)  // monotone chain, counter-clockwise
{
  std::sort(p.begin(), p.end(), [](const auto & a, const auto & b) { return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y()); });
  if (p.size() < 3) return p;
  auto cr = [](const Eigen::Vector2d & o, const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    return (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
  };
  std::vector<Eigen::Vector2d> h(2 * p.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && cr(h[k - 2], h[k - 1], p[i]) <= 0) --k;
    h[k++] = p[i];
  }
  for (std::size_t i = p.size() - 1, t = k + 1; i-- > 0;) {
    while (k >= t && cr(h[k - 2], h[k - 1], p[i]) <= 0) --k;
    h[k++] = p[i];
  }
  h.resize(k - 1);
  return h;
}

// rectangle(): outline cast on the top plane -> minimum-area rectangle (1-99 % spread) -> 4 corners (camera frame).
std::array<Eigen::Vector3d, 4> rectangle(const Mask & top, const Eigen::Matrix3d & K, const Eigen::Vector3d & cen,
                                         const Eigen::Vector3d & n)
{
  const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
  const Mask edge = boundary(top);
  Eigen::Vector3d e1 = n.cross(Eigen::Vector3d::UnitX()).normalized(), e2 = n.cross(e1);
  std::vector<Eigen::Vector2d> E2;
  for (int r = 0; r < edge.rows; ++r)
    for (int c = 0; c < edge.cols; ++c)
      if (edge.at<uchar>(r, c)) {
        const Eigen::Vector3d ray((c - cx) / fx, (r - cy) / fy, 1.0);
        const Eigen::Vector3d E3 = ray * (cen.dot(n) / ray.dot(n));
        E2.emplace_back((E3 - cen).dot(e1), (E3 - cen).dot(e2));
      }
  const auto hull = convex_hull(E2);
  if (hull.size() < 3) throw std::runtime_error("box top outline degenerate");
  double best = -1;
  Eigen::Vector2d t = Eigen::Vector2d::UnitX(), q = Eigen::Vector2d::UnitY();
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Eigen::Vector2d ti = (hull[(i + 1) % hull.size()] - hull[i]).normalized(), qi(-ti.y(), ti.x());
    double a0 = 1e300, a1 = -1e300, b0 = 1e300, b1 = -1e300;
    for (const auto & h : hull) {
      a0 = std::min(a0, h.dot(ti)); a1 = std::max(a1, h.dot(ti));
      b0 = std::min(b0, h.dot(qi)); b1 = std::max(b1, h.dot(qi));
    }
    const double area = (a1 - a0) * (b1 - b0);
    if (best < 0 || area < best) { best = area; t = ti; q = qi; }
  }
  std::vector<double> along(E2.size()), across(E2.size());
  for (std::size_t i = 0; i < E2.size(); ++i) { along[i] = E2[i].dot(t); across[i] = E2[i].dot(q); }
  const double a0 = percentile(along, 1), a1 = percentile(along, 99), b0 = percentile(across, 1), b1 = percentile(across, 99);
  const Eigen::Vector2d r2[4] = {t * a0 + q * b0, t * a1 + q * b0, t * a1 + q * b1, t * a0 + q * b1};
  std::array<Eigen::Vector3d, 4> C3;
  for (int i = 0; i < 4; ++i) C3[i] = cen + r2[i].x() * e1 + r2[i].y() * e2;
  return C3;
}

// Indices of the two corners farthest along the floor forward direction: [second, farthest].
std::array<int, 2> far_pair(const std::array<Eigen::Vector3d, 4> & C3, const Eigen::Vector3d & up)
{
  const Eigen::Vector3d fwd = forward(up);
  std::array<int, 4> o{0, 1, 2, 3};
  std::stable_sort(o.begin(), o.end(), [&](int a, int b) { return C3[a].dot(fwd) < C3[b].dot(fwd); });
  return {o[2], o[3]};
}

// far_corner_visibility(): a far corner is hidden at the image border (8 px) or with something at box height or
// above just past it (6, 9, 12 px outward; two of three).
std::map<int, bool> far_corner_visibility(const cv::Mat & d, const Eigen::Matrix3d & K, const Eigen::Vector3d & cen,
                                          const Eigen::Vector3d & n, const std::array<Eigen::Vector3d, 4> & C3,
                                          const Eigen::Vector3d & up)
{
  const int h = d.rows, w = d.cols;
  const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
  auto proj = [&](const Eigen::Vector3d & P) { return Eigen::Vector2d(fx * P.x() / P.z() + cx, fy * P.y() / P.z() + cy); };
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto & c : C3) mean += c;
  const Eigen::Vector2d mid = proj(mean / 4);
  std::map<int, bool> out;
  for (int k : far_pair(C3, up)) {
    const Eigen::Vector2d q = proj(C3[k]);
    const Eigen::Vector2d dirn = (q - mid) / std::max((q - mid).norm(), 1e-9);
    if (!(8 <= q.x() && q.x() < w - 8 && 8 <= q.y() && q.y() < h - 8)) { out[k] = false; continue; }
    int hidden = 0;
    for (double s : {6.0, 9.0, 12.0}) {
      const int u = static_cast<int>(std::nearbyint(q.x() + dirn.x() * s)), v = static_cast<int>(std::nearbyint(q.y() + dirn.y() * s));
      if (!(0 <= u && u < w && 0 <= v && v < h)) { ++hidden; continue; }
      const double z = d.at<double>(v, u);
      if (z > 0 && (Eigen::Vector3d((u - cx) * z / fx, (v - cy) * z / fy, z) - cen).dot(n) > -0.035) ++hidden;
    }
    out[k] = hidden < 2;
  }
  return out;
}

// complete_square_far(): 205 mm square laid off the far edge toward the camera.
std::array<Eigen::Vector3d, 4> complete_square_far(const std::array<Eigen::Vector3d, 4> & C3, const Eigen::Vector3d & up,
                                                   const std::map<int, bool> & seen, std::vector<std::string> & flags)
{
  const Eigen::Vector3d fwd = forward(up);
  const auto [ka, kb] = far_pair(C3, up);
  const Eigen::Vector3d a = C3[ka], b = C3[kb], along = (b - a).normalized();
  Eigen::Vector3d toward = up.cross(along);
  if (toward.dot(fwd) >= 0) toward = -toward;
  const bool sa = seen.count(ka) ? seen.at(ka) : true, sb = seen.count(kb) ? seen.at(kb) : true;
  Eigen::Vector3d m;
  if (sa && sb) m = (a + b) / 2;
  else if (sa) { m = a + along * kSide / 2; flags.push_back("one far corner hidden: centre from the other"); }
  else if (sb) { m = b - along * kSide / 2; flags.push_back("one far corner hidden: centre from the other"); }
  else { m = (a + b) / 2; flags.push_back("both far corners hidden: lateral uncertain"); }
  const Eigen::Vector3d fl = m - along * kSide / 2, fr = m + along * kSide / 2;
  return {fl, fr, fr + toward * kSide, fl + toward * kSide};
}

}  // namespace

CameraMount load_camera_mount(const std::string & urdf_path)
{
  std::ifstream f(urdf_path);
  if (!f) throw std::runtime_error("cannot read URDF " + urdf_path);
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string xml = ss.str();
  // Fixed joints with parent base_link: child -> (xyz, rpy).
  std::map<std::string, std::pair<Eigen::Vector3d, Eigen::Vector3d>> joints;
  const std::regex joint_re(R"re(<joint\b[\s\S]*?</joint>)re"), child_re(R"re(<child\s+link\s*=\s*"([^"]+)")re"),
    parent_re(R"re(<parent\s+link\s*=\s*"([^"]+)")re"), origin_re(R"re(<origin\b([^>]*)>)re"),
    xyz_re(R"re(xyz\s*=\s*"([^"]*)")re"), rpy_re(R"re(rpy\s*=\s*"([^"]*)")re");
  auto vec = [](const std::string & s) { std::istringstream is(s); Eigen::Vector3d v; is >> v.x() >> v.y() >> v.z(); return v; };
  for (std::sregex_iterator it(xml.begin(), xml.end(), joint_re), end; it != end; ++it) {
    const std::string j = it->str();
    std::smatch c, p, o, m;
    if (!std::regex_search(j, c, child_re) || !std::regex_search(j, p, parent_re) || p[1] != "base_link" ||
        !std::regex_search(j, o, origin_re)) continue;
    const std::string attrs = o[1];
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero(), r = Eigen::Vector3d::Zero();
    if (std::regex_search(attrs, m, xyz_re)) xyz = vec(m[1]);
    if (std::regex_search(attrs, m, rpy_re)) r = vec(m[1]);
    joints[c[1]] = {xyz, r};
  }
  for (const char * name : {"camera_top_color_optical_frame", "base_footprint", "front_laser"})
    if (!joints.count(name)) throw std::runtime_error(std::string("URDF has no base_link joint to ") + name);
  const auto & cam = joints["camera_top_color_optical_frame"];
  if (std::abs(joints["front_laser"].first.x() - kLaserX) > 1e-6)
    throw std::runtime_error("URDF front laser differs from calibration reference");
  CameraMount m;
  m.R = rpy(0, 0, 1.5 * M_PI / 180.0) * rpy(cam.second.x(), cam.second.y(), cam.second.z());
  m.t = cam.first;
  m.t.y() = 0.0322;
  m.t.z() = joints["base_footprint"].first.z() + kCamHeight;
  m.up = m.R.transpose() * Eigen::Vector3d::UnitZ();
  return m;
}

Geometry fit_square(const std::vector<Eigen::Vector2d> & points_in, const Eigen::Vector3d & initial)
{
  if (points_in.size() < 30 || !initial.allFinite()) throw std::runtime_error("invalid or insufficient square boundary points");
  const double half = kSide / 2;
  auto coords = [](const Eigen::Vector2d & pt, const Eigen::Vector3d & p) {
    const double dx = pt.x() - p.x(), dy = pt.y() - p.y(), c = std::cos(p.z()), s = std::sin(p.z());
    return Eigen::Vector2d(c * dx + s * dy, -s * dx + c * dy);
  };
  // The near boundary of a clipped top is not the box edge: keep the far edge and the visible sides.
  std::vector<Eigen::Vector2d> pts;
  for (const auto & pt : points_in) {
    const auto xy = coords(pt, initial);
    if (xy.x() > -0.075 || std::abs(std::abs(xy.y()) - half) < 0.018) pts.push_back(pt);
  }
  if (pts.size() < 30) throw std::runtime_error("insufficient visible far and side edges");
  const int N = pts.size();
  auto residual = [&](const Eigen::Vector3d & p) {
    Eigen::VectorXd f(N);
    for (int i = 0; i < N; ++i) {
      const auto xy = coords(pts[i], p);
      const double far = xy.x() - half, side = std::abs(xy.y()) - half;
      const double far_abs = std::abs(far) + std::max(std::abs(xy.y()) - half, 0.0);
      const double side_abs = std::abs(side) + std::max(std::abs(xy.x()) - half, 0.0);
      f[i] = far_abs <= side_abs ? far : side;
    }
    return f;
  };
  // soft_l1 loss, f_scale 5 mm: cost = 0.5 fs^2 sum 2 (sqrt(1 + (f/fs)^2) - 1). Bounded Levenberg-Marquardt with
  // IRLS weights (scipy least_squares trf here; same minimum, its own path).
  const double fs = 0.005;
  auto cost = [&](const Eigen::VectorXd & f) {
    double c = 0;
    for (int i = 0; i < N; ++i) c += 2 * (std::sqrt(1 + (f[i] / fs) * (f[i] / fs)) - 1);
    return 0.5 * fs * fs * c;
  };
  const Eigen::Vector3d span(0.08, 0.08, 20 * M_PI / 180.0), lo = initial - span, hi = initial + span;
  Eigen::Vector3d x = initial;
  Eigen::VectorXd f = residual(x);
  double F = cost(f), lambda = 1e-3;
  int nfev = 1;
  bool converged = false;
  while (nfev < 80 && !converged) {
    Eigen::MatrixXd J(N, 3);
    for (int k = 0; k < 3; ++k) {
      Eigen::Vector3d xp = x;
      const double h = 1.4901161193847656e-08 * std::max(1.0, std::abs(x[k]));
      xp[k] += h;
      J.col(k) = (residual(xp) - f) / h;
    }
    Eigen::VectorXd w(N);
    for (int i = 0; i < N; ++i) w[i] = 1.0 / std::sqrt(1 + (f[i] / fs) * (f[i] / fs));
    const Eigen::Matrix3d A = J.transpose() * w.asDiagonal() * J;
    const Eigen::Vector3d g = J.transpose() * w.cwiseProduct(f);
    if (g.lpNorm<Eigen::Infinity>() < 1e-8) { converged = true; break; }
    bool stepped = false;
    while (nfev < 80) {
      Eigen::Matrix3d Al = A;
      Al.diagonal() *= 1 + lambda;
      Eigen::Vector3d xn = (x - Al.ldlt().solve(g)).cwiseMax(lo).cwiseMin(hi);
      Eigen::VectorXd fn = residual(xn);
      ++nfev;
      const double Fn = cost(fn);
      if (Fn < F) {
        const double dF = F - Fn, dx = (xn - x).norm();
        x = xn; f = fn;
        converged = dF < 1e-8 * F || dx < 1e-8 * (1e-8 + x.norm());
        F = Fn;
        lambda = std::max(lambda / 10, 1e-12);
        stepped = true;
        break;
      }
      if ((xn - x).norm() < 1e-8 * (1e-8 + x.norm())) { converged = true; break; }
      lambda *= 10;
    }
    if (!stepped && !converged) break;
  }
  if (!converged || !x.allFinite()) throw std::runtime_error("square fit failed");
  int far_n = 0, side_n = 0;
  for (const auto & pt : pts) {
    const auto xy = coords(pt, x);
    far_n += std::abs(xy.x() - half) < 0.012;
    side_n += std::abs(std::abs(xy.y()) - half) < 0.012;
  }
  if (far_n < 8 || side_n < 8) throw std::runtime_error("insufficient supported square edges");
  Geometry g;
  g.center_x = x.x(); g.center_y = x.y(); g.box_yaw_rad = x.z();
  g.near_x = x.x() - half * std::cos(x.z());
  g.lateral_m = x.y() - half * std::sin(x.z());
  g.far_inliers = far_n; g.side_inliers = side_n; g.fit_cost = F;
  return g;
}

Geometry square_far20_geometry(const cv::Mat & depth_in, const Eigen::Matrix3d & K, const CameraMount & mount)
{
  if (depth_in.empty() || depth_in.channels() != 1 || !K.allFinite() || K(0, 0) <= 0 || K(1, 1) <= 0)
    throw std::runtime_error("invalid depth image or intrinsics");
  cv::Mat d;
  depth_in.convertTo(d, CV_64F);
  const Top t = box_top(d, K, mount.up);
  const auto corners = rectangle(t.top, K, t.cen, t.n);
  const auto seen = far_corner_visibility(d, K, t.cen, t.n, corners, mount.up);
  std::vector<std::string> flags;
  const auto square = complete_square_far(corners, mount.up, seen, flags);
  std::array<Eigen::Vector3d, 4> Sb;
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (int i = 0; i < 4; ++i) { Sb[i] = mount.R * square[i] + mount.t; mean += Sb[i].head<2>() / 4; }
  Eigen::Vector2d tangent = Sb[1].head<2>() - Sb[0].head<2>();
  if (tangent.y() < 0) tangent = -tangent;
  const double yaw = std::atan2(-tangent.x(), tangent.y());
  const Eigen::Vector3d initial(mean.x(), mean.y(), yaw);

  // Top outline (not the image border) cast on the top plane, in base_link; at most 800 points, evenly spaced.
  const double fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);
  const Mask edge = boundary(t.top);
  std::vector<cv::Point> px;
  for (int r = 3; r < d.rows - 3; ++r)
    for (int c = 3; c < d.cols - 3; ++c)
      if (edge.at<uchar>(r, c)) px.emplace_back(c, r);
  if (px.size() > 800) {
    std::vector<cv::Point> take(800);
    for (int i = 0; i < 800; ++i) take[i] = px[static_cast<std::size_t>(i * (px.size() - 1.0) / 799.0)];
    px.swap(take);
  }
  std::vector<Eigen::Vector2d> outline;
  for (const auto & p : px) {
    const Eigen::Vector3d ray((p.x - cx) / fx, (p.y - cy) / fy, 1.0);
    const double den = ray.dot(t.n);
    if (std::abs(den) < 1e-9) throw std::runtime_error("degenerate projection");
    outline.push_back((mount.R * (ray * (t.cen.dot(t.n) / den)) + mount.t).head<2>());
  }
  // Far-edge points (more than 40 mm beyond the centre) and side points (more than 75 mm aside).
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<Eigen::Vector2d> pts;
  for (const auto & p : outline) if (c * (p.x() - initial.x()) + s * (p.y() - initial.y()) > 0.04) pts.push_back(p);
  for (const auto & p : outline) if (std::abs(-s * (p.x() - initial.x()) + c * (p.y() - initial.y())) > 0.075) pts.push_back(p);
  Geometry g = fit_square(pts, initial);
  g.raw_gap_mm = (g.near_x - kLaserX) * 1000 - 19;
  g.flags = flags;
  g.plane_rmse_mm = t.rms * 1000;
  g.top_area_m2 = t.area;
  return g;
}

// ---- estimator --------------------------------------------------------------------------------------------------

Calibration load_calibration(const std::string & path)
{
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot read calibration " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  const std::string js = ss.str();
  auto str = [&](const std::string & key) {
    std::smatch m;
    return std::regex_search(js, m, std::regex("\"" + key + "\"\\s*:\\s*\"([^\"]*)\"")) ? m[1].str() : std::string();
  };
  auto arr = [&](const std::string & key) {
    std::smatch m;
    std::vector<double> v;
    if (!std::regex_search(js, m, std::regex("\"" + key + "\"\\s*:\\s*\\[([^\\]]*)\\]"))) return v;
    std::string body = m[1];
    std::replace(body.begin(), body.end(), ',', ' ');
    std::istringstream is(body);
    for (double x; is >> x;) v.push_back(x);
    return v;
  };
  if (str("method") != "square_far20" || str("input_reference") != "nominal_LDS03_front_face_dx_mm__origin_minus_19")
    throw std::runtime_error("not the provided selected model");
  Calibration c{arr("input_knots_mm"), arr("gap_knots_mm")};
  if (c.x.size() < 2 || c.x.size() != c.y.size()) throw std::runtime_error("invalid calibration knots");
  for (std::size_t i = 1; i < c.x.size(); ++i)
    if (!(c.x[i] > c.x[i - 1]) || !(c.y[i] > c.y[i - 1])) throw std::runtime_error("invalid calibration knots");
  return c;
}

double Calibration::apply(double raw, bool & extrapolated) const
{
  const long n = x.size();
  const long i = std::clamp<long>(std::upper_bound(x.begin(), x.end(), raw) - x.begin() - 1, 0, n - 2);
  extrapolated = raw < x.front() || raw > x.back();
  return y[i] + (raw - x[i]) * (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
}

bool LatestGate::close(const Item & a, const Item & b) const
{
  return std::abs(b.value - a.value) <= jump_mm_ + max_rate_mm_s_ * std::max(b.time - a.time, 0.0);
}

LatestGate::Out LatestGate::update(double value, long index, double t, bool extrapolated)
{
  Out out;
  if (!std::isfinite(value) || !std::isfinite(t)) return out;
  const Item item{value, index, t, extrapolated};
  const std::optional<Item> & last = anchor_ ? anchor_ : pending_;
  if (last && (t <= last->time || t - last->time > max_gap_s_)) reset();
  history_.push_back(item);
  if (history_.size() > 3) history_.pop_front();
  if ((anchor_ && close(*anchor_, item)) || (pending_ && close(*pending_, item))) {
    anchor_ = item; pending_.reset();
    out.valid = true; out.state = "ready"; out.value = value; out.index = index; out.extrapolated = extrapolated;
    return out;
  }
  out.state = anchor_ ? "outlier" : "warmup";
  pending_ = item;
  return out;
}

Estimate SquareFar20Estimator::estimate(const std::optional<Geometry> & geo, const std::string & error, long frame_index,
                                        double stamp_s)
{
  Estimate r;
  r.capture_stamp_s = stamp_s;
  r.frame_index = frame_index;
  if (!geo) { r.error = error.empty() ? "missing geometry" : error; return r; }  // skipped, the filter keeps its state
  bool extrapolated = false;
  const double value = calibration_.apply(geo->raw_gap_mm, extrapolated);
  const auto f = gate_.update(value, frame_index, stamp_s, extrapolated);
  r.geometry = geo;
  r.raw_gap_mm = geo->raw_gap_mm;
  r.lateral_m = geo->lateral_m;
  r.box_yaw_rad = geo->box_yaw_rad;
  r.extrapolated = r.current_extrapolated = extrapolated;
  r.state = f.state;
  if (f.valid) {  // LatestGate reports the newest frame itself: the source is this frame
    r.state = "geometry_unverified";
    r.gap_mm = f.value;
    r.source_frame_index = frame_index;
    r.source_stamp_s = stamp_s;
  }
  return r;
}

// ---- bridge contract ------------------------------------------------------------------------------------------------

Policy square_observation_policy(const Estimate & e, bool enabled)
{
  if (!enabled) return {false, "camera diagnostic only", 1.0};
  if (e.state != "geometry_unverified") return {false, e.state, 1.0};
  if (!e.gap_mm || !e.lateral_m || !e.box_yaw_rad || !e.source_stamp_s || !std::isfinite(*e.gap_mm) ||
      !std::isfinite(*e.lateral_m) || !std::isfinite(*e.box_yaw_rad) || !std::isfinite(*e.source_stamp_s))
    return {false, "incomplete or nonfinite camera measurement", 1.0};
  const double gap = *e.gap_mm;
  if (!(0.0 <= gap && gap <= 200.0)) return {false, "camera gap outside 0-200 mm experiment envelope", 1.0};
  const bool experimental = e.extrapolated || !(10.0 <= gap && gap <= 80.0);
  return {true, experimental ? "square_far20 experiment outside recorded calibration coverage" : "square_far20 one-bag validated",
          gap <= 30.0 ? 0.005 : 0.010};
}

bool acquisition_valid(double stamp_s, double now_s, double request_s, double timeout)
{
  return std::isfinite(stamp_s) && std::isfinite(now_s) && std::isfinite(request_s) && stamp_s >= request_s &&
         -0.01 <= now_s - stamp_s && now_s - stamp_s <= timeout;
}

Eigen::Vector3d camera_box_in_map(double gap_m, double lateral_m, double yaw_relative, const Eigen::Vector3d & robot,
                                  double front_face_x, double side)
{
  if (!std::isfinite(gap_m) || !std::isfinite(lateral_m) || !std::isfinite(yaw_relative) || !robot.allFinite() ||
      !std::isfinite(front_face_x) || !std::isfinite(side) || gap_m < 0 || side <= 0)
    throw std::runtime_error("invalid camera geometry");
  // The calibrated measurement is to the near-edge midpoint along base x; offset to the centre along the box normal.
  const double x = front_face_x + gap_m + side * 0.5 * std::cos(yaw_relative);
  const double y = lateral_m + side * 0.5 * std::sin(yaw_relative);
  const double c = std::cos(robot.z()), s = std::sin(robot.z());
  return {robot.x() + c * x - s * y, robot.y() + s * x + c * y, robot.z() + yaw_relative};
}

// ---- LiDAR face -------------------------------------------------------------------------------------------------------

std::optional<FaceFit> fit_box_face(const std::vector<Eigen::Vector2d> & points_map, const Eigen::Vector3d & expected,
                                    const Eigen::Vector3d & robot_pose, double side, double front_face_x)
{
  if (!expected.allFinite() || !robot_pose.allFinite() || !std::isfinite(side) || !std::isfinite(front_face_x) || side <= 0)
    return std::nullopt;
  const Eigen::Vector2d center = expected.head<2>(), robot = robot_pose.head<2>();
  // The face toward the robot, of the four the expected yaw allows.
  Eigen::Vector2d normal = Eigen::Vector2d::UnitX();
  double best = -1e300;
  for (int k = 0; k < 4; ++k) {
    const Eigen::Vector2d c(std::cos(expected.z() + k * M_PI / 2), std::sin(expected.z() + k * M_PI / 2));
    if (c.dot(robot - center) > best) { best = c.dot(robot - center); normal = c; }
  }
  const Eigen::Vector2d tangent(-normal.y(), normal.x()), expected_face = center + normal * side / 2;
  // Local coordinates: x along the normal (off the expected face), y along the face. Points within search_m of the
  // expected face both ways (user 2026-11-13: S1 stood 19 cm behind its planning.yaml place, past the old 10 cm;
  // widened to 80 cm the same day, walls are told apart by length and by reaching the window edge).
  constexpr double search_m = 0.80;
  std::vector<double> X, Y;
  for (const auto & p : points_map) {
    if (!p.allFinite()) return std::nullopt;
    const double x = (p - expected_face).dot(normal), y = (p - expected_face).dot(tangent);
    if (std::abs(x) <= search_m && std::abs(y) <= side / 2 + search_m) { X.push_back(x); Y.push_back(y); }
  }
  const int n = X.size();
  if (n < 6) return std::nullopt;
  struct Hyp { int n; double rmse, a, b; std::vector<char> mask; };
  std::vector<Hyp> hyps;
  std::set<std::vector<char>> seen;
  const double slope_max = std::tan(20 * M_PI / 180.0);
  // Points on the line x = a y + b, only its longest run without a gap over 4 cm along the face: a wall point that
  // happens to lie on the line further along is not part of the face (2026-11-13, S1 east: one north-wall point
  // 26 cm past the face made it 46 cm long, a wall).
  auto inliers = [&](double a, double b) {
    std::vector<char> m(n);
    const double k = std::hypot(1.0, a);
    std::vector<int> on;
    for (int i = 0; i < n; ++i) if (std::abs(X[i] - a * Y[i] - b) / k <= 0.006) on.push_back(i);
    std::sort(on.begin(), on.end(), [&](int p, int q) { return Y[p] < Y[q]; });
    std::size_t best_from = 0, best_to = 0;
    for (std::size_t from = 0, i = 0; i < on.size(); ++i) {
      if (i > 0 && Y[on[i]] - Y[on[i - 1]] > 0.04) from = i;
      if (i + 1 - from > best_to - best_from) { best_from = from; best_to = i + 1; }
    }
    for (std::size_t i = best_from; i < best_to; ++i) m[on[i]] = 1;
    return m;
  };
  auto ptp = [&](const std::vector<char> & m) {
    double lo = 1e300, hi = -1e300;
    for (int i = 0; i < n; ++i) if (m[i]) { lo = std::min(lo, Y[i]); hi = std::max(hi, Y[i]); }
    return hi - lo;
  };
  // A line with a point within 3 cm of the window edge goes on past it: a wall, whatever its length inside (a box face
  // is 20 cm and lies well inside the window).
  auto at_edge = [&](const std::vector<char> & m) {
    for (int i = 0; i < n; ++i)
      if (m[i] && (std::abs(X[i]) > search_m - 0.03 || std::abs(Y[i]) > side / 2 + search_m - 0.03)) return true;
    return false;
  };
  // Every pair of points 40 mm apart along the face proposes a line x = a y + b; each distinct inlier set is refit.
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      const double dy = Y[j] - Y[i];
      if (std::abs(dy) < 0.04) continue;
      const double a0 = (X[j] - X[i]) / dy;
      if (std::abs(a0) > slope_max) continue;
      const auto m0 = inliers(a0, X[i] - a0 * Y[i]);
      if (std::count(m0.begin(), m0.end(), 1) < 6 || !seen.insert(m0).second || ptp(m0) < 0.08) continue;
      double mx = 0, my = 0, cnt = 0;
      for (int k = 0; k < n; ++k) if (m0[k]) { mx += Y[k]; my += X[k]; ++cnt; }
      mx /= cnt; my /= cnt;
      double sxx = 0, sxy = 0;
      for (int k = 0; k < n; ++k) if (m0[k]) { sxx += (Y[k] - mx) * (Y[k] - mx); sxy += (Y[k] - mx) * (X[k] - my); }
      if (sxx <= std::numeric_limits<double>::min()) continue;
      const double a = sxy / sxx, b = my - a * mx;
      auto m = inliers(a, b);
      const int c = std::count(m.begin(), m.end(), 1);
      double se = 0;
      for (int k = 0; k < n; ++k) if (m[k]) { const double r = std::abs(X[k] - a * Y[k] - b) / std::hypot(1.0, a); se += r * r; }
      const double rmse = c ? std::sqrt(se / c) : INFINITY;
      // Longer than a box face, or running off the window: a wall, not the box (the search takes in the walls around).
      if (c < 6 || ptp(m) < 0.08 || ptp(m) > side + 0.03 || at_edge(m) || std::abs(a) > slope_max || std::abs(b) > search_m ||
          rmse > 0.004) continue;
      hyps.push_back({c, rmse, a, b, std::move(m)});
    }
  if (hyps.empty()) return std::nullopt;
  std::stable_sort(hyps.begin(), hyps.end(), [](const Hyp & p, const Hyp & q) { return p.n != q.n ? p.n > q.n : p.rmse < q.rmse; });
  const Hyp & h0 = hyps[0];
  for (std::size_t i = 1; i < hyps.size(); ++i) {       // parallel planes with comparable support: ambiguous
    if (hyps[i].n < std::max(6.0, std::ceil(h0.n * 0.8))) break;
    if (std::abs(std::atan(hyps[i].a) - std::atan(h0.a)) < 10 * M_PI / 180.0 && std::abs(hyps[i].b - h0.b) > 0.012) return std::nullopt;
  }
  const Eigen::Vector2d fit_normal = (normal - h0.a * tangent) / std::hypot(1.0, h0.a);
  // Along the face, the middle of the points on it, not the expected box: a box shifted along its face (S2 2026-11-13,
  // 15 cm) otherwise came out where planning.yaml has it. The face ends (observe_face_endpoints) refine it.
  double y_lo = INFINITY, y_hi = -INFINITY;
  for (int i = 0; i < n; ++i) if (h0.mask[i]) { y_lo = std::min(y_lo, Y[i]); y_hi = std::max(y_hi, Y[i]); }
  const double y_mid = (y_lo + y_hi) / 2;
  const Eigen::Vector2d fit_face = expected_face + (h0.a * y_mid + h0.b) * normal + y_mid * tangent,
    fitted_center = fit_face - fit_normal * side / 2;
  const double heading = robot_pose.z();
  const Eigen::Vector2d fwd(std::cos(heading), std::sin(heading)), left(-fwd.y(), fwd.x());
  const double denom = fit_normal.dot(fwd);
  if (std::abs(denom) < 0.3) return std::nullopt;
  FaceFit r;
  r.box_center_map = fitted_center;
  r.face_normal_map = fit_normal;
  r.box_yaw = std::atan2(fit_normal.y(), fit_normal.x());
  r.gap_m = fit_normal.dot(fit_face - robot) / denom - front_face_x;
  r.lateral_m = (fit_face - robot).dot(left);
  const double facing = std::atan2(-fit_normal.y(), -fit_normal.x()) - heading;
  r.yaw_relative = std::atan2(std::sin(facing), std::cos(facing));
  r.rmse_m = h0.rmse;
  r.inliers = h0.n;
  return r;
}

FaceEnds observe_face_endpoints(const std::vector<double> & ranges, const std::vector<double> & angles,
                                const Eigen::Matrix3d & rotation, const Eigen::Vector3d & origin, const FaceFit & fit,
                                double side, double sensor_min, double sensor_max)
{
  constexpr double face_tol = 0.010, width_tol = 0.006, window = 0.08, straddle_m = 0.040;
  auto fail = [](const std::string & reason) { FaceEnds e; e.reason = reason; return e; };
  const long n = ranges.size();
  if (n < 3 || angles.size() != ranges.size()) return fail("invalid scan");
  const Eigen::Vector2d normal = fit.face_normal_map, tangent(-normal.y(), normal.x());
  const Eigen::Vector2d face_point = fit.box_center_map + normal * side / 2, o = origin.head<2>();
  std::vector<double> predicted(n), along(n);
  std::vector<char> meets(n), on_face(n);
  for (long i = 0; i < n; ++i) {
    const Eigen::Vector2d ray = (rotation * Eigen::Vector3d(std::cos(angles[i]), std::sin(angles[i]), 0)).head<2>();
    const double den = ray.dot(normal);
    predicted[i] = (face_point - o).dot(normal) / den;
    meets[i] = std::isfinite(predicted[i]) && predicted[i] > 0 && den < 0;
    along[i] = meets[i] ? ((o + predicted[i] * ray) - face_point).dot(tangent) : INFINITY;
    on_face[i] = meets[i] && std::abs(along[i]) <= side / 2 + window && std::isfinite(ranges[i]) && ranges[i] > 0 &&
                 std::abs(ranges[i] - predicted[i]) <= face_tol;
  }
  std::vector<long> ids;
  for (long i = 0; i < n; ++i) if (on_face[i]) ids.push_back(i);
  if (ids.size() < 3) return fail("too few beams on the face");
  std::vector<double> inc(n - 1);
  for (long i = 0; i + 1 < n; ++i) inc[i] = std::abs(angles[i + 1] - angles[i]);
  std::nth_element(inc.begin(), inc.begin() + inc.size() / 2, inc.end());
  double increment = inc[inc.size() / 2];
  if (inc.size() % 2 == 0) increment = (increment + *std::max_element(inc.begin(), inc.begin() + inc.size() / 2)) / 2;
  const auto [amin, amax] = std::minmax_element(angles.begin(), angles.end());
  const bool full_turn = *amax - *amin + 1.5 * increment >= 2 * M_PI;
  auto wrap = [&](long i) { return ((i % n) + n) % n; };
  long first, last;
  if (full_turn) {
    std::vector<std::size_t> breaks;
    for (std::size_t k = 0; k < ids.size(); ++k) {
      const long next = k + 1 < ids.size() ? ids[k + 1] : ids[0] + n;
      if (next - ids[k] != 1) breaks.push_back(k);
    }
    if (breaks.size() != 1) return fail("face beams split into " + std::to_string(breaks.size()) + " runs");
    last = ids[breaks[0]];
    first = ids[(breaks[0] + 1) % ids.size()];
  } else {
    for (std::size_t k = 1; k < ids.size(); ++k) if (ids[k] - ids[k - 1] != 1) return fail("face beams split");
    first = ids.front(); last = ids.back();
  }
  auto straddles = [&](long i) {
    const double r = ranges[i];
    return r == 0 || std::isnan(r) || (meets[i] && std::isfinite(r) && predicted[i] - straddle_m <= r && r < predicted[i] - face_tol);
  };
  std::vector<std::pair<double, double>> ends;
  for (const auto & [inner, step] : {std::pair<long, long>{first, -1}, std::pair<long, long>{last, 1}}) {
    long outer = inner + step;
    if (full_turn) outer = wrap(outer);
    else if (outer < 0 || outer >= n) return fail("scan ends at the face edge");
    int skipped = 0;
    while (straddles(outer)) {  // up to two beams straddling the edge are skipped
      ++skipped;
      outer += step;
      if (full_turn) outer = wrap(outer);
      else if (outer < 0 || outer >= n) return fail("scan ends at the face edge");
      if (skipped > 2) return fail("no return beside the face edge");
    }
    const double r = ranges[outer], expected = predicted[outer];
    const bool observable = meets[outer] && expected >= sensor_min && expected <= sensor_max;
    if (std::isinf(r) && r > 0) {
      if (!observable) return fail("no return beside the face edge, face out of sensor range there");
    } else {
      if (r < sensor_min || r > sensor_max) return fail("return beside the face edge outside the sensor range");
      if (meets[outer] && r < expected - face_tol) return fail("something in front of the face edge");
      if (meets[outer] && r <= expected + face_tol) return fail("face continues past the window");
    }
    const double a_inner = along[inner];
    const double spacing = std::abs(a_inner - along[wrap(inner - step)]);
    const double a_outer = std::isfinite(along[outer]) ? along[outer] : a_inner + step * spacing * wrap((outer - inner) * step);
    ends.emplace_back((a_inner + a_outer) / 2, std::abs(a_outer - a_inner) / 2);
  }
  std::sort(ends.begin(), ends.end());
  const double width = ends[1].first - ends[0].first;
  if (std::abs(width - side) > width_tol + ends[0].second + ends[1].second) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "measured face width %.1f mm, not %.1f", width * 1000, side * 1000);
    return fail(buf);
  }
  FaceEnds e;
  e.ok = true;
  e.reason = "both face ends seen";
  e.face_centre_map = face_point + tangent * (ends[0].first + ends[1].first) / 2;
  e.box_center_map = e.face_centre_map - normal * side / 2;
  e.width_m = width;
  return e;
}

std::optional<FoundBox> find_box(const std::vector<Eigen::Vector2d> & points_map, const Eigen::Vector3d & expected,
                                 const Eigen::Vector2d & robot, double side, double radius, double cluster_gap)
{
  const Eigen::Vector2d centre(expected.x(), expected.y());
  std::vector<Eigen::Vector2d> p;
  for (const auto & q : points_map)
    if (q.allFinite() && (q - centre).norm() <= radius) p.push_back(q);
  const int n = static_cast<int>(p.size());
  // ponytail: O(n^2) clustering, fine for the few hundred points around one box.
  std::vector<int> label(n, -1);
  int clusters = 0;
  for (int i = 0; i < n; ++i) {
    if (label[i] >= 0) continue;
    std::vector<int> stack{i};
    label[i] = clusters;
    while (!stack.empty()) {
      const int a = stack.back();
      stack.pop_back();
      for (int b = 0; b < n; ++b)
        if (label[b] < 0 && (p[a] - p[b]).norm() <= cluster_gap) { label[b] = clusters; stack.push_back(b); }
    }
    ++clusters;
  }
  std::optional<FoundBox> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int k = 0; k < clusters; ++k) {
    std::vector<Eigen::Vector2d> c;
    for (int i = 0; i < n; ++i) if (label[i] == k) c.push_back(p[i]);
    if (c.size() < 8) continue;
    // Bounding rectangle per yaw: points' mean distance to its nearest side, smallest wins.
    double cost_min = std::numeric_limits<double>::infinity(), th_best = 0;
    for (int d = 0; d < 90; ++d) {
      const double th = d * M_PI / 180, co = std::cos(th), si = std::sin(th);
      double u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
      for (const auto & q : c) {
        const double u = co * q.x() + si * q.y(), v = -si * q.x() + co * q.y();
        u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v);
      }
      double cost = 0;
      for (const auto & q : c) {
        const double u = co * q.x() + si * q.y(), v = -si * q.x() + co * q.y();
        cost += std::min({u - u0, u1 - u, v - v0, v1 - v});
      }
      cost /= c.size();
      if (cost < cost_min) { cost_min = cost; th_best = th; }
    }
    const double co = std::cos(th_best), si = std::sin(th_best);
    double u0 = 1e9, u1 = -1e9, v0 = 1e9, v1 = -1e9;
    for (const auto & q : c) {
      const double u = co * q.x() + si * q.y(), v = -si * q.x() + co * q.y();
      u0 = std::min(u0, u); u1 = std::max(u1, u); v0 = std::min(v0, v); v1 = std::max(v1, v);
    }
    const double eu = u1 - u0, ev = v1 - v0;
    if (eu > side + 0.04 || ev > side + 0.04 || std::max(eu, ev) < 0.08) continue;  // not one box, or too little of it
    const double ru = co * robot.x() + si * robot.y(), rv = -si * robot.x() + co * robot.y();
    // Both faces of an axis seen (extent ~ side): their middle; else side / 2 behind the face toward the robot.
    const auto mid = [side](double lo, double hi, double r) {
      if (hi - lo >= side - 0.03) return (lo + hi) / 2;
      return r < (lo + hi) / 2 ? lo + side / 2 : hi - side / 2;
    };
    const double cu = mid(u0, u1, ru), cv = mid(v0, v1, rv);
    FoundBox f{co * cu - si * cv, si * cu + co * cv, th_best, static_cast<int>(c.size()), eu, ev};
    // The same square every quarter turn: the yaw nearest the expected one.
    f.yaw = th_best + std::round(std::remainder(expected.z() - th_best, 2 * M_PI) / (M_PI / 2)) * (M_PI / 2);
    const double distance = std::hypot(f.x - centre.x(), f.y - centre.y());
    if (distance < best_distance) { best_distance = distance; best = f; }
  }
  return best;
}

}  // namespace uppercam
