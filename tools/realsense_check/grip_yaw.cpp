// Gripper-cam (그립캠) standing-prism yaw from depth, the C++ port of grip_yaw.py's depth reading (2026-11-15: the
// Python capture + reading took ~3 s with the arm holding the wait pose; the shoulder could not hold that long).
//   grip_yaw [--frames N] [--warm N] [--serial S] [--save PREFIX]   live: depth 640x360, 10 frames, no warm-up
//   grip_yaw --photo PREFIX                                         record only: PREFIX_color.png (1280x720), _depth.png
//   grip_yaw --stack PREFIX                                         offline: PREFIX_stack.u16 + PREFIX_cam.txt
// Prints "<source>: YAW <deg> deg CENTRE <x> <y> <z> mm (depth: raw ..., footprint ..., top ..., n/N frames, sd ...)" (execute.py parses
// "YAW <deg> deg"; CENTRE = the top face's centre in the colour camera frame [mm], median over the frames), or "YAW NONE ...". Calibration: grip_yaw.json next to the binary's source dir (offset_deg, sign).
// Method (as grip_yaw.py): surface plane = RANSAC on the per-pixel median depth (points in the colour camera frame);
// per frame: points 12-75 mm over it, 2 mm grid blobs, the prism-sized blob nearest the camera, its top-face pixels'
// rays cut with the top plane, smallest percentile-extent rectangle; raw = circular mean (90 deg period) over frames.
// PREFIX_cam.txt: "w h fx fy ppx ppy scale" (depth intrinsics) then 9 rotation (row-major, depth->colour) + 3 translation.
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#ifdef CRAFT_PACKAGED_CAMERA
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {
std::string camera_data_dir() {
#ifdef CRAFT_PACKAGED_CAMERA
  return ament_index_cpp::get_package_share_directory("craft_sensors") + "/puck_camera";
#else
  return GRIP_YAW_DIR;
#endif
}
// blobs from the top face only (2026-11-15 20:19: with 12-75 mm a lower object beside the prism joined it): the prism
// is 60 mm tall, its top 48-72 mm over the surface; its top face 20-40 mm across
const double kHLo = 48.0, kHHi = 72.0, kSideLo = 20.0, kSideHi = 40.0, kGrid = 2.0, kTopBand = 6.0, kPct = 3.0;
std::string kCal;

struct V3 {double x, y, z;};
V3 operator-(V3 a, V3 b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
V3 operator+(V3 a, V3 b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
V3 operator*(V3 a, double s) {return {a.x * s, a.y * s, a.z * s};}
double dot(V3 a, V3 b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
V3 cross(V3 a, V3 b) {return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};}
double norm(V3 a) {return std::sqrt(dot(a, a));}

double fold(double a, double p = 90.0) {double r = std::fmod(a + p / 2, p); if (r < 0) {r += p;} return r - p / 2;}
double mean_fold(const std::vector<double> & a, double p = 90.0) {
  double s = 0, c = 0, k = 360.0 / p;
  for (double v : a) {s += std::sin(v * k * M_PI / 180); c += std::cos(v * k * M_PI / 180);}
  return std::atan2(s, c) * 180 / M_PI / k;
}
double pctn(std::vector<double> & v, double q) {  // linear percentile, partial sort (v is reordered)
  double i = q / 100.0 * (v.size() - 1); size_t a = static_cast<size_t>(i);
  std::nth_element(v.begin(), v.begin() + a, v.end());
  double lo = v[a];
  if (a + 1 >= v.size()) {return lo;}
  double hi = *std::min_element(v.begin() + a + 1, v.end());
  return lo + (hi - lo) * (i - a);
}
double pct(std::vector<double> v, double q) {  // numpy-like linear percentile
  if (v.empty()) {return 0;}
  std::sort(v.begin(), v.end());
  double i = q / 100.0 * (v.size() - 1); size_t a = static_cast<size_t>(i);
  return a + 1 < v.size() ? v[a] + (v[a + 1] - v[a]) * (i - a) : v[a];
}

struct Cam {int w, h; double fx, fy, ppx, ppy, scale; double R[9]; double t[3];};
struct Plane {V3 n, p0; bool ok = false;};

std::vector<V3> points(const uint16_t * d, const Cam & c, std::vector<int> * pix = nullptr) {
  std::vector<V3> P; P.reserve(c.w * c.h / 2);
  for (int v = 0; v < c.h; ++v) {
    for (int u = 0; u < c.w; ++u) {
      double z = d[v * c.w + u] * c.scale;
      if (z <= 0.05 || z >= 1.0) {continue;}
      V3 q{(u - c.ppx) / c.fx * z, (v - c.ppy) / c.fy * z, z};
      P.push_back({c.R[0] * q.x + c.R[1] * q.y + c.R[2] * q.z + c.t[0], c.R[3] * q.x + c.R[4] * q.y + c.R[5] * q.z + c.t[1],
                   c.R[6] * q.x + c.R[7] * q.y + c.R[8] * q.z + c.t[2]});
      if (pix) {pix->push_back(v * c.w + u);}
    }
  }
  return P;
}

Plane fit_ls(const std::vector<V3> & S, const std::vector<char> & keep) {
  V3 m{0, 0, 0}; int n = 0;
  for (size_t i = 0; i < S.size(); ++i) {if (keep[i]) {m = m + S[i]; ++n;}}
  Plane pl; if (n < 3) {return pl;}
  m = m * (1.0 / n);
  cv::Matx33d C = cv::Matx33d::zeros();
  for (size_t i = 0; i < S.size(); ++i) {
    if (!keep[i]) {continue;}
    V3 d = S[i] - m; double a[3] = {d.x, d.y, d.z};
    for (int r = 0; r < 3; ++r) {for (int k = 0; k < 3; ++k) {C(r, k) += a[r] * a[k];}}
  }
  cv::Mat ev, evec; cv::eigen(cv::Mat(C), ev, evec);  // descending: the last row is the normal
  pl.n = {evec.at<double>(2, 0), evec.at<double>(2, 1), evec.at<double>(2, 2)};
  pl.p0 = m; pl.ok = true;
  return pl;
}

Plane surface_plane(const std::vector<V3> & P, int iters = 400, double inl = 0.003, size_t max_pts = 15000) {
  Plane best; if (P.size() < 200) {return best;}
  std::mt19937 rng(0);
  std::vector<V3> S;
  if (P.size() > max_pts) {
    std::vector<size_t> idx(P.size()); for (size_t i = 0; i < idx.size(); ++i) {idx[i] = i;}
    std::shuffle(idx.begin(), idx.end(), rng); for (size_t i = 0; i < max_pts; ++i) {S.push_back(P[idx[i]]);}
  } else {S = P;}
  std::uniform_int_distribution<size_t> pick(0, S.size() - 1);
  int bc = -1; V3 bn{0, 0, 1}, ba{0, 0, 0};
  for (int it = 0; it < iters; ++it) {
    V3 a = S[pick(rng)], b = S[pick(rng)], c = S[pick(rng)];
    V3 n = cross(b - a, c - a); double l = norm(n); if (l < 1e-9) {continue;}
    n = n * (1 / l); int cnt = 0;
    for (const auto & s : S) {cnt += std::fabs(dot(s - a, n)) < inl;}
    if (cnt > bc) {bc = cnt; bn = n; ba = a;}
  }
  std::vector<char> keep(S.size());
  for (size_t i = 0; i < S.size(); ++i) {keep[i] = std::fabs(dot(S[i] - ba, bn)) < inl;}
  Plane pl;
  for (int r = 0; r < 2; ++r) {
    pl = fit_ls(S, keep); if (!pl.ok) {return pl;}
    for (size_t i = 0; i < S.size(); ++i) {keep[i] = std::fabs(dot(S[i] - pl.p0, pl.n)) < inl;}
  }
  if (pl.n.z > 0) {pl.n = pl.n * -1;}
  return pl;
}

void axes(V3 n, V3 & ax, V3 & ay) {ax = V3{1, 0, 0} - n * n.x; ax = ax * (1 / norm(ax)); ay = cross(n, ax);}

// smallest FIT_PCT-extent rectangle: angle [deg, folded], sides
double rect_fit(const std::vector<cv::Point2d> & Q, double & w, double & l, cv::Point2d * centre = nullptr, bool raw = false) {
  double cu = 0, cv_ = 0;
  auto area = [&](double t, double * ww, double * ll) {
    double r = t * M_PI / 180, c = std::cos(r), s = std::sin(r);
    std::vector<double> u(Q.size()), v(Q.size());
    for (size_t i = 0; i < Q.size(); ++i) {u[i] = Q[i].x * c + Q[i].y * s; v[i] = -Q[i].x * s + Q[i].y * c;}
    double uh = pctn(u, 100 - kPct), ul = pctn(u, kPct), vh = pctn(v, 100 - kPct), vl = pctn(v, kPct);
    double a = uh - ul, b = vh - vl;
    if (ww) {*ww = a; *ll = b; cu = (uh + ul) / 2; cv_ = (vh + vl) / 2;}
    return a * b;
  };
  double bt = 0, ba = 1e18;
  for (double t = 0; t < 90; t += 1.0) {double a = area(t, nullptr, nullptr); if (a < ba) {ba = a; bt = t;}}
  double t0 = bt;
  for (double t = t0 - 1.0; t <= t0 + 1.0; t += 0.25) {double a = area(t, nullptr, nullptr); if (a < ba) {ba = a; bt = t;}}
  t0 = bt;
  for (double t = t0 - 0.25; t <= t0 + 0.25; t += 0.05) {double a = area(t, nullptr, nullptr); if (a < ba) {ba = a; bt = t;}}
  area(bt, &w, &l);
  if (centre) {double r = bt * M_PI / 180; *centre = {cu * std::cos(r) - cv_ * std::sin(r), cu * std::sin(r) + cv_ * std::cos(r)};}
  return raw ? bt : fold(bt);  // raw: unfolded, w lies along it (a lying puck needs to know its long side)
}

bool centre_only = false;
std::string requested_posture;
std::vector<cv::Point2d> avoid_targets;
double target_x = 0, target_y = 0, target_radius = 0;  // colour optical mm; gate competing blobs

struct Reading {bool ok = false; double raw, w, l, top; V3 centre;};  // centre: top-face centre, colour cam frame [m]

// Standing prism: top 60 mm, square top. Lying prism (2026-11-15): top 30 mm, 30 x 60 footprint, the yaw is its long
// axis (180 deg period). Blobs from the top-face height band only.
struct Mode {double h_lo, h_hi, short_lo, short_hi, long_lo, long_hi, top; bool lying;};
const Mode kStand{48, 72, 20, 40, 20, 40, 60, false};
const Mode kLying{22, 38, 20, 42, 45, 75, 30, true};

Reading measure(const uint16_t * d, const Cam & c, const Plane & pl, const Mode & md = kStand) {
  Reading out;
  std::vector<V3> P = points(d, c);
  V3 ax, ay; axes(pl.n, ax, ay);
  std::vector<V3> Ps; std::vector<double> hs; std::vector<cv::Point2d> Q;
  for (const auto & p : P) {
    double h = dot(p - pl.p0, pl.n) * 1000;
    if (h > md.h_lo && h < md.h_hi) {Ps.push_back(p); hs.push_back(h); Q.push_back({dot(p - pl.p0, ax) * 1000, dot(p - pl.p0, ay) * 1000});}
  }
  if (Ps.size() < 30) {return out;}
  double lx = 1e9, ly = 1e9, hx = -1e9, hy = -1e9;
  for (auto & q : Q) {lx = std::min(lx, q.x); ly = std::min(ly, q.y); hx = std::max(hx, q.x); hy = std::max(hy, q.y);}
  int gw = static_cast<int>((hx - lx) / kGrid) + 1, gh = static_cast<int>((hy - ly) / kGrid) + 1;
  cv::Mat img = cv::Mat::zeros(gh, gw, CV_8U);
  std::vector<cv::Point> ij(Q.size());
  for (size_t i = 0; i < Q.size(); ++i) {
    ij[i] = {static_cast<int>((Q[i].x - lx) / kGrid), static_cast<int>((Q[i].y - ly) / kGrid)};
    img.at<uchar>(ij[i]) = 1;
  }
  cv::Mat k3 = cv::Mat::ones(3, 3, CV_8U);
  cv::morphologyEx(img, img, cv::MORPH_CLOSE, k3);
  cv::morphologyEx(img, img, cv::MORPH_OPEN, k3);
  cv::Mat lab; int nl = cv::connectedComponents(img, lab, 8, CV_32S);
  std::vector<std::vector<size_t>> comp(nl);
  for (size_t i = 0; i < Q.size(); ++i) {int k = lab.at<int>(ij[i]); if (k > 0) {comp[k].push_back(i);}}
  int best = -1; double bd = 1e9;
  for (int k = 1; k < nl; ++k) {
    if (comp[k].size() < 30) {continue;}
    std::vector<cv::Point2f> q; for (auto i : comp[k]) {q.push_back(cv::Point2f(Q[i].x, Q[i].y));}
    cv::Size2f sz = cv::minAreaRect(q).size;
    double a = std::min(sz.width, sz.height), b = std::max(sz.width, sz.height);
    if (a < md.short_lo || a > md.short_hi || b < md.long_lo || b > md.long_hi) {continue;}
    // the most prism-like blob (2026-11-15 20:19: a long object beside the prism, nearer the camera, was taken when
    // the nearest blob won): top near 60 mm and a square footprint
    std::vector<double> hb; for (auto i : comp[k]) {hb.push_back(hs[i]);}
    double cost = std::fabs(pct(hb, 95) - md.top) / 10.0 + (md.lying ? std::fabs(b / a - 2.0) * 2.0 : (1.0 - a / b) * 5.0);
    if (target_radius > 0) {
      V3 mean{0,0,0}; for(auto i:comp[k]) mean=mean+Ps[i]; mean=mean*(1.0/comp[k].size());
      const double distance=std::hypot(mean.x*1000-target_x,mean.y*1000-target_y);
      if(distance>target_radius) continue;
      bool rival=false;
      for(const auto &a:avoid_targets)
        if(std::hypot(mean.x*1000-a.x,mean.y*1000-a.y)<=distance+5) {rival=true;break;}
      if(rival) continue;
      cost=distance;  // selected target takes precedence over which blob looks more square
    }
    if (cost < bd) {bd = cost; best = k;}
  }
  if (best < 0) {return out;}
  std::vector<double> hb; for (auto i : comp[best]) {hb.push_back(hs[i]);}
  double top = pct(hb, 95);
  V3 pt = pl.p0 + pl.n * (top / 1000);
  std::vector<cv::Point2d> Qt;
  for (auto i : comp[best]) {
    if (std::fabs(hs[i] - top) >= kTopBand) {continue;}
    V3 r = Ps[i] * (1 / Ps[i].z);
    V3 X = r * (dot(pt, pl.n) / dot(r, pl.n));
    Qt.push_back({dot(X - pt, ax) * 1000, dot(X - pt, ay) * 1000});
  }
  if (Qt.size() < 30) {return out;}
  cv::Point2d c2;
  if (md.lying) {  // the long axis, folded to 180
    double t = rect_fit(Qt, out.w, out.l, &c2, true);
    out.raw = fold(out.w > out.l ? t : t + 90.0, 180.0);
  } else {
    out.raw = rect_fit(Qt, out.w, out.l, &c2);
  }
  out.top = top; out.ok = true;
  out.centre = pt + ax * (c2.x / 1000) + ay * (c2.y / 1000);
  return out;
}

bool read_cal(double & off, double & sign, const std::string & pre = "") {
  std::ifstream f(kCal); if (!f) {return false;}
  std::stringstream ss; ss << f.rdbuf(); std::string s = ss.str();
  auto num = [&](const std::string & key, double & v) {
    size_t p = s.find("\"" + key + "\""); if (p == std::string::npos) {return false;}
    p = s.find(':', p); v = std::stod(s.substr(p + 1)); return true;
  };
  return num(pre + "offset_deg", off) && num(pre + "sign", sign);
}
}  // namespace

int main(int argc, char ** argv) try {
  auto t0 = std::chrono::steady_clock::now();
  int frames = 10, warm = 0;  // 10 frames in all (user 2026-11-15); a bad first frame is dropped like any other
  std::string serial = "261622073363", stack, save, photo, data_dir;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--data-dir" && i + 1 < argc) {data_dir = argv[++i];}
    else if (a == "--no-yaw") {centre_only=true;requested_posture="standing";}
    else if (a == "--posture" && i+1<argc) {requested_posture=argv[++i];}
    else if (a == "--avoid" && i+2<argc) {double x=std::stod(argv[++i]);double y=std::stod(argv[++i]);avoid_targets.emplace_back(x,y);}
    else if (a == "--target" && i+3<argc) {target_x=std::stod(argv[++i]);target_y=std::stod(argv[++i]);target_radius=std::stod(argv[++i]);}
    else if (a == "--frames" && i + 1 < argc) {frames = std::atoi(argv[++i]);}
    else if (a == "--warm" && i + 1 < argc) {warm = std::atoi(argv[++i]);}
    else if (a == "--serial" && i + 1 < argc) {serial = argv[++i];}
    else if (a == "--stack" && i + 1 < argc) {stack = argv[++i];}
    else if (a == "--save" && i + 1 < argc) {save = argv[++i];}
    else if (a == "--photo" && i + 1 < argc) {photo = argv[++i];}
  }
  kCal = (data_dir.empty() ? camera_data_dir() : data_dir) + "/grip_yaw.json";
  Cam c{}; std::vector<std::vector<uint16_t>> D; std::string src;
  if (!photo.empty()) {  // --photo PREFIX: record only (2026-11-15, --find at the grasp pose): colour 1280x720 + depth, no reading
    rs2::config cfg; cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe; pipe.start(cfg);
    rs2::frameset fs;
    for (int i = 0; i < 5; ++i) {fs = pipe.wait_for_frames();}  // a few frames for the colour exposure
    auto c = fs.get_color_frame(); auto d = fs.get_depth_frame();
    cv::imwrite(photo + "_color.png", cv::Mat(c.get_height(), c.get_width(), CV_8UC3, const_cast<void *>(c.get_data())));
    cv::imwrite(photo + "_depth.png", cv::Mat(d.get_height(), d.get_width(), CV_16U, const_cast<void *>(d.get_data())));
    pipe.stop();
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    printf("%s: PHOTO saved (%.0f ms)\n", photo.c_str(), ms);
    return 0;
  }
  if (!stack.empty()) {
    std::ifstream ct(stack + "_cam.txt");
    ct >> c.w >> c.h >> c.fx >> c.fy >> c.ppx >> c.ppy >> c.scale;
    for (double & r : c.R) {ct >> r;}
    for (double & t : c.t) {ct >> t;}
    std::ifstream sf(stack + "_stack.u16", std::ios::binary);
    std::vector<uint16_t> buf(c.w * c.h);
    while (sf.read(reinterpret_cast<char *>(buf.data()), buf.size() * 2)) {D.push_back(buf);}
    src = stack;
  } else {
    rs2::config cfg; cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 360, RS2_FORMAT_Z16, 30);  // 640x360: sees down to 13 cm (top ~22 cm)
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_BGR8, 30);  // only for the depth->colour extrinsic
    rs2::pipeline pipe; auto prof = pipe.start(cfg);
    auto ds = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto cs = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto in = ds.get_intrinsics(); auto ex = ds.get_extrinsics_to(cs);
    c.w = in.width; c.h = in.height; c.fx = in.fx; c.fy = in.fy; c.ppx = in.ppx; c.ppy = in.ppy;
    c.scale = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
    for (int r = 0; r < 3; ++r) {for (int k = 0; k < 3; ++k) {c.R[r * 3 + k] = ex.rotation[k * 3 + r];}}  // column-major
    for (int k = 0; k < 3; ++k) {c.t[k] = ex.translation[k];}
    for (int i = 0; i < warm; ++i) {pipe.wait_for_frames();}
    // Small depth holes (the AprilTag on a puck top, user 2026-11-16: a cylinder top read 18x19 mm in 3/10 frames):
    // RealSense spatial filter (edge-preserving, fills holes up to holes_fill) then hole filling from the farthest
    // neighbour (a hole at a puck edge takes the background, so the top does not grow).
    rs2::spatial_filter spatial; spatial.set_option(RS2_OPTION_HOLES_FILL, 2);
    rs2::hole_filling_filter holes(1);
    for (int i = 0; i < frames; ++i) {
      rs2::frame f = pipe.wait_for_frames().get_depth_frame();
      f = holes.process(spatial.process(f));
      auto d = f.as<rs2::depth_frame>();
      const auto * p = static_cast<const uint16_t *>(d.get_data());
      D.emplace_back(p, p + c.w * c.h);
    }
    pipe.stop();
    src = "live";
    if (!save.empty()) {
      std::ofstream sf(save + "_stack.u16", std::ios::binary);
      for (auto & f : D) {sf.write(reinterpret_cast<const char *>(f.data()), f.size() * 2);}
      std::ofstream ct(save + "_cam.txt"); ct.precision(10);
      ct << c.w << " " << c.h << " " << c.fx << " " << c.fy << " " << c.ppx << " " << c.ppy << " " << c.scale << "\n";
      for (double r : c.R) {ct << r << " ";}
      ct << "\n" << c.t[0] << " " << c.t[1] << " " << c.t[2] << "\n";
      src = save;
    }
  }
  if (D.empty()) {std::cout << src << ": YAW NONE (no depth frames)\n"; return 1;}
  // surface: per-pixel median over the frames (valid ones)
  std::vector<uint16_t> med(c.w * c.h, 0); std::vector<uint16_t> v; v.reserve(D.size());
  for (int i = 0; i < c.w * c.h; ++i) {
    v.clear(); for (auto & f : D) {if (f[i]) {v.push_back(f[i]);}}
    if (v.empty()) {continue;}
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    med[i] = v[v.size() / 2];
  }
  // Holes still left in one frame take the per-pixel median of all frames (they move from frame to frame), so
  // each frame is read with a full puck top; the frames stay separate for the yaw spread check.
  for (auto & f : D) {for (int i = 0; i < c.w * c.h; ++i) {if (!f[i]) {f[i] = med[i];}}}
  Plane pl = surface_plane(points(med.data(), c));
  if (!pl.ok) {std::cout << src << ": YAW NONE (no surface)\n"; return 1;}
  // The largest plane can be the floor beside the box (user 2026-11-16: with the LM far to one side the floor filled
  // more of the view, the box top read 66-69 mm "high" and the puck on it was out of the height window). The box top
  // the pucks stand on is the highest large plane: a plane parallel to the first (< 5 deg), 55-85 mm above it (the 70 mm box), with
  // at least 10 % of the points replaces it.
  {
    const auto P = points(med.data(), c);
    std::vector<V3> above;
    for (const auto & p : P) {if (dot(p - pl.p0, pl.n) > 0.030) {above.push_back(p);}}
    const Plane up = surface_plane(above);
    if (up.ok) {
      size_t on = 0;
      for (const auto & p : above) {on += std::fabs(dot(p - up.p0, up.n)) < 0.003;}
      const double rise = dot(up.p0 - pl.p0, pl.n), cosang = std::fabs(dot(up.n, pl.n));
      if (cosang > std::cos(5.0 * M_PI / 180) && rise > 0.055 && rise < 0.085 && on * 10 >= P.size()) {
        std::fprintf(stderr, "surface: box top %.0f mm above the largest plane (%zu of %zu points): taken as the shelf\n",
                     rise * 1000, on, P.size());
        pl = up;
      }
    }
  }
  // standing or lying: each frame read both ways, the kind read in more frames wins (a tie: standing)
  std::vector<Reading> rs_s, rs_l;
  for (auto & f : D) {
    Reading r = measure(f.data(), c, pl, kStand); if (r.ok) {rs_s.push_back(r);}
    Reading q = measure(f.data(), c, pl, kLying); if (q.ok) {rs_l.push_back(q);}
  }
  bool lying = requested_posture == "lying" || (requested_posture.empty() && rs_l.size() > rs_s.size());
  const auto & rs = lying ? rs_l : rs_s;
  const double period = lying ? 180.0 : 90.0;
  std::vector<double> raws, ws, ls, tops, cxs, cys, czs;
  for (const auto & r : rs) {
    raws.push_back(r.raw); ws.push_back(std::min(r.w, r.l)); ls.push_back(std::max(r.w, r.l)); tops.push_back(r.top);
    cxs.push_back(r.centre.x * 1000); cys.push_back(r.centre.y * 1000); czs.push_back(r.centre.z * 1000);
  }
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (raws.empty()) {printf("%s: YAW NONE (depth: no prism, %zu frames, %.0f ms)\n", src.c_str(), D.size(), ms); return 1;}
  double raw = mean_fold(raws, period), sd = 0;
  for (double r : raws) {sd += fold(r - raw, period) * fold(r - raw, period);}
  sd = std::sqrt(sd / raws.size());
  // sanity (2026-11-15: a live run with no prism under the camera read footprint 26 x 42, top 47 mm, sd 16.6 deg)
  double top = pct(tops, 50), sw = pct(ws, 50), sl = pct(ls, 50);
  std::string bad;
  if (lying ? (top < 22 || top > 38) : (top < 52 || top > 68)) {bad += " top " + std::to_string(static_cast<int>(top)) + " mm";}
  if (lying ? (sw < 20 || sw > 40 || sl < 45 || sl > 70) : (sw < 20 || sl > 38)) {
    bad += " footprint " + std::to_string(static_cast<int>(sw)) + "x" + std::to_string(static_cast<int>(sl)) + " mm";}
  if (!centre_only && sd > (lying ? 8.0 : 6.0)) {bad += " sd " + std::to_string(static_cast<int>(sd)) + " deg";}
  if (raws.size() * 2 < D.size()) {bad += " only " + std::to_string(raws.size()) + "/" + std::to_string(D.size()) + " frames";}
  if (!bad.empty()) {printf("%s: YAW NONE (depth: not a %s prism:%s; %.0f ms)\n", src.c_str(), lying ? "lying" : "standing", bad.c_str(), ms); return 1;}
  double off = 0, sign = 1; bool cal = read_cal(off, sign, lying ? "lying_" : "");
  double yaw = centre_only ? 0 : fold(sign * (raw - off), period);
  double tilt = std::acos(-pl.n.z) * 180 / M_PI;
  // standing: "YAW" (execute.py's standing flows parse ": YAW "); lying: "LYING YAW" (yaw of the long axis, 180 period)
  // FLOOR: the top-face centre dropped onto the surface along its normal (2026-11-15): the same point for a standing
  // and a lying puck on the same spot, so the standing calibration serves the lying one
  double cx = pct(cxs, 50), cy = pct(cys, 50), cz = pct(czs, 50);
  double fx = cx - pl.n.x * top, fy = cy - pl.n.y * top, fz = cz - pl.n.z * top;
  printf("%s: %sYAW %+.1f deg CENTRE %+.1f %+.1f %.1f mm FLOOR %+.1f %+.1f %.1f mm (depth: raw %+.1f, footprint %.0f x %.0f mm, top %.0f mm, %zu/%zu frames, "
         "sd %.1f deg, tilt %.1f, %s, %.0f ms)\n", src.c_str(), lying ? "LYING " : "", yaw, cx, cy, cz, fx, fy, fz,
         raw, sw, sl, top, raws.size(), D.size(), sd, tilt, cal ? "cal" : (lying ? "NO lying cal" : "NO grip_yaw.json"), ms);
  if (centre_only) printf("VIZ_SHAPE cylinder\n");
  // Uncalibrated geometric footprint axis in the colour optical frame, for world-frame visualization.
  V3 vx, vy; axes(pl.n, vx, vy);
  const double rad = raw * M_PI / 180.0;
  const V3 direction = vx * std::cos(rad) + vy * std::sin(rad);
  printf("VIZ_AXIS %.6f %.6f %.6f\n", direction.x, direction.y, direction.z);
  return 0;
} catch (const std::exception & e) {
  std::cout << "YAW NONE (error: " << e.what() << ")\n";
  return 1;
}
