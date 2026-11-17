// Upper cam (어퍼캠, D435 261322075477) puck positions, the C++ port of puck_locate.py + cam_pick.py's position part
// (2026-11-15, user: camera parts in C++). Coarse position only: the gripper cam corrects it at the wait pose.
//   upper_puck [--warm N] [--save PREFIX]        live: colour 1280x720 + depth aligned to it
//   upper_puck --snap PREFIX                     a saved rs_snap capture (PREFIX_color.ppm, _depth.u16, _meta.txt)
// One line per puck, best score first:
//   PUCK <class> <score> ahead <mm> lateral <mm> top <mm> px <u> <v> depth <m>
//     ahead   = LiDAR front face -> puck centre, straight ahead [mm] (cam_pick.py puck_ahead)
//     lateral = along the LM axis, + = away from the LM motor [mm] (cam_pick.py puck_x_offset, LATERAL_SIGN)
//     top     = visible surface over the floor [mm]
// or "PUCK NONE". Detector: YOLOv8n pucks_v8n_v2 ONNX through cv::dnn (letterbox top-left, pad 114, 1/255, RGB),
// conf 0.25, NMS 0.45. Depth: median of the box's inner 40 %. The surface point is pushed back by the puck radius
// along the horizontal view ray. Camera pose: upper_cam_pose.txt (exported from puck_locate.camera_pose()).
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace {
const char * kPose = GRIP_YAW_DIR "/upper_cam_pose.txt";

struct Pose {double R[9], t[3], z_floor, laser[3], face, radius, sign; std::string model, serial; std::vector<std::string> cls;};

Pose read_pose() {
  Pose p; std::ifstream f(kPose);
  if (!f) {throw std::runtime_error(std::string("no ") + kPose);}
  for (double & v : p.R) {f >> v;}
  for (double & v : p.t) {f >> v;}
  f >> p.z_floor; for (double & v : p.laser) {f >> v;}
  f >> p.face >> p.radius >> p.sign >> p.model >> p.serial;
  std::string c; while (f >> c) {p.cls.push_back(c);}
  return p;
}

struct Det {cv::Rect2d box; int cls; float score;};

std::vector<Det> detect(cv::dnn::Net & net, const cv::Mat & bgr, float conf = 0.25f, float iou = 0.45f) {
  double s = 640.0 / std::max(bgr.cols, bgr.rows);
  cv::Mat img(640, 640, CV_8UC3, cv::Scalar(114, 114, 114)), r;
  cv::resize(bgr, r, cv::Size(static_cast<int>(bgr.cols * s), static_cast<int>(bgr.rows * s)));
  r.copyTo(img(cv::Rect(0, 0, r.cols, r.rows)));
  net.setInput(cv::dnn::blobFromImage(img, 1 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false));
  cv::Mat out = net.forward();                       // 1 x (4 + C) x 8400
  int rows = out.size[1], n = out.size[2];
  cv::Mat o(rows, n, CV_32F, out.ptr<float>());
  std::vector<cv::Rect2d> boxes; std::vector<float> scores; std::vector<int> cls;
  for (int i = 0; i < n; ++i) {
    int best = 0; float bs = 0;
    for (int c = 4; c < rows; ++c) {float v = o.at<float>(c, i); if (v > bs) {bs = v; best = c - 4;}}
    if (bs < conf) {continue;}
    double cx = o.at<float>(0, i), cy = o.at<float>(1, i), w = o.at<float>(2, i), h = o.at<float>(3, i);
    boxes.emplace_back((cx - w / 2) / s, (cy - h / 2) / s, w / s, h / s); scores.push_back(bs); cls.push_back(best);
  }
  std::vector<int> keep; std::vector<cv::Rect> ib;
  for (auto & b : boxes) {ib.emplace_back(cv::Rect(static_cast<int>(b.x), static_cast<int>(b.y), static_cast<int>(b.width), static_cast<int>(b.height)));}
  cv::dnn::NMSBoxes(ib, scores, conf, iou, keep);
  std::vector<Det> d; for (int k : keep) {d.push_back({boxes[k], cls[k], scores[k]});}
  std::sort(d.begin(), d.end(), [](const Det & a, const Det & b) {return a.score > b.score;});
  return d;
}
}  // namespace

int main(int argc, char ** argv) try {
  auto t0 = std::chrono::steady_clock::now();
  int warm = 15; std::string snap, save;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--warm" && i + 1 < argc) {warm = std::atoi(argv[++i]);}
    else if (a == "--snap" && i + 1 < argc) {snap = argv[++i];}
    else if (a == "--save" && i + 1 < argc) {save = argv[++i];}
  }
  Pose P = read_pose();
  cv::Mat bgr; cv::Mat depth;  // depth [m] aligned to colour
  double fx, fy, ppx, ppy;
  if (!snap.empty()) {
    std::ifstream mf(snap + "_meta.txt"); int w, h; double sc; mf >> w >> h >> fx >> fy >> ppx >> ppy >> sc;
    bgr = cv::imread(snap + "_color.ppm");
    std::vector<uint16_t> buf(w * h);
    std::ifstream(snap + "_depth.u16", std::ios::binary).read(reinterpret_cast<char *>(buf.data()), buf.size() * 2);
    cv::Mat(h, w, CV_16U, buf.data()).convertTo(depth, CV_64F, sc);
  } else {
    rs2::config cfg; cfg.enable_device(P.serial);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe; auto prof = pipe.start(cfg);
    double sc = prof.get_device().first<rs2::depth_sensor>().get_depth_scale();
    rs2::align align(RS2_STREAM_COLOR);
    for (int i = 0; i < warm; ++i) {pipe.wait_for_frames();}  // colour auto exposure
    auto fs = align.process(pipe.wait_for_frames());
    auto c = fs.get_color_frame(); auto d = fs.get_depth_frame();
    auto in = c.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    fx = in.fx; fy = in.fy; ppx = in.ppx; ppy = in.ppy;
    bgr = cv::Mat(in.height, in.width, CV_8UC3, const_cast<void *>(c.get_data())).clone();
    cv::Mat(d.get_height(), d.get_width(), CV_16U, const_cast<void *>(d.get_data())).convertTo(depth, CV_64F, sc);
    pipe.stop();
    if (!save.empty()) {
      cv::imwrite(save + "_color.png", bgr);
      cv::Mat d16; depth.convertTo(d16, CV_16U, 1000.0); cv::imwrite(save + "_depth_mm.png", d16);
    }
  }
  cv::dnn::Net net = cv::dnn::readNetFromONNX(P.model);
  auto dets = detect(net, bgr);
  double face_x = P.laser[0] + P.face;
  int n = 0;
  for (auto & dt : dets) {
    auto & b = dt.box;
    int x0 = std::max(0, static_cast<int>(b.x + 0.3 * b.width)), y0 = std::max(0, static_cast<int>(b.y + 0.3 * b.height));
    int x1 = std::min(depth.cols, static_cast<int>(b.x + 0.7 * b.width)), y1 = std::min(depth.rows, static_cast<int>(b.y + 0.7 * b.height));
    std::vector<double> z;
    for (int v = y0; v < y1; ++v) {for (int u = x0; u < x1; ++u) {double q = depth.at<double>(v, u); if (q > 0.1) {z.push_back(q);}}}
    if (z.empty()) {continue;}
    std::nth_element(z.begin(), z.begin() + z.size() / 2, z.end()); double zm = z[z.size() / 2];
    double u = b.x + b.width / 2, v = b.y + b.height / 2;
    double ray[3] = {(u - ppx) / fx * zm, (v - ppy) / fy * zm, zm}, s[3];
    for (int r = 0; r < 3; ++r) {s[r] = P.R[r * 3] * ray[0] + P.R[r * 3 + 1] * ray[1] + P.R[r * 3 + 2] * ray[2] + P.t[r];}
    double hx = s[0] - P.t[0], hy = s[1] - P.t[1], hl = std::hypot(hx, hy) + 1e-9;
    double cx = s[0] + hx / hl * P.radius, cy = s[1] + hy / hl * P.radius;
    // Surface height around this detection, excluding every detected puck. Use the modal horizontal
    // depth band, not a fixed box height: the robot/camera floor calibration has a constant bias.
    std::vector<double> own, surface;
    int xa = std::max(0, static_cast<int>(b.x-b.width)), xb = std::min(depth.cols, static_cast<int>(b.x+2*b.width));
    int ya = std::max(0, static_cast<int>(b.y-b.height)), yb = std::min(depth.rows, static_cast<int>(b.y+2*b.height));
    for (int v = ya; v < yb; v += 2) for (int u = xa; u < xb; u += 2) {
      const double d = depth.at<double>(v,u); if (d <= .1) continue;
      double ray_[3] = {(u-ppx)/fx*d,(v-ppy)/fy*d,d}, q[3];
      for (int k=0;k<3;++k) q[k]=P.R[k*3]*ray_[0]+P.R[k*3+1]*ray_[1]+P.R[k*3+2]*ray_[2]+P.t[k];
      const double dist=std::hypot(q[0]-cx,q[1]-cy);
      if (b.contains(cv::Point2d(u,v)) && dist < .045) own.push_back(q[2]*1000);
      bool puck=false;
      for (const auto &other:dets) if (other.box.contains(cv::Point2d(u,v))) {puck=true;break;}
      if (!puck && dist>.035 && dist<.13 && q[2]-P.z_floor>.025 && q[2]-P.z_floor<.3)
        surface.push_back(q[2]*1000);
    }
    double height = -1;
    if (surface.size()>=30 && own.size()>=20) {
      std::map<int,int> bins;
      for(double h:surface) ++bins[static_cast<int>(std::floor(h/3))];
      int best=0, count=0;
      for(const auto &[bin,n]:bins) if(n>count) {best=bin;count=n;}
      std::vector<double> level;
      for(double h:surface) if(std::abs(h-(best*3+1.5))<=4.5) level.push_back(h);
      if(level.size()>=30 && level.size()*4>=surface.size()) {
        std::sort(level.begin(),level.end());std::sort(own.begin(),own.end());
        height=own[static_cast<size_t>(.95*(own.size()-1))]-level[level.size()/2];
      }
    }
    const std::string cls = dt.cls < static_cast<int>(P.cls.size()) ? P.cls[dt.cls] : "?";
    const char *posture = cls.find("cylinder")!=std::string::npos ? "standing" :
      height>=45 && height<=75 ? "standing" : height>=20 && height<=40 ? "lying" : "unknown";
    printf("PUCK %s %.2f ahead %.1f lateral %+.1f top %.0f px %.0f %.0f depth %.3f posture %s height %.1f\n",
           dt.cls < static_cast<int>(P.cls.size()) ? P.cls[dt.cls].c_str() : "?", dt.score, (cx - face_x) * 1000,
           P.sign * (cy - P.laser[1]) * 1000, (s[2] - P.z_floor) * 1000, u, v, zm, posture, height);
    ++n;
  }
  double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (!n) {printf("PUCK NONE (%zu boxes, %.0f ms)\n", dets.size(), ms); return 1;}
  printf("# %d pucks, %.0f ms\n", n, ms);
  return 0;
} catch (const std::exception & e) {
  std::cout << "PUCK NONE (error: " << e.what() << ")\n";
  return 1;
}
