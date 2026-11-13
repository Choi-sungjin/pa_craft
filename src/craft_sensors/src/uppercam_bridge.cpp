// Upper-cam / LiDAR box sensor bridge, C++ port of demo/corner/uppercam_sensor_bridge.py (2026-11-12).
// Real sensors -> the planner's box observation contract; never publishes motor commands.
//
// Same topics, node name, messages and decisions as the Python bridge:
//   in : /planning/box_observation_request (BoxObservationRequest, transient local), <scan_topic> (LaserScan), TF
//   out: /perception/box/observation (answers the current request, map frame)
//        /perception/uppercam/observation (every camera frame, base_link frame, request_id 0)
//        /perception/uppercam/diagnostics (JSON per camera result while a camera request runs)
//        /perception/uppercam/image_raw/compressed (JPEG preview at preview_hz, only with a subscriber)
//   in : /perception/uppercam/hires (std_msgs/Bool, transient local; 2026-11-14): true = colour at hires_size (the
//        mission barcode, read only at 1080p), false = color_size. The pipeline restarts on a change (~1 s gap).
// Differences: the geometry runs in this process on the camera thread (C++ is ~4x faster than the Python workers, so
// no worker pool); the Python file-hash manifest check is gone (it pinned the Python files; this port was checked
// against them on boxgap_1108_1236 instead: same detections, holdout MAE 1.41 vs 1.43 mm); box_depth reproject and
// the fast5 estimator, both unused, are not ported.
#include "uppercam_geometry.hpp"

#include <craft_interfaces/msg/box_observation.hpp>
#include <craft_interfaces/msg/box_observation_request.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <regex>
#include <sstream>
#include <thread>

using craft_interfaces::msg::BoxObservation;
using craft_interfaces::msg::BoxObservationRequest;
using Request = std::shared_ptr<const BoxObservationRequest>;

namespace
{

constexpr const char * kNoBlob = "no level blob of box-top size at box-top height";

double sec(const builtin_interfaces::msg::Time & t) { return t.sec + t.nanosec * 1e-9; }
int64_t nsec(const builtin_interfaces::msg::Time & t) { return t.sec * 1000000000LL + t.nanosec; }
builtin_interfaces::msg::Time to_msg(double s) { return rclcpp::Time(static_cast<int64_t>(std::max(0.0, s * 1e9))).operator builtin_interfaces::msg::Time(); }
double yaw(const geometry_msgs::msg::Quaternion & q) { return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); }
Eigen::Vector3d planar_pose(const geometry_msgs::msg::Transform & t) { return {t.translation.x, t.translation.y, yaw(t.rotation)}; }
Eigen::Matrix3d rotation(const geometry_msgs::msg::Quaternion & q) { return Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix(); }

std::string json_str(const std::string & s)
{
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (static_cast<unsigned char>(c) < 0x20) { char b[8]; std::snprintf(b, sizeof(b), "\\u%04x", c); o += b; }
    else o += c;
  }
  return o + "\"";
}
template<typename T> std::string json_num(const std::optional<T> & v)
{
  if (!v || !std::isfinite(static_cast<double>(*v))) return "null";
  std::ostringstream s;
  s.precision(12);
  s << *v;
  return s.str();
}

double json_number(const std::string & js, const std::string & key)
{
  std::smatch m;
  if (!std::regex_search(js, m, std::regex("\"" + key + "\"\\s*:\\s*([-+0-9.eE]+)"))) throw std::runtime_error("policy has no " + key);
  return std::stod(m[1]);
}

cv::Size parse_size(const std::string & text, const std::string & name)
{
  int w = 0, h = 0;
  char x = 0;
  std::istringstream is(text);
  if (!(is >> w >> x >> h) || (x != 'x' && x != 'X')) throw std::runtime_error(name + " must be WxH, e.g. 848x480");
  if (!(0 < w && w <= 1920 && 0 < h && h <= 1080)) throw std::runtime_error(name + " out of range");
  return {w, h};
}

// One camera result as the bridge sees it: an estimator result, or a camera / timestamp failure (Python: a bare dict
// {'valid': False, 'state': ..., 'error': ...} without the estimator's keys).
struct CameraItem
{
  uppercam::Estimate e;
  bool estimator{true};
};

// What publish() puts into a BoxObservation.
struct Obs
{
  bool valid{false};
  std::string reason;
  bool extrapolated{false}, face_centre_observed{false};
  double uncertainty_m{1.0}, gap_m{0}, lateral_m{0}, yaw_rad{0};
  Eigen::Vector3d box_map{Eigen::Vector3d::Zero()};
};

}  // namespace

class SensorBridge : public rclcpp::Node
{
public:
  SensorBridge() : Node("box_sensor_bridge")
  {
    urdf_ = declare_parameter<std::string>("urdf");
    const auto calibration = declare_parameter<std::string>("calibration");
    const auto policy = declare_parameter<std::string>("policy");
    serial_ = declare_parameter<std::string>("serial", "261322075477");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/lidar/merged");
    square_enabled_ = declare_parameter<bool>("enable_validated_square", false);
    camera_timeout_ = declare_parameter<double>("camera_timeout", 0.30);
    camera_front_face_x_ = declare_parameter<double>("camera_front_face_x", 0.165);
    camera_box_side_ = declare_parameter<double>("camera_box_side", 0.205);
    depth_size_ = parse_size(declare_parameter<std::string>("depth_size", "424x240"), "depth size");
    color_size_ = parse_size(declare_parameter<std::string>("color_size", "640x360"), "color size");
    const auto box = declare_parameter<std::string>("box_size", "");  // estimator grid, same aspect; default colour size
    box_size_ = box.empty() ? color_size_ : parse_size(box, "box size");
    // Colour size while /perception/uppercam/hires is true; "" = no switching (color_size always).
    const auto hires = declare_parameter<std::string>("hires_size", "");
    hires_size_ = hires.empty() ? color_size_ : parse_size(hires, "hires size");
    const double preview_hz = declare_parameter<double>("preview_hz", 2.0);
    if (!(camera_timeout_ > 0 && camera_timeout_ <= 0.5)) throw std::runtime_error("camera timeout must be >0 and <=0.5 s");
    if (!std::isfinite(camera_front_face_x_) || camera_front_face_x_ < 0 || !std::isfinite(camera_box_side_) || camera_box_side_ <= 0)
      throw std::runtime_error("invalid camera reference geometry");
    for (const auto & cs : {color_size_, hires_size_})
      if (!(box_size_.width <= cs.width && box_size_.height <= cs.height) ||
          std::abs(box_size_.width * cs.height - box_size_.height * cs.width) > std::max(cs.width, cs.height))
        throw std::runtime_error("box size must be <= colour and hires size with the same aspect");
    preview_period_ = 1.0 / preview_hz;
    if (!(std::isfinite(preview_period_) && 1.0 / 30 <= preview_period_ && preview_period_ <= 10.0)) throw std::runtime_error("preview hz must be 0.1..30");
    {
      std::ifstream f(policy);
      std::stringstream ss;
      ss << f.rdbuf();
      const std::string js = ss.str();
      lidar_min_ = json_number(js, "operational_raw_min_m");
      lidar_max_ = json_number(js, "operational_raw_max_m");
      if (!(0 < json_number(js, "spec_range_min_m") && json_number(js, "spec_range_min_m") <= lidar_min_ && lidar_min_ < lidar_max_ &&
            lidar_max_ <= json_number(js, "spec_range_max_m")))
        throw std::runtime_error("invalid LiDAR policy");
    }
    mount_ = uppercam::load_camera_mount(urdf_);
    estimator_ = std::make_unique<uppercam::SquareFar20Estimator>(uppercam::load_calibration(calibration));

    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    pub_ = create_publisher<BoxObservation>("/perception/box/observation", 10);
    camera_pub_ = create_publisher<BoxObservation>("/perception/uppercam/observation", 10);
    diag_ = create_publisher<std_msgs::msg::String>("/perception/uppercam/diagnostics", 10);
    preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/perception/uppercam/image_raw/compressed", rclcpp::SensorDataQoS());
    request_sub_ = create_subscription<BoxObservationRequest>(
      "/planning/box_observation_request", rclcpp::QoS(1).transient_local(),
      [this](BoxObservationRequest::ConstSharedPtr m) { receive_request(m); });
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::LaserScan::ConstSharedPtr s) { scan(s); });
    // Box finder (2026-11-13): planning's expected box (/planning/box_search) -> the box as found among the LiDAR
    // points off the map (/perception/lidar/obstacle, map frame) -> /perception/box/pose for planning's restage.
    found_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/perception/box/pose", 10);
    search_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/box_search", rclcpp::QoS(1).transient_local(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        const auto & q = m->pose.orientation;
        search_ = Eigen::Vector3d(m->pose.position.x, m->pose.position.y,
                                  std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
      });
    obstacle_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/perception/lidar/obstacle", 10, [this](geometry_msgs::msg::PoseArray::ConstSharedPtr m) { find_box(*m); });
    hires_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/perception/uppercam/hires", rclcpp::QoS(1).transient_local(), [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data != want_hires_.exchange(m->data))
          RCLCPP_INFO(get_logger(), "colour %s requested", m->data ? "hires" : "normal");
      });
    drain_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { drain(); });
    lidar_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { retry_lidar(); });
    preview_thread_ = std::thread([this] { preview_worker(); });
    camera_thread_ = std::thread([this] { camera_worker(); });
  }

  ~SensorBridge() override
  {
    quit_ = true;
    preview_cv_.notify_all();
    if (camera_thread_.joinable()) camera_thread_.join();
    if (preview_thread_.joinable()) preview_thread_.join();
  }

private:
  double now_s() { return get_clock()->now().nanoseconds() * 1e-9; }

  Request current()
  {
    std::lock_guard<std::mutex> l(lock_);
    return request_;
  }

  void clear_request_state()  // under lock_
  {
    last_lidar_stamp_ns_.reset();
    last_camera_stamp_ns_.reset();
    pending_lidar_.clear();
  }

  void receive_request(const Request & m)
  {
    if (m->source == "off") {  // stop is unconditional, including default/empty geometry and an old ID
      std::lock_guard<std::mutex> l(lock_);
      highest_request_id_ = highest_request_id_ ? std::max(*highest_request_id_, m->request_id) : m->request_id;
      request_ = m;
      clear_request_state();
      return;
    }
    if ((m->source != "lidar" && m->source != "uppercam") || m->header.frame_id != "map") return;
    const auto & p = m->expected_box.position;
    const auto & q = m->expected_box.orientation;
    for (double v : {m->box_side, m->front_face_x, p.x, p.y, q.x, q.y, q.z, q.w}) if (!std::isfinite(v)) return;
    if (m->box_side <= 0) return;
    std::lock_guard<std::mutex> l(lock_);
    if (highest_request_id_ && m->request_id <= *highest_request_id_) return;
    highest_request_id_ = m->request_id;
    request_ = m;
    clear_request_state();
  }

  void publish(const Request & req, double stamp_s, Obs o, const builtin_interfaces::msg::Time * stamp_msg = nullptr)
  {
    const Request cur = current();
    if (!cur || cur->request_id != req->request_id || cur->source != req->source) return;
    BoxObservation m;
    m.header.frame_id = "map";
    m.header.stamp = stamp_msg ? *stamp_msg : to_msg(stamp_s);
    m.request_id = req->request_id;
    m.source = req->source;
    const double timeout = req->source == "uppercam" ? camera_timeout_ : 0.20;
    if (!uppercam::acquisition_valid(stamp_s, now_s(), sec(req->header.stamp), timeout)) {
      o.valid = false;
      if (!(req->source == "uppercam" && stamp_s == 0)) o.reason = "acquisition stale, future, or predates request";
    }
    for (double * v : {&o.gap_m, &o.lateral_m, &o.yaw_rad})
      if (!std::isfinite(*v)) { *v = 0; o.valid = false; o.reason = "nonfinite measurement"; }
    m.valid = o.valid;
    m.reason = o.reason;
    m.extrapolated = o.extrapolated;
    m.face_centre_observed = o.face_centre_observed;
    m.gap_uncertainty_m = o.uncertainty_m;
    m.gap_m = o.gap_m;
    m.lateral_m = o.lateral_m;
    m.yaw_rad = o.yaw_rad;
    m.box_pose.position.x = o.box_map.x();
    m.box_pose.position.y = o.box_map.y();
    m.box_pose.orientation.z = std::sin(o.box_map.z() / 2);
    m.box_pose.orientation.w = std::cos(o.box_map.z() / 2);
    pub_->publish(m);
  }

  Obs invalid(const std::string & reason) { Obs o; o.reason = reason; return o; }

  // ---- LiDAR -------------------------------------------------------------------------------------------------------
  void scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & s)
  {
    const Request req = current();
    if (!req || req->source != "lidar") return;
    const double stamp = sec(s->header.stamp);
    const int64_t stamp_ns = nsec(s->header.stamp);
    {
      std::lock_guard<std::mutex> l(lock_);
      if (request_ != req || (last_lidar_stamp_ns_ && stamp_ns <= *last_lidar_stamp_ns_)) return;
    }
    if (!uppercam::acquisition_valid(stamp, now_s(), sec(req->header.stamp), 0.20)) {
      // Invalid acquisition stamps do not advance the accepted-scan watermark.
      { std::lock_guard<std::mutex> l(lock_); pending_lidar_.clear(); }
      publish(req, stamp, invalid("stale scan"), &s->header.stamp);
      return;
    }
    bool overflow;
    {
      std::lock_guard<std::mutex> l(lock_);
      if (request_ != req || (last_lidar_stamp_ns_ && stamp_ns <= *last_lidar_stamp_ns_)) return;
      last_lidar_stamp_ns_ = stamp_ns;
      // Do not block the executor waiting for TF: its subscription must run too.
      overflow = pending_lidar_.size() >= 4;
      if (overflow) pending_lidar_.clear();
      else pending_lidar_.push_back({req, s, stamp});
    }
    if (overflow) {
      publish(req, stamp, invalid("lidar TF wait queue overflow"), &s->header.stamp);
      return;
    }
    retry_lidar();
  }

  void retry_lidar()
  {
    while (true) {
      PendingScan p;
      {
        std::lock_guard<std::mutex> l(lock_);
        if (pending_lidar_.empty()) return;
        p = pending_lidar_.front();
        if (request_ != p.req) { pending_lidar_.pop_front(); continue; }
      }
      const auto & s = *p.scan;
      if (!uppercam::acquisition_valid(p.stamp, now_s(), sec(p.req->header.stamp), 0.20)) {
        pop_pending();
        publish(p.req, p.stamp, invalid("lidar TF unavailable before acquisition deadline"), &s.header.stamp);
        continue;
      }
      geometry_msgs::msg::Transform tf, base;
      try {
        tf = buffer_->lookupTransform("map", s.header.frame_id, rclcpp::Time(s.header.stamp)).transform;
        base = buffer_->lookupTransform("map", "base_link", rclcpp::Time(s.header.stamp)).transform;
      } catch (const tf2::TransformException &) {
        return;
      }
      pop_pending();
      publish(p.req, p.stamp, lidar_observation(*p.req, s, tf, base), &s.header.stamp);
    }
  }

  void pop_pending()
  {
    std::lock_guard<std::mutex> l(lock_);
    if (!pending_lidar_.empty()) pending_lidar_.pop_front();
  }

  Obs lidar_observation(const BoxObservationRequest & req, const sensor_msgs::msg::LaserScan & s,
                        const geometry_msgs::msg::Transform & tf, const geometry_msgs::msg::Transform & base)
  {
    const Eigen::Matrix3d R = rotation(tf.rotation);
    const Eigen::Vector3d t(tf.translation.x, tf.translation.y, tf.translation.z);
    std::vector<double> ranges(s.ranges.begin(), s.ranges.end()), angles(ranges.size());
    std::vector<Eigen::Vector2d> pts;
    // Conservative operating bound, distinct from the LDS03 specification (50 mm..12 m).
    const double lo = std::max<double>(s.range_min, lidar_min_), hi = std::min<double>(s.range_max, lidar_max_);
    for (std::size_t i = 0; i < ranges.size(); ++i) {
      angles[i] = s.angle_min + i * s.angle_increment;
      if (std::isfinite(ranges[i]) && ranges[i] >= lo && ranges[i] <= hi)
        pts.push_back((R * Eigen::Vector3d(ranges[i] * std::cos(angles[i]), ranges[i] * std::sin(angles[i]), 0) + t).head<2>());
    }
    const auto & p = req.expected_box.position;
    const Eigen::Vector3d robot = planar_pose(base);
    // The face is looked for within 20 deg of the expected yaw; a box turned further (45 deg, 2026-11-10) is found
    // with the yaw turned 40 deg either way (faces repeat every 90 deg, so these three cover every turn).
    // All three tried, the face with the most points wins (2026-11-13, S2 turned ~25 deg: the 0 deg try took 12 cm of
    // the side face before the -40 deg one could take the whole front face).
    std::optional<uppercam::FaceFit> fit;
    for (double turn : {0.0, 40 * M_PI / 180, -40 * M_PI / 180}) {
      auto f = uppercam::fit_box_face(pts, {p.x, p.y, yaw(req.expected_box.orientation) + turn}, robot, req.box_side, req.front_face_x);
      if (f && (!fit || f->inliers > fit->inliers)) fit = f;
    }
    if (!fit) return invalid("no unambiguous LiDAR box face");
    Obs o;
    o.valid = true;
    o.reason = "lidar face fit";
    o.box_map = {fit->box_center_map.x(), fit->box_center_map.y(), fit->box_yaw};
    o.gap_m = fit->gap_m;
    o.lateral_m = fit->lateral_m;
    o.yaw_rad = fit->yaw_relative;
    o.uncertainty_m = std::max(0.010, fit->rmse_m);
    // Both face ends in the raw beams: the box centre along the face measured, not the request's prior (2026-11-10).
    // /lidar/merged (2026-11-14) bins every 0.5 deg, finer than the LiDARs: every other bin is empty (NaN), and the
    // face end search reads a NaN as a beam straddling the edge, so the face came out split into ~11 runs and its ends
    // never counted. The ends are looked for among the returns only.
    std::vector<double> rr, ra;
    for (std::size_t i = 0; i < ranges.size(); ++i) if (std::isfinite(ranges[i])) { rr.push_back(ranges[i]); ra.push_back(angles[i]); }
    auto ends = uppercam::observe_face_endpoints(rr, ra, R, t, *fit, req.box_side, s.range_min, s.range_max);
    // Seen obliquely (a turned box), the side face's start passes for a face end and the centre comes out cm off
    // (2026-11-10: 29 deg, 6 cm): the ends count only with the face within 20 deg of facing the laser.
    if (ends.ok) {
      const Eigen::Vector2d v = t.head<2>() - ends.face_centre_map;
      if (fit->face_normal_map.dot(v) < std::cos(20 * M_PI / 180) * v.norm()) { ends.ok = false; ends.reason = "face seen obliquely, ends not used"; }
    }
    if (ends.ok) {
      o.box_map = {ends.box_center_map.x(), ends.box_center_map.y(), fit->box_yaw};
      o.face_centre_observed = true;
      o.lateral_m = (ends.face_centre_map - robot.head<2>()).dot(Eigen::Vector2d(-std::sin(robot.z()), std::cos(robot.z())));
      char b[96];
      std::snprintf(b, sizeof(b), "lidar face fit, both face ends seen (width %.1f mm)", ends.width_m * 1000);
      o.reason = b;
    } else {
      o.reason = "lidar face fit; centre: prior (" + ends.reason + ")";
    }
    return o;
  }

  // ---- camera ------------------------------------------------------------------------------------------------------
  void enqueue(CameraItem item)
  {
    std::lock_guard<std::mutex> l(results_lock_);
    if (results_.size() >= 2) results_.pop_front();
    results_.push_back(std::move(item));
  }

  void enqueue_failure(const std::string & state, const std::string & error = "")
  {
    CameraItem it;
    it.estimator = false;
    it.e.state = state;
    it.e.error = error;
    it.e.capture_stamp_s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    enqueue(std::move(it));
  }

  void offer_preview(const rs2::video_frame & colour)
  {
    try {
      if (!preview_pub_->get_subscription_count()) return;
      const double mono = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      if (mono - preview_last_at_ < preview_period_) return;
      const auto domain = colour.get_frame_timestamp_domain();
      const double stamp = colour.get_timestamp() / 1000.0;
      if (domain != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME && domain != RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME) return;
      if (!uppercam::acquisition_valid(stamp, now_s(), 0.0, camera_timeout_)) return;
      std::lock_guard<std::mutex> l(preview_lock_);
      if (preview_frame_) return;  // the encoder is still busy: drop this one
      preview_frame_ = {stamp, cv::Mat(colour.get_height(), colour.get_width(), CV_8UC3, const_cast<void *>(colour.get_data())).clone()};
      preview_last_at_ = mono;
      preview_cv_.notify_one();
    } catch (const std::exception &) {
    }
  }

  void preview_worker()
  {
    double published_at = -1e300;
    while (!quit_) {
      std::pair<double, cv::Mat> f;
      {
        std::unique_lock<std::mutex> l(preview_lock_);
        preview_cv_.wait_for(l, std::chrono::milliseconds(100), [this] { return quit_ || preview_frame_.has_value(); });
        if (!preview_frame_) continue;
        f = std::move(*preview_frame_);
        preview_frame_.reset();
      }
      try {
        const double mono = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (mono - published_at < preview_period_) continue;
        std::vector<uchar> jpeg;
        if (!cv::imencode(".jpg", f.second, jpeg, {cv::IMWRITE_JPEG_QUALITY, 80}) || !preview_pub_->get_subscription_count()) continue;
        if (!uppercam::acquisition_valid(f.first, now_s(), 0.0, camera_timeout_)) continue;
        sensor_msgs::msg::CompressedImage msg;
        msg.header.frame_id = "camera_top_color_optical_frame";
        msg.header.stamp = to_msg(f.first);
        msg.format = "jpeg";
        msg.data = std::move(jpeg);
        preview_pub_->publish(msg);
        published_at = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      } catch (const std::exception &) {
      }
    }
  }

  void camera_worker()
  {
    while (!quit_) {
      std::vector<std::pair<rs2::sensor, float>> global_options;
      std::optional<rs2::pipeline> pipe;
      auto release = [&] {
        if (pipe) { try { pipe->stop(); } catch (const std::exception &) {} pipe.reset(); }
        for (auto & [sensor, value] : global_options) { try { sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, value); } catch (const std::exception &) {} }
        global_options.clear();
      };
      try {
        rs2::context ctx;
        std::optional<rs2::device> device;
        for (auto && d : ctx.query_devices())
          if (d.supports(RS2_CAMERA_INFO_SERIAL_NUMBER) && serial_ == d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) device = d;
        if (!device) throw std::runtime_error("upper cam " + serial_ + " not connected");
        for (auto && sensor : device->query_sensors())
          if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
            global_options.emplace_back(sensor, sensor.get_option(RS2_OPTION_GLOBAL_TIME_ENABLED));
            sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f);
          }
        pipe.emplace(ctx);
        rs2::config cfg;
        cfg.enable_device(serial_);
        // Depth is aligned to colour, so the estimator's input grid is the colour size (or box_size below it).
        cfg.enable_stream(RS2_STREAM_DEPTH, depth_size_.width, depth_size_.height, RS2_FORMAT_Z16, 30);
        const bool hires = want_hires_.load();
        const cv::Size cs = hires ? hires_size_ : color_size_;
        cfg.enable_stream(RS2_STREAM_COLOR, cs.width, cs.height, RS2_FORMAT_BGR8, 30);
        const auto profile = pipe->start(cfg);
        RCLCPP_INFO(get_logger(), "upper cam colour %dx%d", cs.width, cs.height);
        const double scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
        rs2::align align(RS2_STREAM_COLOR);
        estimator_->reset();
        while (!quit_ && want_hires_.load() == hires) {
          rs2::frameset frames;
          if (!pipe->try_wait_for_frames(&frames, 100)) continue;
          const rs2::video_frame colour = frames.get_color_frame();
          const rs2::depth_frame depth = frames.get_depth_frame();
          if (!colour || !depth) continue;
          const double stamp = depth.get_timestamp() / 1000.0;
          const auto domain = depth.get_frame_timestamp_domain();
          if (domain != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME && domain != RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME) {
            enqueue_failure("unmapped_hardware_timestamp");
            continue;
          }
          offer_preview(colour);
          // Hires (barcode) frames: no depth align / box estimate. Aligning depth to 1080p on every frame made the
          // frames too old for the 0.3 s preview check: no preview at all (robot 2026-11-14). The box is not
          // observed while the barcode is read.
          if (hires) continue;
          const long index = static_cast<long>(depth.get_frame_number());
          const auto aligned = align.process(frames);
          const rs2::depth_frame d = aligned.get_depth_frame();
          const auto intr = aligned.get_color_frame().get_profile().as<rs2::video_stream_profile>().get_intrinsics();
          cv::Mat frame;
          cv::Mat(d.get_height(), d.get_width(), CV_16U, const_cast<void *>(d.get_data())).convertTo(frame, CV_64F, scale);
          Eigen::Matrix3d K;
          K << intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1;
          if (box_size_ != frame.size()) {  // align_nearest: high-res aligned depth -> box grid (pixel centres)
            const double sx = static_cast<double>(box_size_.width) / frame.cols, sy = static_cast<double>(box_size_.height) / frame.rows;
            cv::Mat small;
            cv::resize(frame, small, box_size_, 0, 0, cv::INTER_NEAREST_EXACT);
            frame = small;
            K(0, 0) *= sx; K(1, 1) *= sy;
            K(0, 2) = (K(0, 2) + 0.5) * sx - 0.5; K(1, 2) = (K(1, 2) + 0.5) * sy - 0.5;
          }
          const auto begin = std::chrono::steady_clock::now();
          std::optional<uppercam::Geometry> geo;
          std::string error;
          try {
            geo = uppercam::square_far20_geometry(frame, K, mount_);
          } catch (const std::exception & e) {
            error = e.what();
          }
          CameraItem it;
          it.e = estimator_->estimate(geo, error, index, stamp);
          it.e.processing_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
          enqueue(std::move(it));
        }
      } catch (const std::exception & e) {
        enqueue_failure("camera_exception", e.what());
        release();
        estimator_->reset();
        for (int i = 0; i < 10 && !quit_; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      release();
    }
  }

  // ---- results -----------------------------------------------------------------------------------------------------
  static bool no_blob(const CameraItem & it)
  {
    return it.e.state == "invalid" && !it.e.valid && it.e.error == kNoBlob;
  }

  void publish_continuous(const CameraItem & it, double now)
  {
    const auto & e = it.e;
    const bool usable = e.source_stamp_s && std::isfinite(*e.source_stamp_s) && *e.source_stamp_s > 0;
    const bool blob_missing = no_blob(it);
    const double capture = e.capture_stamp_s;
    const double stamp = usable ? *e.source_stamp_s : (blob_missing && std::isfinite(capture) && capture > 0 ? capture : 0.0);
    const bool future = stamp > now;
    if (stamp > 0) {
      const int64_t ns = static_cast<int64_t>(stamp * 1e9);
      if (future) {
        if (last_future_stamp_ns_ == ns) return;
        last_future_stamp_ns_ = ns;
      } else {
        if (last_continuous_stamp_ns_ && ns <= *last_continuous_stamp_ns_) return;
        last_continuous_stamp_ns_ = ns;
      }
    }
    BoxObservation m;
    m.header.frame_id = "base_link";
    m.header.stamp = to_msg(stamp);
    m.request_id = 0;
    m.source = "uppercam";
    m.extrapolated = e.extrapolated;
    auto policy = uppercam::square_observation_policy(e, square_enabled_);
    if (!usable) {
      policy.valid = false;
      policy.reason = blob_missing ? "uppercam_geometry_not_detected" : "camera source timestamp missing or invalid: " + e.state;
    } else if (future || !uppercam::acquisition_valid(stamp, now, 0.0, camera_timeout_)) {
      policy.valid = false;
      policy.reason = "camera source timestamp stale or future";
    }
    m.valid = policy.valid;
    m.reason = policy.reason;
    m.gap_uncertainty_m = policy.uncertainty_m;
    // As the Python bridge: an estimator result without a filtered gap (warm-up, outlier, no geometry) ends here as
    // "nonfinite camera geometry" (float(None) there); lateral / yaw default to 0 when absent.
    if (it.estimator && !e.gap_mm) { m.valid = false; m.reason = "nonfinite camera geometry"; }
    m.gap_m = e.gap_mm.value_or(0.0) * 0.001;
    m.lateral_m = e.lateral_m.value_or(0.0);
    m.yaw_rad = e.box_yaw_rad.value_or(0.0);
    if (!std::isfinite(m.gap_m) || !std::isfinite(m.lateral_m) || !std::isfinite(m.yaw_rad)) {
      m.valid = false; m.reason = "nonfinite camera geometry";
      if (!std::isfinite(m.gap_m)) m.gap_m = 0;
      if (!std::isfinite(m.lateral_m)) m.lateral_m = 0;
      if (!std::isfinite(m.yaw_rad)) m.yaw_rad = 0;
    }
    m.box_pose.orientation.w = 1.0;
    if (m.valid) {
      try {
        const auto b = uppercam::camera_box_in_map(m.gap_m, m.lateral_m, m.yaw_rad, Eigen::Vector3d::Zero(), camera_front_face_x_, camera_box_side_);
        m.box_pose.position.x = b.x();
        m.box_pose.position.y = b.y();
        m.box_pose.orientation.z = std::sin(b.z() / 2);
        m.box_pose.orientation.w = std::cos(b.z() / 2);
      } catch (const std::exception &) {
        m.valid = false;
        m.reason = "invalid camera pose geometry";
      }
    }
    camera_pub_->publish(m);
  }

  void find_box(const geometry_msgs::msg::PoseArray & m)
  {
    if (!search_ || m.header.frame_id != "map" || m.poses.empty()) return;
    Eigen::Vector2d robot;
    try {
      const auto t = buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform.translation;
      robot = {t.x, t.y};
    } catch (const tf2::TransformException &) { return; }
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(m.poses.size());
    for (const auto & p : m.poses) pts.emplace_back(p.position.x, p.position.y);
    const auto b = uppercam::find_box(pts, *search_, robot, camera_box_side_);
    if (!b) return;
    geometry_msgs::msg::PoseStamped out;
    out.header = m.header;
    out.pose.position.x = b->x;
    out.pose.position.y = b->y;
    out.pose.orientation.z = std::sin(b->yaw / 2);
    out.pose.orientation.w = std::cos(b->yaw / 2);
    found_pub_->publish(out);
  }

  void diagnostic(const CameraItem & it, uint64_t request_id, std::optional<double> capture_age_ms, std::optional<double> source_age_ms)
  {
    const auto & e = it.e;
    std::ostringstream s;
    s << "{\"valid\": " << (e.valid ? "true" : "false") << ", \"state\": " << json_str(e.state)
      << ", \"gap_mm\": " << json_num(e.gap_mm) << ", \"raw_gap_mm\": " << json_num(e.raw_gap_mm)
      << ", \"lateral_m\": " << json_num(e.lateral_m) << ", \"box_yaw_rad\": " << json_num(e.box_yaw_rad)
      << ", \"stamp_s\": " << json_num(std::optional<double>(e.capture_stamp_s))
      << ", \"capture_stamp_s\": " << json_num(std::optional<double>(e.capture_stamp_s))
      << ", \"source_stamp_s\": " << json_num(e.source_stamp_s) << ", \"frame_index\": " << e.frame_index
      << ", \"source_frame_index\": " << json_num(e.source_frame_index)
      << ", \"extrapolated\": " << (e.extrapolated ? "true" : "false")
      << ", \"current_extrapolated\": " << (e.current_extrapolated ? "true" : "false")
      << ", \"processing_ms\": " << json_num(std::optional<double>(e.processing_ms))
      << ", \"request_id\": " << request_id << ", \"capture_age_ms\": " << json_num(capture_age_ms)
      << ", \"source_age_ms\": " << json_num(source_age_ms) << ", \"method\": " << json_str(uppercam::kMethod)
      << ", \"provenance\": \"spec_reimplementation\", \"frame_id\": \"camera_top_color_optical_frame\""
      << ", \"geometry_frame_id\": \"base_link\"";
    if (!e.error.empty()) s << ", \"error\": " << json_str(e.error);
    if (e.geometry) {
      const auto & g = *e.geometry;
      s << ", \"far_inliers\": " << g.far_inliers << ", \"side_inliers\": " << g.side_inliers
        << ", \"fit_cost\": " << json_num(std::optional<double>(g.fit_cost)) << ", \"plane_rmse_mm\": " << json_num(std::optional<double>(g.plane_rmse_mm))
        << ", \"top_area_m2\": " << json_num(std::optional<double>(g.top_area_m2)) << ", \"flags\": [";
      for (std::size_t i = 0; i < g.flags.size(); ++i) s << (i ? ", " : "") << json_str(g.flags[i]);
      s << "]";
    }
    s << "}";
    std_msgs::msg::String msg;
    msg.data = s.str();
    diag_->publish(msg);
  }

  void drain()
  {
    CameraItem it;
    {
      std::lock_guard<std::mutex> l(results_lock_);
      if (results_.empty()) return;
      it = std::move(results_.front());
      results_.pop_front();
    }
    const double now = now_s();
    publish_continuous(it, now);
    const Request req = current();
    if (!req || req->source != "uppercam") return;
    const auto & e = it.e;
    const double req_s = sec(req->header.stamp), capture = e.capture_stamp_s;
    // A pre-request capture or selected frame is never a new request observation.
    if (std::isfinite(capture) && capture <= req_s) return;
    if (e.source_stamp_s && std::isfinite(*e.source_stamp_s) && *e.source_stamp_s <= req_s) return;
    const bool usable_source = e.source_stamp_s && std::isfinite(*e.source_stamp_s) && *e.source_stamp_s > 0;
    const std::optional<double> capture_age = std::isfinite(capture) ? std::optional<double>((now - capture) * 1000) : std::nullopt;
    const std::optional<double> source_age = usable_source ? std::optional<double>((now - *e.source_stamp_s) * 1000) : std::nullopt;
    if ((usable_source && *e.source_stamp_s > now) || (std::isfinite(capture) && capture > now)) {
      publish(req, 0.0, invalid("camera timestamp future"));
      return;
    }
    diagnostic(it, req->request_id, capture_age, source_age);
    if (e.state == "warmup" || e.state == "warming_up") return;
    // A detected capture with no box-top blob is distinct from camera/TF errors; its capture time is the only
    // trustworthy time when geometry is absent.
    const bool blob_missing = no_blob(it) && !e.extrapolated && !e.current_extrapolated && !e.source_stamp_s &&
                              !e.source_frame_index && !e.gap_mm && !e.raw_gap_mm && !e.lateral_m && !e.box_yaw_rad;
    if (blob_missing && std::isfinite(capture) && capture > 0) {
      std::lock_guard<std::mutex> l(lock_);
      if (request_ != req) return;
      if (last_camera_stamp_ns_ && static_cast<int64_t>(capture * 1e9) <= *last_camera_stamp_ns_) return;
    }
    // Before any camera observation, a stale no-detection startup frame carries no usable geometry: drop it rather
    // than manufacture a stamp=0 failure. The planner stays held and its acquisition deadline still runs.
    if (blob_missing && std::isfinite(capture) && req_s < capture && capture <= now && now - capture > camera_timeout_) {
      std::lock_guard<std::mutex> l(lock_);
      if (request_ == req && !last_camera_stamp_ns_) return;
    }
    const bool usable_capture = blob_missing && std::isfinite(capture) && capture > 0 && req_s < capture && capture <= now &&
                                now - capture <= camera_timeout_;
    if (blob_missing && usable_capture) {
      const int64_t ns = static_cast<int64_t>(capture * 1e9);
      {
        std::lock_guard<std::mutex> l(lock_);
        if (request_ != req || (last_camera_stamp_ns_ && ns <= *last_camera_stamp_ns_)) return;
        last_camera_stamp_ns_ = ns;
      }
      publish(req, capture, invalid("uppercam_geometry_not_detected"));
      return;
    }
    // The same source frame chosen again is not a newer measurement and does not reset the planner deadline.
    double stamp = 0.0;
    if (usable_source) {
      const int64_t ns = static_cast<int64_t>(*e.source_stamp_s * 1e9);
      std::lock_guard<std::mutex> l(lock_);
      if (request_ != req || (last_camera_stamp_ns_ && ns <= *last_camera_stamp_ns_)) return;
      last_camera_stamp_ns_ = ns;
      stamp = *e.source_stamp_s;
    }
    auto policy = uppercam::square_observation_policy(e, square_enabled_);
    if (!usable_source) { policy.valid = false; policy.reason = "camera source timestamp missing or invalid: " + e.state; }
    Obs o;
    o.valid = policy.valid;
    o.reason = policy.reason;
    o.extrapolated = e.extrapolated;
    o.uncertainty_m = policy.uncertainty_m;
    if (usable_source && e.gap_mm && e.lateral_m) {
      try {
        o.gap_m = *e.gap_mm / 1000;
        o.lateral_m = *e.lateral_m;
        o.yaw_rad = e.box_yaw_rad.value();
        const auto tf = buffer_->lookupTransform("map", "base_link", rclcpp::Time(static_cast<int64_t>(stamp * 1e9))).transform;
        o.box_map = uppercam::camera_box_in_map(o.gap_m, o.lateral_m, o.yaw_rad, planar_pose(tf), req->front_face_x, req->box_side);
      } catch (const std::exception & ex) {
        o.valid = false;
        o.reason = ex.what();
      }
    }
    publish(req, stamp, o);
  }

  struct PendingScan
  {
    Request req;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    double stamp{};
  };

  // parameters
  std::string urdf_, serial_;
  bool square_enabled_{false};
  double camera_timeout_{}, camera_front_face_x_{}, camera_box_side_{}, preview_period_{}, lidar_min_{}, lidar_max_{};
  cv::Size depth_size_, color_size_, box_size_, hires_size_;
  std::atomic<bool> want_hires_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hires_sub_;
  uppercam::CameraMount mount_;
  std::unique_ptr<uppercam::SquareFar20Estimator> estimator_;  // camera thread only

  // request state (lock_)
  std::mutex lock_;
  Request request_;
  std::optional<uint64_t> highest_request_id_;
  std::optional<int64_t> last_lidar_stamp_ns_, last_camera_stamp_ns_;
  std::deque<PendingScan> pending_lidar_;

  // executor-only state
  std::optional<int64_t> last_continuous_stamp_ns_, last_future_stamp_ns_;

  // camera -> executor
  std::mutex results_lock_;
  std::deque<CameraItem> results_;

  // camera -> preview encoder
  std::mutex preview_lock_;
  std::condition_variable preview_cv_;
  std::optional<std::pair<double, cv::Mat>> preview_frame_;
  double preview_last_at_{-1e300};

  std::atomic<bool> quit_{false};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<BoxObservation>::SharedPtr pub_, camera_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  rclcpp::Subscription<BoxObservationRequest>::SharedPtr request_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr drain_timer_, lidar_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr found_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr search_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  std::optional<Eigen::Vector3d> search_;
  std::thread preview_thread_, camera_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorBridge>());
  rclcpp::shutdown();
}
