// realsense_processor: D435 color + depth (librealsense2) -> depth aligned to color -> shared memory
// pub: /vision/image_offset (std_msgs/UInt64MultiArray [offset, width, height])
//      slot at that offset in /dev/shm/craft_vision: BGR8 (w*h*3) then aligned depth Z16 (w*h*2, 1 unit = 1 mm)
//      (no image copy over DDS; readers that only need color read the first w*h*3 bytes)
// pub: /vision/camera_info (color intrinsics, valid for the aligned depth too)
// pub: /vision/image, /lidar/merged (TODO)

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <librealsense2/rs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

using namespace std::chrono_literals;

constexpr char kShmName[] = "/craft_vision";
// ponytail: ring of 4 slots, no lock. Reader must finish within 3 frames (~100ms @30fps),
// add a seq/lock per slot if a slow reader shows torn frames.
constexpr size_t kSlots = 4;

struct Unmap
{
  size_t len;
  void operator()(uint8_t * p) const {munmap(p, len);}
};

class RealsenseProcessor : public rclcpp::Node
{
public:
  RealsenseProcessor()
  : Node("realsense_processor"), align_(RS2_STREAM_COLOR)
  {
    const std::string serial = declare_parameter("serial", std::string(""));
    w_ = declare_parameter("width", 640);
    h_ = declare_parameter("height", 360);
    const int fps = declare_parameter("fps", 30);
    const int depth_w = declare_parameter("depth_width", 424);
    const int depth_h = declare_parameter("depth_height", 240);

    rs2::config cfg;
    if (!serial.empty()) {
      cfg.enable_device(serial);
    }
    cfg.enable_stream(RS2_STREAM_COLOR, w_, h_, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, depth_w, depth_h, RS2_FORMAT_Z16, fps);
    rs2::pipeline_profile profile = pipe_.start(cfg);  // throws rs2::error if no camera
    auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_DEPTH_UNITS)) {
      depth_sensor.set_option(RS2_OPTION_DEPTH_UNITS, 0.001f);  // Z16 = mm
    }

    color_bytes_ = static_cast<size_t>(w_) * h_ * 3;
    slot_size_ = color_bytes_ + static_cast<size_t>(w_) * h_ * 2;
    const int fd = shm_open(kShmName, O_CREAT | O_RDWR, 0666);
    if (fd < 0 || ftruncate(fd, slot_size_ * kSlots) < 0) {
      throw std::runtime_error("shm_open/ftruncate failed");
    }
    void * p = mmap(nullptr, slot_size_ * kSlots, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED) {
      throw std::runtime_error("mmap failed");
    }
    shm_ = std::unique_ptr<uint8_t, Unmap>(static_cast<uint8_t *>(p), Unmap{slot_size_ * kSlots});

    const auto in = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    camera_info_.header.frame_id = "camera_color_optical_frame";
    camera_info_.width = w_;
    camera_info_.height = h_;
    camera_info_.distortion_model = "plumb_bob";
    camera_info_.d.assign(in.coeffs, in.coeffs + 5);
    camera_info_.k = {in.fx, 0, in.ppx, 0, in.fy, in.ppy, 0, 0, 1};
    camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    camera_info_.p = {in.fx, 0, in.ppx, 0, 0, in.fy, in.ppy, 0, 0, 0, 1, 0};
    RCLCPP_INFO(get_logger(), "D435 color %dx%d + depth %dx%d @%dfps (aligned) -> /dev/shm%s (%zu slots)",
      w_, h_, depth_w, depth_h, fps, kShmName, kSlots);

    vision_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/image", 10);
    vision_image_offset_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>("/vision/image_offset", 1);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/vision/camera_info", 10);
    lidar_merged_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/merged", 10);
    timer_ = create_wall_timer(5ms, [this]() {realsense_loop();});
  }

  ~RealsenseProcessor() override
  {
    pipe_.stop();
    shm_unlink(kShmName);
  }

private:
  void realsense_loop()
  {
    rs2::frameset frames;
    if (!pipe_.poll_for_frames(&frames)) {
      return;
    }
    frames = align_.process(frames);
    const rs2::video_frame color = frames.get_color_frame();
    const rs2::depth_frame depth = frames.get_depth_frame();
    if (!color || !depth || color.get_width() != w_ || depth.get_width() != w_ || depth.get_height() != h_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "missing/mismatched frame, skipped");
      return;
    }

    const size_t offset = slot_ * slot_size_;
    uint8_t * dst = shm_.get() + offset;
    std::memcpy(dst, color.get_data(), color_bytes_);
    std::memcpy(dst + color_bytes_, depth.get_data(), slot_size_ - color_bytes_);
    slot_ = (slot_ + 1) % kSlots;

    auto msg = std::make_unique<std_msgs::msg::UInt64MultiArray>();
    msg->data = {offset, static_cast<uint64_t>(w_), static_cast<uint64_t>(h_)};
    vision_image_offset_pub_->publish(std::move(msg));
    camera_info_.header.stamp = now();
    camera_info_pub_->publish(camera_info_);
    // TODO: depth frame -> /lidar/merged
  }

  rs2::pipeline pipe_;
  rs2::align align_;
  int w_{0}, h_{0};
  size_t color_bytes_{0}, slot_size_{0}, slot_{0};
  std::unique_ptr<uint8_t, Unmap> shm_;
  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vision_image_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr vision_image_offset_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<RealsenseProcessor>());
  } catch (const std::exception & e) {  // rs2::error (no device, ...) / shm failure
    RCLCPP_FATAL(rclcpp::get_logger("realsense_processor"), "%s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
