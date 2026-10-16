// c310_processor: Logitech C310 (V4L2) -> shared memory for object_pose_estimator
// pub: /vision/c310_image_offset (std_msgs/UInt64MultiArray [offset, width, height])
//      BGR frame lives in shared memory /dev/shm/craft_c310 at that offset (no image copy over DDS)
// pub: /vision/c310_image (TODO)

#include <fcntl.h>
#include <glob.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/u_int64_multi_array.hpp"

using namespace std::chrono_literals;

constexpr char kShmName[] = "/craft_c310";
// ponytail: ring of 4 slots, no lock. Reader must finish within 3 frames (~100ms @30fps),
// add a seq/lock per slot if a slow reader shows torn frames.
constexpr size_t kSlots = 4;

struct Unmap
{
  size_t len;
  void operator()(uint8_t * p) const {munmap(p, len);}
};

// "auto" -> first C310 by USB id (046d:081b); videoN changes with plug order
std::string resolve_camera(const std::string & camera)
{
  if (camera != "auto") {
    return camera;
  }
  glob_t g{};
  std::string found;
  if (glob("/dev/v4l/by-id/*046d_081b*-video-index0", 0, nullptr, &g) == 0) {
    found = g.gl_pathv[0];
  }
  globfree(&g);
  if (found.empty()) {
    throw std::runtime_error("C310 not found in /dev/v4l/by-id (plugged in?), set the camera parameter");
  }
  return found;
}

class C310Processor : public rclcpp::Node
{
public:
  C310Processor()
  : Node("c310_processor")
  {
    const std::string camera = resolve_camera(declare_parameter("camera", std::string("auto")));
    const int width = declare_parameter("width", 640);
    const int height = declare_parameter("height", 480);
    const int fps = declare_parameter("fps", 30);
    if (!cap_.open(camera, cv::CAP_V4L2)) {
      throw std::runtime_error("camera open failed: " + camera);
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, fps);
    // driver picks the closest supported mode; the real size goes to shm and /vision/c310_image_offset
    w_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    h_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    if (w_ != width || h_ != height) {
      RCLCPP_WARN(get_logger(), "requested %dx%d, camera gave %dx%d", width, height, w_, h_);
    }
    slot_size_ = static_cast<size_t>(w_) * h_ * 3;

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
    RCLCPP_INFO(get_logger(), "camera %s %dx%d -> /dev/shm%s (%zu slots)", camera.c_str(), w_, h_, kShmName, kSlots);

    vision_c310_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/c310_image", 10);
    vision_c310_image_offset_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>("/vision/c310_image_offset", 1);
    timer_ = create_wall_timer(10ms, [this]() {on_timer();});
  }

  ~C310Processor() override {shm_unlink(kShmName);}

private:
  void on_timer()
  {
    const size_t offset = slot_ * slot_size_;
    uint8_t * dst = shm_.get() + offset;
    cv::Mat frame(h_, w_, CV_8UC3, dst);  // decode straight into shared memory
    if (!cap_.read(frame)) {
      return;
    }
    if (frame.data != dst) {  // driver changed size/format -> OpenCV reallocated
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "frame size changed, skipped");
      return;
    }
    slot_ = (slot_ + 1) % kSlots;

    auto msg = std::make_unique<std_msgs::msg::UInt64MultiArray>();
    msg->data = {offset, static_cast<uint64_t>(w_), static_cast<uint64_t>(h_)};
    vision_c310_image_offset_pub_->publish(std::move(msg));
  }

  cv::VideoCapture cap_;
  int w_{0}, h_{0};
  size_t slot_size_{0}, slot_{0};
  std::unique_ptr<uint8_t, Unmap> shm_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vision_c310_image_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr vision_c310_image_offset_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    rclcpp::spin(std::make_shared<C310Processor>());
  } catch (const std::exception & e) {  // camera not found / open / shm failure
    RCLCPP_FATAL(rclcpp::get_logger("c310_processor"), "%s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
