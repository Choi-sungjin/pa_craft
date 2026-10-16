// realsense_processor: D435 RGB/Depth (V4L2) -> image, depth scan merged into /lidar/merged
// pub: /vision/image, /lidar/merged
// pub: /vision/image_offset (std_msgs/UInt64MultiArray [offset, width, height])
//      BGR frame lives in shared memory /dev/shm/craft_vision at that offset (no image copy over DDS)

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/videoio.hpp>

#include "rclcpp/rclcpp.hpp"
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
  : Node("realsense_processor")
  {
    // udev symlink from 99-realsense-d435-streams.rules (videoN changes with plug order)
    const std::string camera = declare_parameter("camera", std::string("/dev/realsense_color"));
    if (!cap_.open(camera, cv::CAP_V4L2)) {
      throw std::runtime_error("camera open failed: " + camera);
    }
    w_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    h_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
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

    vision_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/vision/image", 10);
    vision_image_offset_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>("/vision/image_offset", 1);
    lidar_merged_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/merged", 10);
    timer_ = create_wall_timer(10ms, [this]() {on_timer();});
  }

  ~RealsenseProcessor() override {shm_unlink(kShmName);}

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
    vision_image_offset_pub_->publish(std::move(msg));
    // TODO: depth frame -> /lidar/merged
  }

  cv::VideoCapture cap_;
  int w_{0}, h_{0};
  size_t slot_size_{0}, slot_{0};
  std::unique_ptr<uint8_t, Unmap> shm_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr vision_image_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr vision_image_offset_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_merged_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealsenseProcessor>());
  rclcpp::shutdown();
  return 0;
}
