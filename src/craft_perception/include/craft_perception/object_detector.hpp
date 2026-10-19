#pragma once
#include "craft_perception/perception_types.hpp"
#include <net.h>
#include <string>
#include <vector>

namespace craft_perception {
struct Letterbox {
  cv::Mat image;
  float scale;
  int left, top;
  cv::Size original;
};
Letterbox letterbox(const cv::Mat &bgr, int size);
std::vector<Detection> decode_yolo(const ncnn::Mat &output, const Letterbox &transform,
  int classes, float confidence, float iou, int max_detections);
struct DetectorConfig {
  std::string model_dir, input_blob = "in0", output_blob = "out0";
  int input_size = 640, classes = 5, threads = 1, max_detections = 100;
  float confidence = 0.5f, iou = 0.45f;
};
class ObjectDetector {
public:
  bool configure(const DetectorConfig &config);
  bool is_valid_input(const cv::Mat &bgr) const;
  std::vector<Detection> compute_result(const cv::Mat &bgr);
private:
  DetectorConfig config_;
  ncnn::Net net_;
};
}
