#pragma once
#include "craft_perception/perception_types.hpp"
#include <net.h>
#include <memory>
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
// Row-major [4+classes, anchors] float32: cx, cy, w, h rows, then one probability row per class.
std::vector<Detection> decode_yolo(const float *data, int rows, int anchors, const Letterbox &transform,
  int classes, float confidence, float iou, int max_detections);
std::vector<Detection> decode_yolo(const ncnn::Mat &output, const Letterbox &transform,
  int classes, float confidence, float iou, int max_detections);
struct DetectorConfig {
  std::string backend = "ncnn";  // ncnn | tensorrt
  std::string model_dir;         // ncnn: model.ncnn.param/bin
  std::string engine_path;       // tensorrt: engine serialized on the target device
  std::string input_blob = "in0", output_blob = "out0";
  int input_size = 640, classes = 5, threads = 1, max_detections = 100;
  float confidence = 0.5f, iou = 0.45f;
};
// One inference engine per detector. The returned output stays valid until the next infer().
class InferenceBackend {
public:
  virtual ~InferenceBackend() = default;
  // letterboxed_bgr: input_size x input_size BGR8. Returns [rows, anchors] row-major float32.
  virtual const float *infer(const cv::Mat &letterboxed_bgr, int &rows, int &anchors) = 0;
};
bool tensorrt_available();  // true when built with CRAFT_WITH_TENSORRT
class YoloDetector {
public:
  bool configure(const DetectorConfig &config);
  bool is_valid_input(const cv::Mat &bgr) const;
  std::vector<Detection> compute_result(const cv::Mat &bgr);
private:
  DetectorConfig config_;
  std::unique_ptr<InferenceBackend> backend_;
};
}
