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
// Row-major [4+classes(+mask_dim), anchors] float32: cx, cy, w, h rows, one probability row per
// class, then (seg models) mask_dim mask-coefficient rows.
std::vector<Detection> decode_yolo(const float *data, int rows, int anchors, const Letterbox &transform,
  int classes, float confidence, float iou, int max_detections);
std::vector<Detection> decode_yolo(const ncnn::Mat &output, const Letterbox &transform,
  int classes, float confidence, float iou, int max_detections);
// Seg: one detection's mask over roi (original image pixels). coefficients[k * stride] are its
// mask_dim coefficients; protos holds mask_dim planes of h x w, plane k at protos + k * plane_stride.
// logits = coefficients . protos, sampled back through the letterbox, kept where > 0 (sigmoid > 0.5).
cv::Mat decode_mask(const float *coefficients, size_t stride, const float *protos, size_t plane_stride,
  int mask_dim, int h, int w, const Letterbox &transform, int input_size, const cv::Rect &roi);
struct DetectorConfig {
  std::string backend = "ncnn";  // ncnn | tensorrt
  std::string model_dir;         // ncnn: model.ncnn.param/bin
  std::string engine_path;       // tensorrt: engine serialized on the target device
  std::string input_blob = "in0", output_blob = "out0";
  std::string proto_blob;        // seg models only: prototype masks; empty = detect model
  int input_size = 640, classes = 5, threads = 1, max_detections = 100;
  float confidence = 0.5f, iou = 0.45f;
};
// Raw network outputs; pointers stay valid until the next infer().
struct InferenceOutput {
  const float *boxes = nullptr;   // [rows, anchors] row-major
  int rows = 0, anchors = 0;
  const float *protos = nullptr;  // seg only: mask_dim planes of mask_h x mask_w
  size_t plane_stride = 0;        // floats between planes (ncnn pads channels)
  int mask_dim = 0, mask_h = 0, mask_w = 0;
};
// One inference engine per detector.
class InferenceBackend {
public:
  virtual ~InferenceBackend() = default;
  // letterboxed_bgr: input_size x input_size BGR8.
  virtual InferenceOutput infer(const cv::Mat &letterboxed_bgr) = 0;
};
bool tensorrt_available();  // true when built with CRAFT_WITH_TENSORRT
class YoloDetector {
public:
  bool configure(const DetectorConfig &config);
  bool is_valid_input(const cv::Mat &bgr) const;
  // Seg models also fill Detection::mask.
  std::vector<Detection> compute_result(const cv::Mat &bgr);
  bool is_segmentation() const { return !config_.proto_blob.empty(); }
private:
  DetectorConfig config_;
  std::unique_ptr<InferenceBackend> backend_;
};
}
