#include "craft_perception/object_detector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace craft_perception {

Letterbox letterbox(const cv::Mat &bgr, int size) {
  if (bgr.empty() || bgr.type() != CV_8UC3 || size <= 0) {
    throw std::invalid_argument("letterbox needs BGR8 and a positive input size");
  }
  const float scale = std::min(static_cast<float>(size) / bgr.cols,
                              static_cast<float>(size) / bgr.rows);
  const int width = std::max(1, static_cast<int>(std::round(bgr.cols * scale)));
  const int height = std::max(1, static_cast<int>(std::round(bgr.rows * scale)));
  const int left = (size - width) / 2, top = (size - height) / 2;
  cv::Mat resized, padded;
  cv::resize(bgr, resized, cv::Size(width, height));
  cv::copyMakeBorder(resized, padded, top, size - height - top,
                    left, size - width - left, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  return {padded, scale, left, top, bgr.size()};
}

std::vector<Detection> decode_yolo(
  const ncnn::Mat &out, const Letterbox &t, int classes,
  float confidence, float iou, int max_detections) {
  // Ultralytics NCNN one-to-many export: [4+classes, anchors], cxcywh in input pixels.
  // Refuse incompatible exports instead of guessing end-to-end/DFL tensor layouts.
  if (out.dims != 2 || out.h != classes + 4 || out.w <= 0 || out.elempack != 1 ||
      out.elemsize != sizeof(float)) {
    throw std::runtime_error("NCNN output must be float32 [4+class_count,anchors]; got dims=" +
      std::to_string(out.dims) + " w=" + std::to_string(out.w) + " h=" + std::to_string(out.h) +
      " c=" + std::to_string(out.c));
  }
  std::vector<Detection> candidates;
  for (int i = 0; i < out.w; ++i) {
    int id = -1;
    float score = confidence;
    for (int c = 0; c < classes; ++c) {
      const float value = out.row(c + 4)[i];
      if (std::isfinite(value) && value >= score && value <= 1.0f) { id = c; score = value; }
    }
    if (id < 0) continue;
    const float cx = out.row(0)[i], cy = out.row(1)[i];
    const float width = out.row(2)[i], height = out.row(3)[i];
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) ||
        !std::isfinite(height) || width <= 0 || height <= 0) continue;
    const float x1 = std::clamp((cx - width / 2 - t.left) / t.scale, 0.0f, float(t.original.width));
    const float y1 = std::clamp((cy - height / 2 - t.top) / t.scale, 0.0f, float(t.original.height));
    const float x2 = std::clamp((cx + width / 2 - t.left) / t.scale, 0.0f, float(t.original.width));
    const float y2 = std::clamp((cy + height / 2 - t.top) / t.scale, 0.0f, float(t.original.height));
    if (x2 > x1 && y2 > y1) candidates.push_back({{x1, y1, x2 - x1, y2 - y1}, id, score});
  }
  std::stable_sort(candidates.begin(), candidates.end(),
    [](const Detection &a, const Detection &b) { return a.score > b.score; });
  std::vector<Detection> kept;
  for (const auto &candidate : candidates) {
    bool suppressed = false;
    for (const auto &previous : kept) {
      if (candidate.class_id != previous.class_id) continue;
      const float intersection = (candidate.box & previous.box).area();
      const float union_area = candidate.box.area() + previous.box.area() - intersection;
      if (union_area > 0 && intersection / union_area > iou) { suppressed = true; break; }
    }
    if (!suppressed) kept.push_back(candidate);
    if (static_cast<int>(kept.size()) >= max_detections) break;
  }
  return kept;
}

bool ObjectDetector::configure(const DetectorConfig &config) {
  if (config.model_dir.empty() || config.classes <= 0 || config.threads < 1 ||
      config.input_size <= 0 || config.input_size % 32 || config.max_detections < 1 ||
      !std::isfinite(config.confidence) || config.confidence <= 0 || config.confidence > 1 ||
      !std::isfinite(config.iou) || config.iou <= 0 || config.iou > 1) {
    return false;
  }
  net_.clear();
  net_.opt.num_threads = config.threads;
  net_.opt.use_vulkan_compute = false;
  net_.opt.use_packing_layout = false;
  if (net_.load_param((config.model_dir + "/model.ncnn.param").c_str()) != 0 ||
      net_.load_model((config.model_dir + "/model.ncnn.bin").c_str()) != 0) {
    return false;
  }
  config_ = config;
  return true;
}

bool ObjectDetector::is_valid_input(const cv::Mat &bgr) const {
  return !bgr.empty() && bgr.dims == 2 && bgr.type() == CV_8UC3;
}

std::vector<Detection> ObjectDetector::compute_result(const cv::Mat &bgr) {
  const auto transform = letterbox(bgr, config_.input_size);
  ncnn::Mat input = ncnn::Mat::from_pixels(transform.image.data,
    ncnn::Mat::PIXEL_BGR2RGB, transform.image.cols, transform.image.rows);
  const float scale[] = {1.0f / 255, 1.0f / 255, 1.0f / 255};
  input.substract_mean_normalize(nullptr, scale);
  auto extractor = net_.create_extractor();
  ncnn::Mat output;
  if (extractor.input(config_.input_blob.c_str(), input) != 0 ||
      extractor.extract(config_.output_blob.c_str(), output) != 0) {
    throw std::runtime_error("NCNN input/extract failed; check blob names");
  }
  return decode_yolo(output, transform, config_.classes, config_.confidence,
                     config_.iou, config_.max_detections);
}

}  // namespace craft_perception
