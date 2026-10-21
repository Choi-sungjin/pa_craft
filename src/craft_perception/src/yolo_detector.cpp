#include "craft_perception/yolo_detector.hpp"
#include "geometry_utils.hpp"
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

namespace {
// Shared by decode_yolo and seg: also returns each kept detection's anchor index.
std::vector<std::pair<Detection, int>> decode(const float *data, int rows, int anchors,
  const Letterbox &t, int classes, int mask_dim, float confidence, float iou, int max_detections) {
  // Ultralytics one-to-many export (ncnn, or ONNX with nms=None): [4+classes(+mask_dim), anchors],
  // cxcywh in input pixels. Refuse incompatible exports instead of guessing end-to-end/DFL layouts.
  if (data == nullptr || rows != classes + 4 + mask_dim || anchors <= 0) {
    throw std::runtime_error("YOLO output must be float32 [4+class_count" +
      std::string(mask_dim ? "+mask_dim" : "") + ",anchors]; got rows=" + std::to_string(rows) +
      " anchors=" + std::to_string(anchors));
  }
  const auto row = [data, anchors](int r) { return data + static_cast<size_t>(r) * anchors; };
  std::vector<std::pair<Detection, int>> candidates;
  for (int i = 0; i < anchors; ++i) {
    int id = -1;
    float score = confidence;
    for (int c = 0; c < classes; ++c) {
      const float value = row(c + 4)[i];
      if (std::isfinite(value) && value >= score && value <= 1.0f) { id = c; score = value; }
    }
    if (id < 0) continue;
    const float cx = row(0)[i], cy = row(1)[i];
    const float width = row(2)[i], height = row(3)[i];
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(width) ||
        !std::isfinite(height) || width <= 0 || height <= 0) continue;
    const float x1 = std::clamp((cx - width / 2 - t.left) / t.scale, 0.0f, float(t.original.width));
    const float y1 = std::clamp((cy - height / 2 - t.top) / t.scale, 0.0f, float(t.original.height));
    const float x2 = std::clamp((cx + width / 2 - t.left) / t.scale, 0.0f, float(t.original.width));
    const float y2 = std::clamp((cy + height / 2 - t.top) / t.scale, 0.0f, float(t.original.height));
    if (x2 > x1 && y2 > y1) candidates.push_back({{{x1, y1, x2 - x1, y2 - y1}, id, score, {}}, i});
  }
  std::stable_sort(candidates.begin(), candidates.end(),
    [](const auto &a, const auto &b) { return a.first.score > b.first.score; });
  std::vector<std::pair<Detection, int>> kept;
  for (const auto &candidate : candidates) {
    bool suppressed = false;
    for (const auto &previous : kept) {
      if (candidate.first.class_id != previous.first.class_id) continue;
      const float intersection = (candidate.first.box & previous.first.box).area();
      const float union_area = candidate.first.box.area() + previous.first.box.area() - intersection;
      if (union_area > 0 && intersection / union_area > iou) { suppressed = true; break; }
    }
    if (!suppressed) kept.push_back(candidate);
    if (static_cast<int>(kept.size()) >= max_detections) break;
  }
  return kept;
}
}  // namespace

std::vector<Detection> decode_yolo(
  const float *data, int rows, int anchors, const Letterbox &t, int classes,
  float confidence, float iou, int max_detections) {
  std::vector<Detection> output;
  for (auto &kept : decode(data, rows, anchors, t, classes, 0, confidence, iou, max_detections)) {
    output.push_back(std::move(kept.first));
  }
  return output;
}

cv::Mat decode_mask(const float *coefficients, size_t stride, const float *protos, size_t plane_stride,
  int mask_dim, int h, int w, const Letterbox &t, int input_size, const cv::Rect &roi) {
  cv::Mat logits(h, w, CV_32F, cv::Scalar(0));
  for (int k = 0; k < mask_dim; ++k) {
    const cv::Mat plane(h, w, CV_32F, const_cast<float *>(protos + k * plane_stride));
    cv::scaleAdd(plane, coefficients[k * stride], logits, logits);
  }
  // Original pixel center -> letterboxed input -> proto grid (pixel centers at +0.5).
  cv::Mat map_x(roi.height, roi.width, CV_32F), map_y(roi.height, roi.width, CV_32F);
  const float to_proto_x = float(w) / input_size, to_proto_y = float(h) / input_size;
  for (int y = 0; y < roi.height; ++y) {
    const float py = ((roi.y + y + 0.5f) * t.scale + t.top) * to_proto_y - 0.5f;
    for (int x = 0; x < roi.width; ++x) {
      map_x.at<float>(y, x) = ((roi.x + x + 0.5f) * t.scale + t.left) * to_proto_x - 0.5f;
      map_y.at<float>(y, x) = py;
    }
  }
  cv::Mat sampled, mask;
  cv::remap(logits, sampled, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  cv::compare(sampled, 0, mask, cv::CMP_GT);
  return mask;
}

std::vector<Detection> decode_yolo(
  const ncnn::Mat &out, const Letterbox &t, int classes,
  float confidence, float iou, int max_detections) {
  if (out.dims != 2 || out.elempack != 1 || out.elemsize != sizeof(float)) {
    throw std::runtime_error("NCNN output must be float32 [4+class_count,anchors]; got dims=" +
      std::to_string(out.dims) + " w=" + std::to_string(out.w) + " h=" + std::to_string(out.h) +
      " c=" + std::to_string(out.c));
  }
  return decode_yolo(static_cast<const float *>(out.data), out.h, out.w, t, classes,
                     confidence, iou, max_detections);
}

namespace {
class NcnnBackend : public InferenceBackend {
public:
  bool configure(const DetectorConfig &config) {
    net_.opt.num_threads = config.threads;
    net_.opt.use_vulkan_compute = false;
    net_.opt.use_packing_layout = false;
    input_blob_ = config.input_blob;
    output_blob_ = config.output_blob;
    proto_blob_ = config.proto_blob;
    return net_.load_param((config.model_dir + "/model.ncnn.param").c_str()) == 0 &&
           net_.load_model((config.model_dir + "/model.ncnn.bin").c_str()) == 0;
  }
  InferenceOutput infer(const cv::Mat &image) override {
    ncnn::Mat input = ncnn::Mat::from_pixels(image.data, ncnn::Mat::PIXEL_BGR2RGB, image.cols, image.rows);
    const float scale[] = {1.0f / 255, 1.0f / 255, 1.0f / 255};
    input.substract_mean_normalize(nullptr, scale);
    auto extractor = net_.create_extractor();
    if (extractor.input(input_blob_.c_str(), input) != 0 ||
        extractor.extract(output_blob_.c_str(), output_) != 0) {
      throw std::runtime_error("NCNN input/extract failed; check blob names");
    }
    if (output_.dims != 2 || output_.elempack != 1 || output_.elemsize != sizeof(float)) {
      throw std::runtime_error("NCNN output must be float32 [4+class_count,anchors]; got dims=" +
        std::to_string(output_.dims) + " w=" + std::to_string(output_.w) + " h=" +
        std::to_string(output_.h) + " c=" + std::to_string(output_.c));
    }
    InferenceOutput result;
    result.boxes = static_cast<const float *>(output_.data);
    result.rows = output_.h;
    result.anchors = output_.w;
    if (!proto_blob_.empty()) {
      if (extractor.extract(proto_blob_.c_str(), protos_) != 0 || protos_.dims != 3 ||
          protos_.elempack != 1 || protos_.elemsize != sizeof(float)) {
        throw std::runtime_error("NCNN seg proto '" + proto_blob_ + "' must be float32 [mask_dim,h,w]");
      }
      result.protos = static_cast<const float *>(protos_.data);
      result.plane_stride = protos_.cstep;
      result.mask_dim = protos_.c;
      result.mask_h = protos_.h;
      result.mask_w = protos_.w;
    }
    return result;
  }
private:
  ncnn::Net net_;
  ncnn::Mat output_, protos_;
  std::string input_blob_, output_blob_, proto_blob_;
};
}  // namespace

#ifdef CRAFT_WITH_TENSORRT
std::unique_ptr<InferenceBackend> make_tensorrt_backend(const DetectorConfig &config);  // tensorrt_backend.cpp
bool tensorrt_available() { return true; }
#else
bool tensorrt_available() { return false; }
#endif

bool YoloDetector::configure(const DetectorConfig &config) {
  if (config.classes <= 0 || config.threads < 1 ||
      config.input_size <= 0 || config.input_size % 32 || config.max_detections < 1 ||
      !std::isfinite(config.confidence) || config.confidence <= 0 || config.confidence > 1 ||
      !std::isfinite(config.iou) || config.iou <= 0 || config.iou > 1) {
    return false;
  }
  backend_.reset();
  if (config.backend == "ncnn") {
    if (config.model_dir.empty()) return false;
    auto ncnn = std::make_unique<NcnnBackend>();
    if (!ncnn->configure(config)) return false;
    backend_ = std::move(ncnn);
  } else if (config.backend == "tensorrt") {
    if (config.engine_path.empty()) return false;
#ifdef CRAFT_WITH_TENSORRT
    backend_ = make_tensorrt_backend(config);  // nullptr on load/shape failure (logged)
    if (!backend_) return false;
#else
    throw std::invalid_argument("backend tensorrt requested but craft_perception was built without "
      "TensorRT (rebuild on the Jetson with -DCRAFT_TENSORRT=on)");
#endif
  } else {
    return false;
  }
  config_ = config;
  return true;
}

bool YoloDetector::is_valid_input(const cv::Mat &bgr) const {
  return !bgr.empty() && bgr.dims == 2 && bgr.type() == CV_8UC3;
}

std::vector<Detection> YoloDetector::compute_result(const cv::Mat &bgr) {
  if (!backend_) throw std::logic_error("YoloDetector used before a successful configure()");
  const auto transform = letterbox(bgr, config_.input_size);
  const auto out = backend_->infer(transform.image);
  if (is_segmentation() && (!out.protos || out.mask_dim <= 0)) {
    throw std::runtime_error("seg model configured but the backend returned no prototype masks");
  }
  auto kept = decode(out.boxes, out.rows, out.anchors, transform, config_.classes,
                     is_segmentation() ? out.mask_dim : 0, config_.confidence, config_.iou,
                     config_.max_detections);
  std::vector<Detection> output;
  for (auto &[detection, anchor] : kept) {
    if (is_segmentation()) {
      const auto roi = detail::clipped(detection.box, bgr.size());
      if (!roi.empty()) {
        const float *coefficients = out.boxes + static_cast<size_t>(4 + config_.classes) * out.anchors + anchor;
        detection.mask = decode_mask(coefficients, out.anchors, out.protos, out.plane_stride,
          out.mask_dim, out.mask_h, out.mask_w, transform, config_.input_size, roi);
      }
    }
    output.push_back(std::move(detection));
  }
  return output;
}

}  // namespace craft_perception
