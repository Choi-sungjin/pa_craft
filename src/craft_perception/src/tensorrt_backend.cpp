// TensorRT YOLO backend (Jetson). Built only with CRAFT_WITH_TENSORRT; see CMakeLists.txt.
// Engine contract: input  <input_blob>  float32 [1,3,S,S] RGB in [0,1]
//                  output <output_blob> float32 [1,4+classes,anchors]  (ONNX export with nms=None)
// Uses the TensorRT >= 8.5 named-tensor API (JetPack 5.1+ / 6.x).
#include "craft_perception/yolo_detector.hpp"
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

namespace craft_perception {
namespace {

class Logger : public nvinfer1::ILogger {
public:
  void log(Severity severity, const char *message) noexcept override {
    if (severity <= Severity::kWARNING) std::cerr << "[TensorRT] " << message << '\n';
  }
};
Logger &logger() { static Logger instance; return instance; }

void check(cudaError_t status, const char *what) {
  if (status != cudaSuccess) {
    throw std::runtime_error(std::string("CUDA ") + what + ": " + cudaGetErrorString(status));
  }
}

std::string shape(const nvinfer1::Dims &d) {
  std::string s = "[";
  for (int i = 0; i < d.nbDims; ++i) s += (i ? "," : "") + std::to_string(d.d[i]);
  return s + "]";
}

class TensorRtBackend : public InferenceBackend {
public:
  ~TensorRtBackend() override {
    if (stream_) cudaStreamDestroy(stream_);
    cudaFree(device_input_);
    cudaFree(device_output_);
    cudaFreeHost(host_input_);
    cudaFreeHost(host_output_);
  }

  // Returns false with a message on stderr; never leaves a half-usable backend.
  bool configure(const DetectorConfig &config) {
    std::ifstream file(config.engine_path, std::ios::binary);
    if (!file) return fail("cannot open engine " + config.engine_path);
    const std::vector<char> blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    runtime_.reset(nvinfer1::createInferRuntime(logger()));
    if (!runtime_) return fail("createInferRuntime failed");
    engine_.reset(runtime_->deserializeCudaEngine(blob.data(), blob.size()));
    if (!engine_) return fail("cannot deserialize " + config.engine_path +
                              " (engines are tied to the device and TensorRT version that built them)");
    context_.reset(engine_->createExecutionContext());
    if (!context_) return fail("createExecutionContext failed");

    input_ = config.input_blob;
    output_ = config.output_blob;
    if (!is_float_tensor(input_, nvinfer1::TensorIOMode::kINPUT) ||
        !is_float_tensor(output_, nvinfer1::TensorIOMode::kOUTPUT)) {
      return fail("engine needs float32 input '" + input_ + "' and output '" + output_ +
                  "'; check input_blob/output_blob (Ultralytics ONNX: images/output0)");
    }
    const int s = config.input_size;
    const auto in = engine_->getTensorShape(input_.c_str());
    if (in.nbDims != 4 || (in.d[0] != 1 && in.d[0] != -1) || in.d[1] != 3 ||
        (in.d[2] != s && in.d[2] != -1) || (in.d[3] != s && in.d[3] != -1)) {
      return fail("input shape " + shape(in) + " does not match [1,3," + std::to_string(s) + "," +
                  std::to_string(s) + "]");
    }
    if (!context_->setInputShape(input_.c_str(), nvinfer1::Dims4{1, 3, s, s})) {
      return fail("setInputShape failed for " + shape(in));
    }
    const auto out = context_->getTensorShape(output_.c_str());
    if (out.nbDims != 3 || out.d[0] != 1 || out.d[1] != config.classes + 4 || out.d[2] <= 0) {
      return fail("output shape " + shape(out) + " is not [1," + std::to_string(config.classes + 4) +
                  ",anchors]; export ONNX with nms=None and match class_names");
    }
    size_ = s;
    rows_ = out.d[1];
    anchors_ = out.d[2];
    const size_t input_bytes = sizeof(float) * 3 * s * s;
    const size_t output_bytes = sizeof(float) * rows_ * anchors_;
    check(cudaStreamCreate(&stream_), "stream");
    check(cudaMalloc(&device_input_, input_bytes), "malloc input");
    check(cudaMalloc(&device_output_, output_bytes), "malloc output");
    check(cudaMallocHost(reinterpret_cast<void **>(&host_input_), input_bytes), "host input");
    check(cudaMallocHost(reinterpret_cast<void **>(&host_output_), output_bytes), "host output");
    if (!context_->setTensorAddress(input_.c_str(), device_input_) ||
        !context_->setTensorAddress(output_.c_str(), device_output_)) {
      return fail("setTensorAddress failed");
    }
    return true;
  }

  const float *infer(const cv::Mat &image, int &rows, int &anchors) override {
    if (image.cols != size_ || image.rows != size_ || image.type() != CV_8UC3) {
      throw std::invalid_argument("TensorRT input must be the letterboxed BGR8 square image");
    }
    // HWC BGR8 -> CHW RGB float [0,1], same preprocessing as the ncnn backend.
    const size_t plane = static_cast<size_t>(size_) * size_;
    float *r = host_input_, *g = host_input_ + plane, *b = host_input_ + 2 * plane;
    for (int y = 0; y < size_; ++y) {
      const uint8_t *p = image.ptr<uint8_t>(y);
      for (int x = 0; x < size_; ++x, p += 3) {
        const size_t i = static_cast<size_t>(y) * size_ + x;
        b[i] = p[0] / 255.0f; g[i] = p[1] / 255.0f; r[i] = p[2] / 255.0f;
      }
    }
    check(cudaMemcpyAsync(device_input_, host_input_, sizeof(float) * 3 * plane,
                          cudaMemcpyHostToDevice, stream_), "copy input");
    if (!context_->enqueueV3(stream_)) throw std::runtime_error("TensorRT enqueueV3 failed");
    check(cudaMemcpyAsync(host_output_, device_output_, sizeof(float) * rows_ * anchors_,
                          cudaMemcpyDeviceToHost, stream_), "copy output");
    check(cudaStreamSynchronize(stream_), "synchronize");
    rows = rows_;
    anchors = anchors_;
    return host_output_;
  }

private:
  bool is_float_tensor(const std::string &name, nvinfer1::TensorIOMode mode) const {
    for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
      if (name == engine_->getIOTensorName(i)) {
        return engine_->getTensorIOMode(name.c_str()) == mode &&
               engine_->getTensorDataType(name.c_str()) == nvinfer1::DataType::kFLOAT;
      }
    }
    return false;
  }
  static bool fail(const std::string &message) {
    std::cerr << "[craft_perception] TensorRT: " << message << '\n';
    return false;
  }

  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  std::string input_, output_;
  int size_ = 0, rows_ = 0, anchors_ = 0;
  cudaStream_t stream_ = nullptr;
  void *device_input_ = nullptr, *device_output_ = nullptr;
  float *host_input_ = nullptr, *host_output_ = nullptr;
};

}  // namespace

std::unique_ptr<InferenceBackend> make_tensorrt_backend(const DetectorConfig &config) {
  auto backend = std::make_unique<TensorRtBackend>();
  try {
    if (!backend->configure(config)) return nullptr;
  } catch (const std::exception &e) {
    std::cerr << "[craft_perception] TensorRT: " << e.what() << '\n';
    return nullptr;
  }
  return backend;
}

}  // namespace craft_perception
