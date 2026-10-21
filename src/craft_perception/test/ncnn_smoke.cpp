#include "craft_perception/yolo_detector.hpp"
#include <iostream>
#include <string>
#include <stdexcept>

int main(int argc, char **argv) {
  if (argc != 2 && argc != 3) return 2;  // model_dir [class_count, default 5]
  craft_perception::DetectorConfig config;
  config.model_dir = argv[1];
  if (argc == 3) config.classes = std::stoi(argv[2]);
  craft_perception::YoloDetector detector;
  if (!detector.configure(config)) throw std::runtime_error("NCNN configuration/model load failed");
  cv::Mat image(360, 640, CV_8UC3, cv::Scalar(114, 114, 114));
  if (!detector.is_valid_input(image)) return 1;
  const auto output = detector.compute_result(image);
  std::cout << "NCNN model inference and output decoding passed; detections=" << output.size() << '\n';
}
