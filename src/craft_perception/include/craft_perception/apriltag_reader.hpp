#pragma once
#include <opencv2/aruco.hpp>
#include <string>
#include <vector>

namespace craft_perception {
// AprilTag ids sorted left -> right, using OpenCV's built-in AprilTag dictionaries.
class AprilTagReader {
public:
  bool configure(const std::string &family);  // tag16h5 | tag25h9 | tag36h10 | tag36h11
  bool is_valid_input(const cv::Mat &bgr) const;
  std::vector<int> compute_result(const cv::Mat &bgr) const;
private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};
}
