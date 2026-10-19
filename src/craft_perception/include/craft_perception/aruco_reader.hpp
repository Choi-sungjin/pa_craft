#pragma once
#include <opencv2/aruco.hpp>
#include <vector>

namespace craft_perception {
class ArucoReader {
public:
  bool configure(int dictionary);
  bool is_valid_input(const cv::Mat &bgr) const;
  std::vector<int> compute_result(const cv::Mat &bgr) const;
private:
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
};
}
