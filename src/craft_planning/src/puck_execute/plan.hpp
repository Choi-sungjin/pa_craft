// Port of demo/puck_pick/plan.py: the parts execute.py uses (URDF chain, FK, limits, ik_top_down, paths, ASSUME).
#pragma once

#include <Eigen/Dense>

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace puck {

namespace fs = std::filesystem;
using Pose = std::vector<double>;
using JointMap = std::map<std::string, double>;

// Installed package resources by default; explicit source overrides retain the demo workflow.
fs::path here();
fs::path ws();
fs::path urdf_path();
fs::path control_yaml();
fs::path home();
fs::path state_dir();
fs::path camera_data_dir();
fs::path camera_executable(const std::string &name);

double py_mod(double a, double b);  // Python a % b (floor modulo)

// plan.ASSUME entries execute.py reads (it never changes them).
constexpr double ASSUME_LIDAR_CENTER_X = 0.128;
constexpr double ASSUME_LIDAR_FRONT_Y = 0.034;
constexpr double ASSUME_TOOL_ROLL = 0.0;

class Chain {
public:
  static const std::vector<std::string> NAMES;  // slide, shoulder, elbow, wrist_pitch, tool_roll
  explicit Chain(const fs::path &urdf);
  std::pair<double, double> limits(const std::string &name) const;
  Eigen::Matrix4d fk(const JointMap &q) const;
  Eigen::Vector3d origin(const std::string &name) const;
  Eigen::Vector3d axis(const std::string &name) const;

private:
  struct Joint {
    std::string name, type;
    Eigen::Matrix4d origin = Eigen::Matrix4d::Identity();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero(), axis = Eigen::Vector3d::Zero();
    bool has_limit = false;
    double lower = 0, upper = 0;
  };
  std::map<std::string, Joint> j_;
  std::vector<Joint> chain_;
};

const Chain &chain();  // plan.Chain(plan.URDF), parsed once per process

std::vector<JointMap> ik_top_down(const Chain &ch, const Eigen::Vector3d &tcp, double tilt_deg = 0.0);

}  // namespace puck
