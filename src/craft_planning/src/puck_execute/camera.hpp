#pragma once

#include <string>
#include <vector>

namespace puck::execute {
class Runner;
struct CameraResult {
  int returncode;
  std::string out;
  std::string err;
};
// capture_output=True, text=True, timeout semantics; throws on timeout/exec error.
CameraResult camera_run(Runner *node, const std::vector<std::string> &command, double timeout_s);
}

#include "geometry.hpp"
#include "pick_policy.hpp"
#include <filesystem>
#include <optional>
#include <utility>
namespace puck::execute {
struct CameraContext {
  Runner *node = nullptr;
  // Optional offline injection for deterministic handler tests; production leaves this empty.
  std::function<CameraResult(const std::vector<std::string>&, double)> capture;
  std::optional<policy::Candidate> target;
  std::vector<policy::Candidate> candidates;
  Json find = Json::object();
  Json cal_shots = Json::object();
  Json gripcal_shots = Json::object();
  std::optional<std::string> learn_ref;
  Eigen::Vector2d fixed = Eigen::Vector2d::Zero();
  int tries = 0;
  double shelf_fix = 0;
  std::optional<double> roll_fix;
  std::optional<Eigen::Vector3d> previous_cam, previous_tag;
  std::string previous_plan;
  std::filesystem::path run_dir;
};
using CameraStepResult = std::pair<bool, std::string>;
policy::Candidate survey_pick(CameraContext &ctx);
policy::Candidate redock_for_reach(CameraContext &ctx, const policy::Candidate &chosen);
CameraStepResult find_step(CameraContext &ctx, Steps &steps, std::size_t i, const Json &arg);
CameraStepResult stand_yaw_step(CameraContext &ctx, Steps &steps, std::size_t i);
CameraStepResult lie_yaw_step(CameraContext &ctx, Steps &steps, std::size_t i, const Json &arg);
CameraStepResult standcal_step(CameraContext &ctx, const Json &arg);
CameraStepResult gripzero_step(CameraContext &ctx, const std::string &what);
CameraStepResult gripcal_step(CameraContext &ctx, const std::string &what);
CameraStepResult gripcheck_step(CameraContext &ctx, Steps &steps, std::size_t i);
std::string held_bad(CameraContext &ctx, const std::string &name, const std::string &shot, const Steps &steps, std::size_t i);
void record_loaded(const std::string &name, bool grasped);
void learn_reference(CameraContext &ctx);
}
