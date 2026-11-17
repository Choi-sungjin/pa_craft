// Port of demo/puck_pick/teach.py: only what execute.py calls (the serial teaching UI is not ported).
#pragma once

#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace puck::teach {

YAML::Node control_params();  // control.yaml control_master.ros__parameters, reloaded per call

// Raw motor degrees LM, shoulder, elbow, wrist_pitch, tool_roll, gripper -> MANIPULATOR target: URDF slide [m],
// shoulder, elbow, wrist_pitch, tool_roll [rad], gripper motor angle [rad].
std::vector<double> raw_to_target(const std::vector<double> &raw_deg);

// Inverse conversion for inspecting targets in motor degrees; no hardware access.
std::vector<double> target_to_raw(const std::vector<double> &target);

// [(min, max)] motor rad per axis LM..gripper.
std::vector<std::pair<double, double>> motor_limits();

}  // namespace puck::teach
