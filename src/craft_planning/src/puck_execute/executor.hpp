#pragma once
#include "camera.hpp"
#include "runtime.hpp"
namespace puck::execute {
bool run_steps(Runner &node, CameraContext &camera, Steps &steps);
int execute_main(const Options &opts);
}
