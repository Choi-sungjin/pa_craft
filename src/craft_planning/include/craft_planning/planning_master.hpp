#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace craft_planning
{
// Owns configuration, ROS I/O and scheduling; algorithm modules own no ROS nodes.
class PlanningMaster : public rclcpp::Node
{
public:
  explicit PlanningMaster(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanningMaster() override;

private:
  void configure();
  void connect();
  void planning_loop();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace craft_planning
