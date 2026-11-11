#pragma once
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <string>
namespace coarm7_planning {
inline bool is_gripper(const std::string &group) {
  return group == "left_gripper" || group == "right_gripper" || group == "both_grippers";
}
inline bool empty_constraints(const moveit_msgs::msg::Constraints &c) {
  return c.joint_constraints.empty() && c.position_constraints.empty() &&
         c.orientation_constraints.empty() && c.visibility_constraints.empty();
}
inline moveit_msgs::msg::MotionPlanRequest routed_request(const moveit_msgs::msg::MotionPlanRequest &request) {
  auto result=request;
  if (is_gripper(result.group_name)) {
    result.planner_id="PTP";
    result.num_planning_attempts=1;
  }
  return result;
}
}  // namespace coarm7_planning
