#pragma once
#include "coarm7_planning/routing.hpp"
#include <moveit/robot_model/robot_model.h>
#include <cmath>
#include <map>
#include <set>
#include <string>
namespace coarm7_planning {
// RViz/MoveGroupInterface includes mimic variables in its joint goal. Validate
// those redundant constraints against the model before retaining active joints.
inline bool normalize_gripper_goal(const moveit::core::RobotModel &model,
    const moveit_msgs::msg::MotionPlanRequest &input,
    moveit_msgs::msg::MotionPlanRequest &output,std::string &reason) {
  auto fail=[&](const std::string &message) { reason=message;return false; };
  if (!is_gripper(input.group_name) || input.goal_constraints.size()!=1 ||
      !empty_constraints(input.path_constraints) || !input.trajectory_constraints.constraints.empty())
    return fail("requires one joint goal and no path/trajectory constraints");
  const auto *group=model.getJointModelGroup(input.group_name);
  if (!group) return fail("unknown gripper group");
  const auto &goal=input.goal_constraints.front();
  if (goal.joint_constraints.empty() || !goal.position_constraints.empty() ||
      !goal.orientation_constraints.empty() || !goal.visibility_constraints.empty())
    return fail("requires joint-only gripper goals");
  std::map<std::string,const moveit_msgs::msg::JointConstraint *> constraints;
  for (const auto &constraint:goal.joint_constraints) {
    const auto *joint=model.getJointModel(constraint.joint_name);
    if (!joint || joint->getVariableCount()!=1 || !group->hasJointModel(joint->getName()))
      return fail("joint outside selected group: "+constraint.joint_name);
    if (!constraints.emplace(constraint.joint_name,&constraint).second)
      return fail("duplicate joint: "+constraint.joint_name);
    if (!std::isfinite(constraint.position) || !std::isfinite(constraint.tolerance_above) ||
        !std::isfinite(constraint.tolerance_below) || !std::isfinite(constraint.weight) ||
        constraint.tolerance_above<0 || constraint.tolerance_below<0 || constraint.weight<=0)
      return fail("invalid joint constraint: "+constraint.joint_name);
  }
  for (const auto *joint:group->getActiveJointModels())
    if (joint->getVariableCount()!=1 || constraints.count(joint->getName())!=1)
      return fail("missing independent joint: "+joint->getName());
  for (const auto &constraint:goal.joint_constraints) {
    const auto *joint=model.getJointModel(constraint.joint_name);
    if (!joint->getMimic()) continue;
    double factor=1,offset=0;
    std::set<const moveit::core::JointModel *> visited;
    while (joint->getMimic()) {
      if (!visited.insert(joint).second) return fail("cyclic mimic chain");
      offset+=factor*joint->getMimicOffset();factor*=joint->getMimicFactor();joint=joint->getMimic();
    }
    auto master=constraints.find(joint->getName());
    if (master==constraints.end()) return fail("missing mimic master: "+joint->getName());
    const double expected=factor*master->second->position+offset;
    // PTP uses the exact master target. It must satisfy the original mimic
    // interval; contradictory commands are rejected, not silently stripped.
    if (!std::isfinite(expected) || expected<constraint.position-constraint.tolerance_below-1e-9 ||
        expected>constraint.position+constraint.tolerance_above+1e-9)
      return fail("inconsistent mimic target: "+constraint.joint_name);
  }
  output=routed_request(input);
  auto &joints=output.goal_constraints.front().joint_constraints;
  joints.clear();
  for (const auto *joint:group->getActiveJointModels()) joints.push_back(*constraints.at(joint->getName()));
  reason.clear();return true;
}
}  // namespace coarm7_planning
