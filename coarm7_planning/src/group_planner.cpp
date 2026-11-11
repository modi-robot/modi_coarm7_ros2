#include "coarm7_planning/gripper_goal.hpp"
#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
namespace coarm7_planning {
class GroupPlanner final : public planning_interface::PlannerManager {
public:
  bool initialize(const moveit::core::RobotModelConstPtr &model,
                  const rclcpp::Node::SharedPtr &node, const std::string &ns) override {
    node_=node;model_=model;
    try {
      loader_=std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>(
          "moveit_core", "planning_interface::PlannerManager");
      ompl_=loader_->createSharedInstance("ompl_interface/OMPLPlanner");
      ptp_=loader_->createSharedInstance("pilz_industrial_motion_planner/CommandPlanner");
      if (!ompl_->initialize(model,node,ns) || !ptp_->initialize(model,node,ns)) return false;
      config_settings_=ompl_->getPlannerConfigurations();
      RCLCPP_INFO(node_->get_logger(),"CoArm7 routing ready: left/right/both grippers -> PTP; arms -> existing OMPL");
      return true;
    } catch (const std::exception &error) {
      RCLCPP_ERROR(node_->get_logger(),"CoArm7 planning backend initialization failed: %s",error.what());
      return false;
    }
  }
  std::string getDescription() const override { return "CoArm7 OMPL with gripper PTP"; }
  void getPlanningAlgorithms(std::vector<std::string> &algorithms) const override {
    // Keep the existing RViz algorithm list. Gripper requests are routed automatically.
    if (ompl_) ompl_->getPlanningAlgorithms(algorithms);
  }
  bool canServiceRequest(const planning_interface::MotionPlanRequest &request) const override {
    if (is_gripper(request.group_name)) {
      planning_interface::MotionPlanRequest normalized;std::string reason;
      return ptp_ && normalize_gripper_goal(*model_,request,normalized,reason) && ptp_->canServiceRequest(normalized);
    }
    return ompl_ && ompl_->canServiceRequest(request);
  }
  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr &scene,
      const planning_interface::MotionPlanRequest &request,
      moveit_msgs::msg::MoveItErrorCodes &error) const override {
    if (is_gripper(request.group_name)) {
      planning_interface::MotionPlanRequest normalized;std::string reason;
      if (!ptp_ || !normalize_gripper_goal(*model_,request,normalized,reason)) {
        error.val=moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        RCLCPP_ERROR(node_->get_logger(),"PTP gripper request rejected group=%s: %s (goals=%zu)",
                     request.group_name.c_str(),reason.c_str(),request.goal_constraints.size());
        return nullptr;
      }
      RCLCPP_INFO(node_->get_logger(),"CoArm7 planning route group=%s backend=PTP requested_planner=%s",
                  request.group_name.c_str(),request.planner_id.c_str());
      // A PTP failure is returned to MoveIt. Never fall back to a random gripper path.
      RCLCPP_INFO(node_->get_logger(),"PTP goal normalized joints=%zu -> %zu",
                  request.goal_constraints.front().joint_constraints.size(),
                  normalized.goal_constraints.front().joint_constraints.size());
      return ptp_->getPlanningContext(scene,normalized,error);
    }
    RCLCPP_INFO(node_->get_logger(),"CoArm7 planning route group=%s backend=OMPL planner=%s",
                request.group_name.c_str(),request.planner_id.c_str());
    return ompl_->getPlanningContext(scene,request,error);
  }
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap &settings) override {
    if (ompl_) {
      ompl_->setPlannerConfigurations(settings);
      config_settings_=ompl_->getPlannerConfigurations();
    } else config_settings_=settings;
  }
private:
  // Destroy backend objects before their class loader.
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> loader_;
  planning_interface::PlannerManagerPtr ompl_,ptp_;
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr model_;
};
}  // namespace coarm7_planning
PLUGINLIB_EXPORT_CLASS(coarm7_planning::GroupPlanner, planning_interface::PlannerManager)
