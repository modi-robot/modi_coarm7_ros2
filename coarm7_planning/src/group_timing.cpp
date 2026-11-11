#include "coarm7_planning/routing.hpp"
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace coarm7_planning {
class GroupTiming final : public planning_request_adapter::PlanningRequestAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr &node,const std::string &ns) override {
    loader_=std::make_unique<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>>(
        "moveit_core","planning_request_adapter::PlanningRequestAdapter");
    arm_timing_=loader_->createSharedInstance("default_planner_request_adapters/AddTimeOptimalParameterization");
    arm_timing_->initialize(node,ns);
  }
  std::string getDescription() const override { return "Preserve gripper PTP timing; original arm TOTG"; }
  bool adaptAndPlan(const PlannerFn &planner,const planning_scene::PlanningSceneConstPtr &scene,
                    const planning_interface::MotionPlanRequest &request,
                    planning_interface::MotionPlanResponse &response,
                    std::vector<std::size_t> &added) const override {
    if (is_gripper(request.group_name)) return planner(scene,request,response);
    return arm_timing_->adaptAndPlan(planner,scene,request,response,added);
  }
private:
  std::unique_ptr<pluginlib::ClassLoader<planning_request_adapter::PlanningRequestAdapter>> loader_;
  planning_request_adapter::PlanningRequestAdapterPtr arm_timing_;
};
}  // namespace coarm7_planning
PLUGINLIB_EXPORT_CLASS(coarm7_planning::GroupTiming,planning_request_adapter::PlanningRequestAdapter)
