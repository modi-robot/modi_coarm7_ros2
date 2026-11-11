#pragma once
#include <controller_interface/controller_interface.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <atomic>
#include <mutex>

namespace coarm7_hardware {
// EG2 is an endpoint actuator, not a cyclic position servo. This controller
// exposes the standard GripperCommand action and an explicit stop transaction.
class EndpointGripperController final : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &,const rclcpp::Duration &) override;
private:
  using Action=control_msgs::action::GripperCommand;
  using Goal=rclcpp_action::ServerGoalHandle<Action>;
  struct Command { double position=0,goal_id=0,stop_id=0; };
  void monitor();
  void request_stop(bool canceled,const char *reason,bool stalled=false);
  void finish(bool success,bool canceled,bool stalled);
  static double now();
  std::string joint_;
  double lower_=0,upper_=0.864,tolerance_=0.01,goal_timeout_=15,stall_timeout_=2;
  double feedback_timeout_=0.5,movement_threshold_=0.002,stop_timeout_=0.5;
  std::mutex mutex_; // Action/timer/lifecycle only; never acquired by update().
  std::shared_ptr<Goal> goal_;
  bool reserved_=false,stopping_=false,canceling_=false,stalled_=false;
  double started_=0,last_progress_=0,progress_position_=0,last_stamp_=0,stop_started_=0;
  int settled_=0;
  Command command_;
  realtime_tools::RealtimeBuffer<Command> commands_;
  std::atomic<bool> active_{false};
  std::atomic<double> position_{0},stamp_{0},ack_{0};
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_pub_;
};
}
