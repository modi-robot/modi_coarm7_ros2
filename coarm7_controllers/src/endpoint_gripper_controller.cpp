#include "coarm7_hardware/endpoint_gripper_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace coarm7_hardware {
using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
double EndpointGripperController::now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
CallbackReturn EndpointGripperController::on_init() {
  try {
    auto_declare<std::string>("joint","");
    for (auto p : {std::pair<const char *,double>{"lower",0}, {"upper",0.864},
         {"goal_tolerance",0.01},{"goal_timeout",15},{"stall_timeout",2},
         {"feedback_timeout",0.5},{"movement_threshold",0.002},{"stop_timeout",0.5}})
      auto_declare<double>(p.first,p.second);
  } catch (...) { return CallbackReturn::ERROR; }
  return CallbackReturn::SUCCESS;
}
InterfaceConfiguration EndpointGripperController::command_interface_configuration() const {
  return {interface_configuration_type::INDIVIDUAL,{joint_+"/position",joint_+"/goal_id",joint_+"/stop_id"}};
}
InterfaceConfiguration EndpointGripperController::state_interface_configuration() const {
  return {interface_configuration_type::INDIVIDUAL,{joint_+"/position",joint_+"/feedback_stamp",joint_+"/stop_ack"}};
}
CallbackReturn EndpointGripperController::on_configure(const rclcpp_lifecycle::State &) {
  const auto node=get_node();
  joint_=node->get_parameter("joint").as_string();
  lower_=node->get_parameter("lower").as_double(); upper_=node->get_parameter("upper").as_double();
  tolerance_=node->get_parameter("goal_tolerance").as_double();
  goal_timeout_=node->get_parameter("goal_timeout").as_double();
  stall_timeout_=node->get_parameter("stall_timeout").as_double();
  feedback_timeout_=node->get_parameter("feedback_timeout").as_double();
  movement_threshold_=node->get_parameter("movement_threshold").as_double();
  stop_timeout_=node->get_parameter("stop_timeout").as_double();
  for(double value : {lower_,upper_,tolerance_,goal_timeout_,stall_timeout_,feedback_timeout_,movement_threshold_,stop_timeout_})
    if(!std::isfinite(value)) return CallbackReturn::ERROR;
  if(joint_.empty() || lower_>=upper_ || tolerance_<=0 || movement_threshold_<=0 ||
     goal_timeout_<=0 || stall_timeout_<=0 || feedback_timeout_<=0 || stop_timeout_<=0) return CallbackReturn::ERROR;
  server_=rclcpp_action::create_server<Action>(node,"~/gripper_cmd",
    [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal> request) {
      std::lock_guard<std::mutex> lock(mutex_);
      const double q=request->command.position;
      // PTP endpoint arithmetic can produce -4e-17 for an exact zero goal.
      // Normalize only roundoff (far below feedback resolution), not real
      // out-of-range requests. The hardware limits themselves stay unchanged.
      constexpr double roundoff=1e-9;
      const double stamp=stamp_.load(),t=now();
      const char *reason=nullptr;
      if(!active_.load()) reason="controller inactive";
      else if(goal_ || reserved_) reason="another goal pending";
      else if(ack_.load()<command_.stop_id) reason="stop submission pending";
      else if(!std::isfinite(q) || q<lower_-roundoff || q>upper_+roundoff) reason="position outside limits";
      else if(!std::isfinite(request->command.max_effort) || request->command.max_effort!=0) reason="unsupported max_effort";
      else if(!std::isfinite(stamp) || stamp<=0 || stamp>t || t-stamp>feedback_timeout_) reason="feedback stale or invalid";
      if(reason) {
        RCLCPP_WARN(get_node()->get_logger(),"Gripper goal rejected reason=%s requested_rad=%.17g limits=[%.17g,%.17g] max_effort=%.17g feedback_age_s=%.6f stop_id=%.0f stop_submit_ack=%.0f",
          reason,q,lower_,upper_,request->command.max_effort,t-stamp,command_.stop_id,ack_.load());
        return rclcpp_action::GoalResponse::REJECT;
      }
      reserved_=true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [this](const std::shared_ptr<Goal> handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      if(handle!=goal_) return rclcpp_action::CancelResponse::REJECT;
      request_stop(true,"cancel");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<Goal> handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      reserved_=false; goal_=handle;
      if(!active_.load()) { finish(false,false,false); return; }
      stopping_=canceling_=stalled_=false; settled_=0;
      started_=last_progress_=now(); progress_position_=position_.load(); last_stamp_=stamp_.load();
      command_.position=std::clamp(handle->get_goal()->command.position,lower_,upper_); ++command_.goal_id;
      commands_.writeFromNonRT(command_);
      RCLCPP_INFO(get_node()->get_logger(),"Endpoint goal id=%.0f requested_rad=%.17g target_rad=%.17g feedback_rad=%.6f",command_.goal_id,handle->get_goal()->command.position,command_.position,position_.load());
    });
  state_pub_=node->create_publisher<control_msgs::msg::JointTrajectoryControllerState>("~/controller_state",rclcpp::SystemDefaultsQoS());
  timer_=node->create_wall_timer(std::chrono::milliseconds(50),[this] { monitor(); });
  return CallbackReturn::SUCCESS;
}
CallbackReturn EndpointGripperController::on_activate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(command_interfaces_.size()!=3 || state_interfaces_.size()!=3) return CallbackReturn::ERROR;
  command_={state_interfaces_[0].get_value(),command_interfaces_[1].get_value(),command_interfaces_[2].get_value()};
  if(!std::isfinite(command_.position) || !std::isfinite(command_.goal_id) || !std::isfinite(command_.stop_id)) return CallbackReturn::ERROR;
  position_.store(command_.position); stamp_.store(state_interfaces_[1].get_value()); ack_.store(state_interfaces_[2].get_value());
  commands_.writeFromNonRT(command_); reserved_=false; stopping_=false;
  state_pub_->on_activate(); active_.store(true);
  return CallbackReturn::SUCCESS;
}
CallbackReturn EndpointGripperController::on_deactivate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(mutex_);
  active_.store(false);
  // Unclaiming position also stops the SDK protocol. Keep the persistent stop
  // request in the command interface for the final hardware write of this cycle.
  ++command_.stop_id; commands_.writeFromNonRT(command_);
  if(command_interfaces_.size()==3) command_interfaces_[2].set_value(command_.stop_id);
  if(goal_) finish(false,false,false);
  reserved_=false; state_pub_->on_deactivate();
  return CallbackReturn::SUCCESS;
}
controller_interface::return_type EndpointGripperController::update(const rclcpp::Time &,const rclcpp::Duration &) {
  if(!active_.load()) return controller_interface::return_type::OK;
  const auto c=*commands_.readFromRT();
  command_interfaces_[0].set_value(c.position);
  command_interfaces_[1].set_value(c.goal_id);
  command_interfaces_[2].set_value(c.stop_id);
  position_.store(state_interfaces_[0].get_value());
  ack_.store(state_interfaces_[2].get_value());
  stamp_.store(state_interfaces_[1].get_value());
  return controller_interface::return_type::OK;
}
void EndpointGripperController::request_stop(bool canceled,const char *reason,bool stalled) {
  if(stopping_) return;
  stopping_=true; canceling_=canceled; stalled_=stalled; stop_started_=now();
  ++command_.stop_id; commands_.writeFromNonRT(command_);
  RCLCPP_WARN(get_node()->get_logger(),"Endpoint stop reason=%s goal_id=%.0f stop_id=%.0f target_rad=%.6f feedback_rad=%.6f (no measured-position retarget)",reason,command_.goal_id,command_.stop_id,command_.position,position_.load());
}
void EndpointGripperController::finish(bool success,bool canceled,bool stalled) {
  if(!goal_) return;
  auto result=std::make_shared<Action::Result>();
  result->position=position_.load(); result->effort=0; result->reached_goal=success; result->stalled=stalled;
  if(success) goal_->succeed(result);
  else if(canceled && goal_->is_canceling()) goal_->canceled(result);
  else goal_->abort(result);
  RCLCPP_INFO(get_node()->get_logger(),"Endpoint result goal_id=%.0f success=%s canceled=%s target_rad=%.6f feedback_rad=%.6f stop_submit_ack=%.0f",command_.goal_id,success?"true":"false",canceled?"true":"false",command_.position,result->position,ack_.load());
  goal_.reset(); stopping_=false;
}
void EndpointGripperController::monitor() {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!active_.load()) return;
  const double t=now(),q=position_.load(),stamp=stamp_.load();
  control_msgs::msg::JointTrajectoryControllerState state;
  state.header.stamp=get_node()->now(); state.joint_names={joint_};
  state.reference.positions={command_.position}; state.feedback.positions={q};
  state.error.positions={command_.position-q}; state.desired=state.reference; state.actual=state.feedback;
  state_pub_->publish(state);
  if(!goal_) return;
  auto feedback=std::make_shared<Action::Feedback>();
  feedback->position=q; feedback->effort=0; feedback->reached_goal=false; feedback->stalled=stalled_;
  goal_->publish_feedback(feedback);
  if(stopping_) {
    if(ack_.load()>=command_.stop_id) finish(false,canceling_,stalled_);
    else if(t-stop_started_>stop_timeout_) {
      RCLCPP_ERROR(get_node()->get_logger(),"Gripper stop SDK submission not confirmed within %.3f s; persistent stop request retained",stop_timeout_);
      finish(false,canceling_,stalled_);
    }
    return;
  }
  if(!std::isfinite(q) || stamp<=0 || stamp>t || t-stamp>feedback_timeout_) { request_stop(false,"feedback timeout"); return; }
  if(stamp!=last_stamp_) {
    last_stamp_=stamp;
    if(std::abs(q-command_.position)<=tolerance_) {
      if(++settled_>=2) { finish(true,false,false); return; }
    } else settled_=0;
    if(std::abs(q-progress_position_)>=movement_threshold_) { progress_position_=q; last_progress_=t; }
  }
  if(t-started_>goal_timeout_) request_stop(false,"goal timeout");
  else if(t-last_progress_>stall_timeout_ && std::abs(q-command_.position)>tolerance_) request_stop(false,"no feedback progress",true);
}
}
PLUGINLIB_EXPORT_CLASS(coarm7_hardware::EndpointGripperController,controller_interface::ControllerInterface)
