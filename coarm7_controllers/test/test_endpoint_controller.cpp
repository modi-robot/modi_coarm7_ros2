#include "coarm7_hardware/endpoint_gripper_controller.hpp"
#include <gtest/gtest.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <thread>
#include <cstdlib>
using namespace coarm7_hardware;
using Action=control_msgs::action::GripperCommand;
using ClientGoal=rclcpp_action::ClientGoalHandle<Action>;
struct EndpointAction : testing::Test {
  std::shared_ptr<EndpointGripperController> controller;
  rclcpp::Node::SharedPtr node;
  rclcpp_action::Client<Action>::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<hardware_interface::CommandInterface> commands;
  std::vector<hardware_interface::StateInterface> states;
  double target=0,goal_id=0,stop_id=0,position=0,stamp=0,ack=0;
  bool fresh=true,ack_stops=true,activated=false;
  static double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
  void SetUp() override {
    controller=std::make_shared<EndpointGripperController>();
    ASSERT_EQ(controller->init("endpoint_action_test"),controller_interface::return_type::OK);
    controller->get_node()->set_parameters({rclcpp::Parameter("joint","gripper"),
      rclcpp::Parameter("feedback_timeout",0.3),rclcpp::Parameter("stall_timeout",0.5),
      rclcpp::Parameter("goal_timeout",1.0),rclcpp::Parameter("stop_timeout",0.2)});
    ASSERT_EQ(controller->on_configure(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);
    commands.emplace_back("gripper","position",&target);commands.emplace_back("gripper","goal_id",&goal_id);commands.emplace_back("gripper","stop_id",&stop_id);
    states.emplace_back("gripper","position",&position);states.emplace_back("gripper","feedback_stamp",&stamp);states.emplace_back("gripper","stop_ack",&ack);
    std::vector<hardware_interface::LoanedCommandInterface> lc;
    std::vector<hardware_interface::LoanedStateInterface> ls;
    for(auto &c:commands) lc.emplace_back(c);
    for(auto &s:states) ls.emplace_back(s);
    controller->assign_interfaces(std::move(lc),std::move(ls)); stamp=now();
    ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()),controller_interface::CallbackReturn::SUCCESS);activated=true;
    node=std::make_shared<rclcpp::Node>("endpoint_action_client");
    executor.add_node(controller->get_node()->get_node_base_interface());executor.add_node(node);
    client=rclcpp_action::create_client<Action>(node,"/endpoint_action_test/gripper_cmd");
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  }
  void pump(double seconds) {
    const auto end=now()+seconds;
    do {
      if(fresh) stamp=now(); if(ack_stops) ack=stop_id;
      controller->update(rclcpp::Time(0),rclcpp::Duration::from_seconds(0.005));
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while(now()<end);
  }
  template<class Future> auto wait(Future f) {
    const auto end=now()+2;
    while(f.wait_for(std::chrono::seconds(0))!=std::future_status::ready && now()<end) pump(0.005);
    if(f.wait_for(std::chrono::seconds(0))!=std::future_status::ready) throw std::runtime_error("action timeout");
    return f.get();
  }
  std::shared_ptr<ClientGoal> send(double q,double effort=0) {
    Action::Goal goal;goal.command.position=q;goal.command.max_effort=effort;
    return wait(client->async_send_goal(goal));
  }
  void TearDown() override {
    if(activated) controller->on_deactivate(rclcpp_lifecycle::State());
    executor.spin_some();
    executor.remove_node(controller->get_node()->get_node_base_interface());executor.remove_node(node);
    controller->release_interfaces();
  }
};
TEST_F(EndpointAction, CancelUsesExplicitStopAndWaitsForSubmitAckWithoutMeasuredRetarget) {
  position=0.3;pump(0.01);EXPECT_EQ(goal_id,0);EXPECT_EQ(stop_id,0);
  auto handle=send(0.8);ASSERT_TRUE(handle);pump(0.02);
  EXPECT_EQ(target,0.8);EXPECT_EQ(goal_id,1);
  ack_stops=false;
  auto response=wait(client->async_cancel_goal(handle));ASSERT_FALSE(response->goals_canceling.empty());
  auto result=client->async_get_result(handle);pump(0.06);
  EXPECT_EQ(stop_id,1);EXPECT_EQ(target,0.8);
  EXPECT_NE(result.wait_for(std::chrono::seconds(0)),std::future_status::ready);
  ack_stops=true;
  auto ended=wait(result);EXPECT_EQ(ended.code,rclcpp_action::ResultCode::CANCELED);EXPECT_FALSE(ended.result->reached_goal);
  auto again=send(0.8);ASSERT_TRUE(again);pump(0.01);EXPECT_EQ(goal_id,2);
}
TEST_F(EndpointAction, OnlyFreshFeedbackCanCompleteAndUnsupportedGoalsAreRejected) {
  EXPECT_FALSE(send(1.0)); EXPECT_FALSE(send(0.4,1.0));
  auto handle=send(0.8);ASSERT_TRUE(handle);EXPECT_FALSE(send(0.5));
  position=0.8;fresh=false;
  auto result=client->async_get_result(handle);pump(0.13);
  EXPECT_NE(result.wait_for(std::chrono::seconds(0)),std::future_status::ready);
  fresh=true;
  const auto ended=wait(result);EXPECT_EQ(ended.code,rclcpp_action::ResultCode::SUCCEEDED);EXPECT_TRUE(ended.result->reached_goal);
  EXPECT_EQ(stop_id,0);
}
TEST_F(EndpointAction, StalledEndpointAbortsWithStopRatherThanReportingSuccess) {
  auto handle=send(0.8);ASSERT_TRUE(handle);
  const auto ended=wait(client->async_get_result(handle));
  EXPECT_EQ(ended.code,rclcpp_action::ResultCode::ABORTED);EXPECT_TRUE(ended.result->stalled);
  EXPECT_FALSE(ended.result->reached_goal);EXPECT_EQ(stop_id,1);EXPECT_EQ(target,0.8);
}
TEST_F(EndpointAction, MissingStopAckBlocksNewGoalsAndDeactivationRetainsStop) {
  auto handle=send(0.8);ASSERT_TRUE(handle);ack_stops=false;
  wait(client->async_cancel_goal(handle));
  const auto ended=wait(client->async_get_result(handle));
  EXPECT_EQ(ended.code,rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(send(0.4));EXPECT_EQ(stop_id,1);
  controller->on_deactivate(rclcpp_lifecycle::State());activated=false;
  EXPECT_EQ(stop_id,2);EXPECT_EQ(target,0.8);
}
TEST_F(EndpointAction, ExpiredFeedbackRequestsStopAndCannotReportArrival) {
  auto handle=send(0.8);ASSERT_TRUE(handle);fresh=false;
  const auto ended=wait(client->async_get_result(handle));
  EXPECT_EQ(ended.code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(ended.result->reached_goal);EXPECT_FALSE(ended.result->stalled);
  EXPECT_EQ(stop_id,1);EXPECT_EQ(target,0.8);
}
TEST_F(EndpointAction, ProgressCannotBypassTheTotalGoalTimeout) {
  auto handle=send(0.8);ASSERT_TRUE(handle);
  auto result=client->async_get_result(handle);
  for(int i=0;i<25 && result.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i) {
    position+=0.01;pump(0.05);
  }
  const auto ended=wait(result);
  EXPECT_EQ(ended.code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(ended.result->reached_goal);EXPECT_FALSE(ended.result->stalled);
  EXPECT_EQ(stop_id,1);EXPECT_EQ(target,0.8);
}
TEST_F(EndpointAction, PtpEndpointRoundoffIsNormalizedWithoutRelaxingRealBounds) {
  EXPECT_FALSE(send(-2e-9));EXPECT_FALSE(send(0.864+2e-9));
  auto handle=send(-4.163336342344337e-17);ASSERT_TRUE(handle);
  auto ended=wait(client->async_get_result(handle));
  EXPECT_EQ(ended.code,rclcpp_action::ResultCode::SUCCEEDED);EXPECT_DOUBLE_EQ(target,0.0);
  handle=send(0.864+1e-16);ASSERT_TRUE(handle);pump(0.02);EXPECT_DOUBLE_EQ(target,0.864);
  wait(client->async_cancel_goal(handle));ended=wait(client->async_get_result(handle));
  EXPECT_EQ(ended.code,rclcpp_action::ResultCode::CANCELED);
}
int main(int argc,char **argv) {
  setenv("ROS_DOMAIN_ID","165",1);setenv("ROS_LOCALHOST_ONLY","1",1);
  rclcpp::init(argc,argv);testing::InitGoogleTest(&argc,argv);
  const auto result=RUN_ALL_TESTS();rclcpp::shutdown();return result;
}
