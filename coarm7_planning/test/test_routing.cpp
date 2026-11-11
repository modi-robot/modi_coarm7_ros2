#include "coarm7_planning/gripper_goal.hpp"
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <gtest/gtest.h>
#include <sstream>
#include <algorithm>
#include <limits>
using moveit_msgs::msg::MotionPlanRequest;
namespace {
moveit::core::RobotModelPtr model(bool special=false) {
  std::ostringstream xml,sem;xml<<"<robot name='test'><link name='base'/>";sem<<"<robot name='test'>";
  for(const auto &side:{std::string("L"),std::string("R")}) {
    sem<<"<group name='"<<(side=="L"?"left_gripper":"right_gripper")<<"'>";
    for(int i=0;i<6;++i) {
      std::string name=side+"_4C2_"+(i==0?"r_1":"mimic"+std::to_string(i))+"_joint";
      xml<<"<link name='"<<name<<"_link'/><joint name='"<<name<<"' type='revolute'><parent link='base'/><child link='"<<name<<"_link'/><axis xyz='0 1 0'/><limit lower='-4' upper='4' velocity='1' effort='1'/>";
      if(i)xml<<"<mimic joint='"<<side<<"_4C2_r_1_joint' multiplier='"<<((special&&i==1)?2:1)<<"' offset='"<<((special&&i==1)?0.1:0)<<"'/>";
      xml<<"</joint>";sem<<"<joint name='"<<name<<"'/>";
    }
    sem<<"</group>";
  }
  xml<<"</robot>";sem<<"<group name='both_grippers'><group name='left_gripper'/><group name='right_gripper'/></group></robot>";
  auto u=urdf::parseURDF(xml.str());auto s=std::make_shared<srdf::Model>();s->initString(*u,sem.str());
  return std::make_shared<moveit::core::RobotModel>(u,s);
}
MotionPlanRequest goal(const moveit::core::RobotModelPtr &m,const std::string &group,bool rviz=true) {
  MotionPlanRequest r;r.group_name=group;r.planner_id="RRTConnectkConfigDefault";r.pipeline_id="ompl";
  r.num_planning_attempts=10;r.max_velocity_scaling_factor=1;r.max_acceleration_scaling_factor=0.5;
  moveit::core::RobotState state(m);state.setToDefaultValues();const auto *g=m->getJointModelGroup(group);
  state.setJointGroupActivePositions(g,std::vector<double>(g->getActiveJointModels().size(),0.4));
  r.goal_constraints={kinematic_constraints::constructGoalConstraints(state,g,0.001)};
  if(!rviz) {
    auto &joints=r.goal_constraints[0].joint_constraints;
    joints.erase(std::remove_if(joints.begin(),joints.end(),[&](const auto &j){return m->getJointModel(j.joint_name)->getMimic()!=nullptr;}),joints.end());
  }
  return r;
}
bool normalize(const moveit::core::RobotModelPtr &m,const MotionPlanRequest &r,MotionPlanRequest &out) {
  std::string reason;return coarm7_planning::normalize_gripper_goal(*m,r,out,reason);
}
}
TEST(Routing, RealMoveItGoalBuilderIncludesMimicsAndNormalizesAllThreeGroups) {
  auto m=model();
  for(const auto &name:{"left_gripper","right_gripper","both_grippers"}) {
    auto input=goal(m,name);auto expected=goal(m,name,false);MotionPlanRequest out;
    ASSERT_EQ(input.goal_constraints[0].joint_constraints.size(),std::string(name)=="both_grippers"?12u:6u);
    ASSERT_TRUE(normalize(m,input,out));expected.planner_id="PTP";expected.num_planning_attempts=1;
    EXPECT_EQ(out,expected);
  }
}
TEST(Routing, IndependentOnlyRequestsRemainSupported) {
  auto m=model();auto input=goal(m,"both_grippers",false);MotionPlanRequest out;ASSERT_TRUE(normalize(m,input,out));
  auto expected=input;expected.planner_id="PTP";expected.num_planning_attempts=1;EXPECT_EQ(out,expected);
}
TEST(Routing, MimicFactorAndOffsetComeFromModel) {
  auto m=model(true);MotionPlanRequest out;ASSERT_TRUE(normalize(m,goal(m,"both_grippers"),out));
}
TEST(Routing, ContradictoryMimicTargetIsRejected) {
  auto m=model();auto r=goal(m,"left_gripper");MotionPlanRequest out;
  for(auto &c:r.goal_constraints[0].joint_constraints)if(m->getJointModel(c.joint_name)->getMimic()){c.position+=0.1;break;}
  EXPECT_FALSE(normalize(m,r,out));
}
TEST(Routing, DuplicateAndUnknownAndCrossGroupJointsAreRejected) {
  auto m=model();MotionPlanRequest out;auto r=goal(m,"left_gripper");
  r.goal_constraints[0].joint_constraints.push_back(r.goal_constraints[0].joint_constraints.front());EXPECT_FALSE(normalize(m,r,out));
  r=goal(m,"left_gripper");r.goal_constraints[0].joint_constraints[0].joint_name="R_4C2_r_1_joint";EXPECT_FALSE(normalize(m,r,out));
  r.goal_constraints[0].joint_constraints[0].joint_name="unknown";EXPECT_FALSE(normalize(m,r,out));
}
TEST(Routing, MissingMasterAndPartialBothGrippersAreRejected) {
  auto m=model();MotionPlanRequest out;auto r=goal(m,"both_grippers",false);r.goal_constraints[0].joint_constraints.pop_back();EXPECT_FALSE(normalize(m,r,out));
}
TEST(Routing, NonfiniteAndInvalidToleranceAreRejected) {
  auto m=model();MotionPlanRequest out;auto r=goal(m,"left_gripper");r.goal_constraints[0].joint_constraints[0].position=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(normalize(m,r,out));
  r=goal(m,"left_gripper");r.goal_constraints[0].joint_constraints[0].tolerance_above=-1;EXPECT_FALSE(normalize(m,r,out));
}
TEST(Routing, PosePathTrajectoryAndAlternativeGoalsAreRejected) {
  auto m=model();MotionPlanRequest out;auto r=goal(m,"left_gripper");r.goal_constraints[0].position_constraints.resize(1);EXPECT_FALSE(normalize(m,r,out));
  r=goal(m,"left_gripper");r.path_constraints=r.goal_constraints.front();EXPECT_FALSE(normalize(m,r,out));
  r=goal(m,"left_gripper");r.trajectory_constraints.constraints=r.goal_constraints;EXPECT_FALSE(normalize(m,r,out));
  r=goal(m,"left_gripper");r.goal_constraints.push_back(r.goal_constraints.front());EXPECT_FALSE(normalize(m,r,out));
}
TEST(Routing, ArmAndUnknownRequestsStayUnchanged) {
  for(const auto &name:{"left_arm","right_arm","both_arms","unknown","left_gripper_extra"}) {
    MotionPlanRequest r;r.group_name=name;r.planner_id="custom";r.num_planning_attempts=7;
    r.path_constraints.name="keep";EXPECT_EQ(coarm7_planning::routed_request(r),r);
  }
}
