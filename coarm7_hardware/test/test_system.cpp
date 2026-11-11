#include "coarm7_hardware/system.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
using namespace coarm7_hardware;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
static hardware_interface::HardwareInfo info() {
  hardware_interface::HardwareInfo h;h.name="test";h.type="system";
  h.hardware_parameters["config_file"]=CONFIG_PATH;
  const auto c=Config::load(CONFIG_PATH);
  auto add=[&](const std::string &name,const std::string &mimic="") {
    hardware_interface::ComponentInfo j;j.name=name;j.type="joint";
    hardware_interface::InterfaceInfo p;p.name="position";
    hardware_interface::InterfaceInfo v;v.name="velocity";
    j.state_interfaces={p,v};
    if(mimic.empty())j.command_interfaces={p};else {j.parameters["mimic"]=mimic;j.parameters["multiplier"]="1";}
    h.joints.push_back(j);
  };
  for(const auto &a:c.axes)add(a.name);
  for(const auto &g:c.grippers) {
    add(g.name); auto &j=h.joints.back();
    hardware_interface::InterfaceInfo a,b;a.name="goal_id";b.name="stop_id";
    j.command_interfaces.push_back(a);j.command_interfaces.push_back(b);
    a.name="feedback_stamp";b.name="stop_ack";
    j.state_interfaces.push_back(a);j.state_interfaces.push_back(b);
  }
  add("L_finger_mimic",c.grippers[0].name);add("R_finger_mimic",c.grippers[1].name);
  std::ostringstream xml; xml << std::setprecision(17) << "<robot name='test'><link name='base'/>";
  for(const auto &a:c.axes) xml << "<link name='" << a.name << "_link'/><joint name='" << a.name
    << "' type='revolute'><parent link='base'/><child link='" << a.name << "_link'/>"
    << "<limit effort='10' velocity='1' lower='" << a.lower << "' upper='" << a.upper << "'/></joint>";
  xml << "</robot>"; h.original_xml=xml.str();
  return h;
}
TEST(System, LifecycleInterfacesAndMimicReadback) {
  CoArm7System s;rclcpp_lifecycle::State state;
  ASSERT_EQ(s.on_init(info()),CallbackReturn::SUCCESS);
  auto commands=s.export_command_interfaces();auto states=s.export_state_interfaces();
  EXPECT_EQ(commands.size(),20u);EXPECT_EQ(states.size(),40u);
  ASSERT_EQ(s.on_configure(state),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.on_activate(state),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.perform_command_mode_switch({"L_4C2_r_1_joint/position","L_4C2_r_1_joint/goal_id","L_4C2_r_1_joint/stop_id"},{}),return_type::OK);
  commands[14].set_value(0.4);commands[16].set_value(1);
  const auto t=rclcpp::Time(0);const auto dt=rclcpp::Duration::from_seconds(0.005);
  ASSERT_EQ(s.read(t,dt),return_type::OK);ASSERT_EQ(s.write(t,dt),return_type::OK);ASSERT_EQ(s.read(t,dt),return_type::OK);
  EXPECT_GT(states[28].get_value(),0);EXPECT_DOUBLE_EQ(states[32].get_value(),states[28].get_value());
  EXPECT_EQ(s.on_deactivate(state),CallbackReturn::SUCCESS);EXPECT_EQ(s.on_cleanup(state),CallbackReturn::SUCCESS);
}
TEST(System, RejectsUnexpectedInterfacesAndMissingAxes) {
  CoArm7System s;auto h=info();h.joints[0].command_interfaces[0].name="effort";
  EXPECT_EQ(s.on_init(h),CallbackReturn::ERROR);
  CoArm7System other;h=info();h.joints.erase(h.joints.begin());EXPECT_EQ(other.on_init(h),CallbackReturn::ERROR);
}
TEST(System, RejectsVelocityModeAndStopsOnNan) {
  CoArm7System s;rclcpp_lifecycle::State state;ASSERT_EQ(s.on_init(info()),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.on_configure(state),CallbackReturn::SUCCESS);ASSERT_EQ(s.on_activate(state),CallbackReturn::SUCCESS);
  EXPECT_EQ(s.prepare_command_mode_switch({"L_joint1/velocity"},{}),return_type::ERROR);
  ASSERT_EQ(s.perform_command_mode_switch({"L_joint1/position"},{}),return_type::OK);
  auto commands=s.export_command_interfaces();commands[0].set_value(std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(s.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(0.005)),return_type::ERROR);
  EXPECT_EQ(s.on_error(state),CallbackReturn::SUCCESS);
}


TEST(System, FirstWriteRejectionIsLoggedOnceBeforeReadCanBeDisabled) {
  CoArm7System s; rclcpp_lifecycle::State state;
  ASSERT_EQ(s.on_init(info()),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.on_configure(state),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.on_activate(state),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.perform_command_mode_switch({"L_joint5/position"},{}),return_type::OK);
  auto commands=s.export_command_interfaces(); commands[4].set_value(100);
  testing::internal::CaptureStderr();
  EXPECT_EQ(s.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),return_type::ERROR);
  const auto message=testing::internal::GetCapturedStderr();
  EXPECT_NE(message.find("First hardware rejection phase=write"),std::string::npos);
  EXPECT_NE(message.find("joint=L_joint5"),std::string::npos);
  EXPECT_NE(message.find("target_rad=100"),std::string::npos);
  EXPECT_NE(message.find("effective_bounds_rad="),std::string::npos);
  commands[4].set_value(0);
  testing::internal::CaptureStderr();
  EXPECT_EQ(s.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),return_type::ERROR);
  EXPECT_EQ(s.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),return_type::ERROR);
  EXPECT_EQ(testing::internal::GetCapturedStderr().find("First hardware rejection"),std::string::npos);
  s.on_cleanup(state);
}
TEST(System, EffectiveLimitSnapshotExistsOnlyAfterSuccessfulConfigure) {
  char pattern[]="/tmp/coarm7-limit-test-XXXXXX";
  const char *folder=mkdtemp(pattern); ASSERT_NE(folder,nullptr);
  const auto path=std::filesystem::path(folder)/"limits.yaml";
  auto h=info(); h.hardware_parameters["effective_limits_file"]=path.string();
  CoArm7System s; rclcpp_lifecycle::State state;
  ASSERT_EQ(s.on_init(h),CallbackReturn::SUCCESS);
  EXPECT_FALSE(std::filesystem::exists(path));
  ASSERT_EQ(s.on_configure(state),CallbackReturn::SUCCESS);
  auto doc=YAML::LoadFile(path.string()); const auto config=Config::load(CONFIG_PATH);
  EXPECT_EQ(doc["schema"].as<int>(),1); EXPECT_EQ(doc["axes"].size(),14u);
  EXPECT_EQ(doc["coordinate_system"].as<std::string>(),"ros_joint_radians");
  for (const auto &a:config.axes) {
    EXPECT_DOUBLE_EQ(doc["axes"][a.name]["lower"].as<double>(),a.lower);
    EXPECT_DOUBLE_EQ(doc["axes"][a.name]["upper"].as<double>(),a.upper);
  }
  CoArm7System stale; EXPECT_EQ(stale.on_init(h),CallbackReturn::ERROR);
  EXPECT_EQ(s.on_cleanup(state),CallbackReturn::SUCCESS);
  EXPECT_FALSE(std::filesystem::exists(path));
  std::filesystem::remove_all(folder);
}

TEST(System, UrdfOverridesYamlPositionBoundsAndIsRequired) {
  auto h=info();
  const auto start=h.original_xml.find("<joint name='L_joint1'");
  const auto lo=h.original_xml.find("lower='",start)+7;
  h.original_xml.replace(lo,h.original_xml.find("'",lo)-lo,"-3.2");
  const auto hi=h.original_xml.find("upper='",start)+7;
  h.original_xml.replace(hi,h.original_xml.find("'",hi)-hi,"3.2");
  const auto path=std::filesystem::temp_directory_path()/("coarm7_urdf_test_"+std::to_string(getpid())+".yaml");
  std::filesystem::remove(path); h.hardware_parameters["effective_limits_file"]=path.string();
  CoArm7System s; rclcpp_lifecycle::State state;
  ASSERT_EQ(s.on_init(h),CallbackReturn::SUCCESS);
  ASSERT_EQ(s.on_configure(state),CallbackReturn::SUCCESS);
  const auto snapshot=YAML::LoadFile(path.string());
  EXPECT_DOUBLE_EQ(snapshot["axes"]["L_joint1"]["lower"].as<double>(),-3.2);
  EXPECT_DOUBLE_EQ(snapshot["axes"]["L_joint1"]["upper"].as<double>(),3.2);
  EXPECT_EQ(snapshot["position_limits_source"].as<std::string>(),"urdf");
  EXPECT_EQ(s.on_cleanup(state),CallbackReturn::SUCCESS);
  h=info(); h.original_xml.clear(); CoArm7System missing;
  EXPECT_EQ(missing.on_init(h),CallbackReturn::ERROR);
}
