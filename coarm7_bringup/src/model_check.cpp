#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <random_numbers/random_numbers.h>
#include <rclcpp/rclcpp.hpp>

std::string Read(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("Cannot read " + path);
  std::ostringstream content;
  content << stream.rdbuf();
  return content.str();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    if (argc < 3) throw std::runtime_error("Usage: coarm7_model_check URDF SRDF [samples]");
    auto node = std::make_shared<rclcpp::Node>("coarm7_model_check");
    robot_model_loader::RobotModelLoader::Options options(Read(argv[1]), Read(argv[2]));
    options.load_kinematics_solvers_ = false;
    robot_model_loader::RobotModelLoader loader(node, options);
    const auto model = loader.getModel();
    if (!model) throw std::runtime_error("Cannot load CoArm7 robot model");
    const auto group = model->getJointModelGroup("both_arms");
    if (!group || group->getVariableCount() != 14)
      throw std::runtime_error("both_arms must contain fourteen variables");
    for (const auto& name : model->getJointModelGroupNames()) {
      const auto g = model->getJointModelGroup(name);
      std::cout << "GROUP " << name << " active=" << g->getActiveJointModels().size()
                << " variables=" << g->getVariableCount() << '\n';
    }
    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    if (!state.setToDefaultValues(group, "home"))
      throw std::runtime_error("Missing both_arms home state");
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene.checkSelfCollision(request, result, state);
    std::cout << "HOME bounds=" << state.satisfiesBounds() << " collision=" << result.collision << '\n';
    for (const auto& item : result.contacts)
      std::cout << "HOME_CONTACT " << item.first.first << ' ' << item.first.second << '\n';
    const bool valid_home = state.satisfiesBounds() && !result.collision;
    // Sweep the complete gripper range, including mimic branches and loop pins.
    std::map<std::pair<std::string, std::string>, int> gripper_contacts;
    int invalid_gripper_states = 0;
    for (int i = 0; i <= 40; ++i) {
      const double q = 0.864 * (i / 40.0);
      for (const std::string side : {"L", "R"})
        state.setVariablePosition(side + "_4C2_r_1_joint", q);
      state.update();
      result.clear();
      scene.checkSelfCollision(request, result, state);
      if (result.collision || !state.satisfiesBounds()) ++invalid_gripper_states;
      for (const auto& item : result.contacts) ++gripper_contacts[item.first];
      if (i == 0 || i == 20 || i == 40) {
        const auto left = state.getGlobalLinkTransform("L_4C2_ee_link_2").translation();
        const auto right = state.getGlobalLinkTransform("L_4C2_ee_link_3").translation();
        std::cout << "GRIPPER q=" << q << " fingertip_frame_distance_m=" << (left-right).norm() << '\n';
      }
    }
    std::cout << "GRIPPER_SWEEP samples=41 invalid=" << invalid_gripper_states << '\n';
    for (const auto& item : gripper_contacts)
      std::cout << "GRIPPER_PAIR " << item.first.first << ' ' << item.first.second << ' ' << item.second << '\n';
    state.setToDefaultValues();
    state.setToDefaultValues(group, "home");
    random_numbers::RandomNumberGenerator random(72451);
    std::map<std::pair<std::string, std::string>, int> contacts;
    const int samples = argc > 3 ? std::stoi(argv[3]) : 1000;
    int valid = 0;
    for (int i = 0; i < samples; ++i) {
      state.setToRandomPositions(group, random);
      state.setToRandomPositions(model->getJointModelGroup("both_grippers"), random);
      state.update();
      result.clear();
      scene.checkSelfCollision(request, result, state);
      if (!result.collision) ++valid;
      for (const auto& item : result.contacts) ++contacts[item.first];
    }
    std::cout << "SAMPLES " << samples << " collision_free=" << valid << '\n';
    for (const auto& item : contacts)
      std::cout << "PAIR " << item.first.first << ' ' << item.first.second << ' ' << item.second << '\n';
    rclcpp::shutdown();
    return valid_home && invalid_gripper_states == 0 ? 0 : 2;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
