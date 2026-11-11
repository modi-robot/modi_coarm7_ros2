#pragma once
#include "coarm7_hardware/core.hpp"
#include <hardware_interface/system_interface.hpp>
#include <mutex>
namespace coarm7_hardware {
class CoArm7System final : public hardware_interface::SystemInterface {
public:
  ~CoArm7System() override;
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string> &, const std::vector<std::string> &) override;
private:
  void report_fault_once(const char *phase);
  void export_effective_limits();
  std::string effective_limits_file_;
  bool fault_reported_ = false;
  bool next_claims(const std::vector<std::string> &, const std::vector<std::string> &, std::array<bool,kCommands> &) const;
  std::unique_ptr<Engine> engine_;
  std::vector<int> indices_;
  std::vector<double> multipliers_, offsets_, state_position_, state_velocity_;
  std::array<std::string,kCommands> names_;
  std::array<bool,kCommands> claims_{};
  std::mutex lifecycle_mutex_;
  bool configured_ = false;
};
}
