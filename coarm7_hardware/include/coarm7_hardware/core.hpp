#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace coarm7_hardware {
constexpr std::size_t kArms = 14, kGrippers = 2, kCommands = 16;
using Positions = std::array<double, kCommands>;
struct AxisConfig {
  std::string name;
  int bus = 0, id = 0;
  double sign = 1, offset = 0, lower = 0, upper = 0, initial = 0;
  double max_velocity = 0.5, max_acceleration = 2, following_error = 0.3;
};
struct GripperConfig {
  std::string name;
  int bus = 0, tool_id = 0, device_id = 1;
  // Piecewise-linear calibration: angle increases, raw opening decreases.
  std::vector<double> angle, raw;
  // Feedback calibration and the permitted command range are separate.
  double command_raw_min = 10, command_raw_max = 1000;
  int speed_raw = 500;  // EG2 speed setting, 1..1000; not ROS joint velocity.
  double max_velocity = 0.8, raw_endpoint_margin = 10;
};
struct Config {
  std::string transport = "fake", deactivate_policy = "pending";
  bool commissioning_confirmed = false, gripper_read_only = false;
  int period_us = 5000, joint_timeout_ms = 50, gripper_timeout_ms = 500;
  // In public-SDK mode, split the total feedback age budget between SDK cache
  // validity and delivery of a checked snapshot to the ROS thread.
  int feedback_snapshot_timeout_ms = 15;
  int gripper_poll_ms = 50, gripper_response_ms = 200, gripper_command_ms = 5;
  int gripper_position_ms = 50;  // Coalesce SDK position writes; polls retain their own cadence.
  int startup_timeout_ms = 1000, max_consecutive_overruns = 3;
  int sdo_timeout_ms = 500, disable_settle_ms = 20;
  int stop_wait_ms = 1000;
  double stop_velocity_tolerance = 0.02;
  std::string device_report_file;
  double max_cycle_ms = 25;
  int rx_cpu = -1, rx_priority = 90;
  int tx_cpu = -1, tx_priority = 90;
  bool tx_timestamp_diagnostics = false, tx_trace_diagnostics = true, tx_lock_memory = true;
  std::array<std::string, 2> buses{{"can0", "can1"}};
  std::array<AxisConfig, kArms> axes;
  std::array<GripperConfig, kGrippers> grippers;
  static Config load(const std::string &path);
  void validate() const;
  bool gripper_commands_enabled() const { return transport != "sdk" || !gripper_read_only; }
  int sdk_cache_timeout_ms() const { return joint_timeout_ms-feedback_snapshot_timeout_ms; }
};
bool sdk_extensions_available() noexcept;
double to_raw(const GripperConfig &, double angle);
double to_angle(const GripperConfig &, double raw);

struct AxisFeedback {
  double position = 0, velocity = 0;
  int64_t received_ns = 0;
  // Public SDK returns cache validity but no joint-PDO receive timestamp.
  // checked_ns is captured BEFORE calling that getter, never presented as Rx.
  int64_t checked_ns = 0;
  bool sdk_checked = false;
  bool valid = false, running_csp = true, fault = false;
  int pdo_result = 0;
  uint16_t statusword = 0, emcy = 0;
  bool emcy_valid = false;
};
struct GripperFeedback {
  double raw = 0;
  int64_t received_ns = 0;
  bool valid = false;
  uint8_t fault = 0, status = 0;
  int command_result = 0;  // Host protocol error, separate from device fault bits.
  uint64_t stop_ack = 0;  // SDK send accepted, NOT physical-stop acknowledgement.
};
struct Snapshot {
  int64_t sampled_ns = 0;
  bool applied_valid = false;
  Positions applied_position{};
  std::array<AxisFeedback, kArms> axes;
  std::array<GripperFeedback, kGrippers> grippers;
};
class Transport {
public:
  virtual ~Transport() = default;
  virtual bool open(const Config &) = 0;
  virtual void apply_runtime_limits(Config &) const {}
  virtual bool startup_feedback(Snapshot &) const { return false; }
  virtual void close() noexcept = 0;
  // The SDK transport returns its periodic worker’s latest feedback without waiting.
  virtual bool sample(int64_t now, Snapshot &) = 0;
  virtual bool activate(const Positions &hold) = 0;
  virtual bool limits_axis_steps() const noexcept { return false; }
  virtual void gripper_requests(const std::array<uint64_t,2> &, const std::array<uint64_t,2> &) {}
  virtual bool send(const Positions &position, const std::array<double, 2> &raw,
                    const std::array<bool, kGrippers> &gripper_claims) = 0;
  // Request a stop. True means transmit already confirmed, not physical acknowledgement.
  virtual bool stop(const Positions &hold) noexcept = 0;
  virtual bool deactivate(const Positions &hold) = 0;
};
class FakeTransport final : public Transport {
public:
  bool open(const Config &) override;
  void close() noexcept override { opened = false; }
  bool sample(int64_t, Snapshot &) override;
  bool activate(const Positions &) override { return opened; }
  bool send(const Positions &, const std::array<double, 2> &,
            const std::array<bool, kGrippers> &) override;
  void gripper_requests(const std::array<uint64_t,2> &g, const std::array<uint64_t,2> &s) override { goal_ids_=g; stop_ids_=s; }
  bool stop(const Positions &) noexcept override;
  bool deactivate(const Positions &p) override { ++deactivations; last_target=p; moving_.fill(false); handled_goals_=goal_ids_; return opened && !fail_send; }
  bool opened = false, fail_sample = false, fail_send = false;
  int stale_axis = -1, fault_axis = -1, wrong_mode_axis = -1, stale_gripper = -1;
  int bad_gripper = -1, stops = 0;
  int deactivations = 0;
  int64_t sample_delay_ns = 0;  // Offline injection: Rx arrival during a read.
  std::array<int, 2> sync_count{};
  Positions last_target{};
  std::array<bool,kGrippers> last_gripper_claims{};
  Snapshot feedback{};
private:
  Config config_;
  std::array<uint64_t,2> goal_ids_{}, stop_ids_{}, handled_goals_{}, handled_stops_{};
  std::array<bool,2> moving_{};
  int64_t previous_ns_ = 0;
};
std::unique_ptr<Transport> make_transport(const Config &);
// Standalone recovery under exclusive CAN ownership, without a servo session.
bool clear_sdk_drive_faults(const Config &);

enum class Fault { none, transport, feedback, mode, drive, gripper, command, tracking, deadline };
class Engine {
public:
  Engine(Config config, std::unique_ptr<Transport> transport);
  bool configure();
  bool activate(int64_t now);
  bool read(int64_t now);
  bool write();
  bool deactivate();
  void cleanup() noexcept;
  bool set_claims(const std::array<bool, kCommands> &claims);
  const Positions &positions() const { return position_; }
  const Positions &velocities() const { return velocity_; }
  Positions &commands() { return command_; }
  std::array<double,2> &gripper_goal_ids() { return goal_ids_; }
  std::array<double,2> &gripper_stop_ids() { return stop_ids_; }
  std::array<double,2> &gripper_stop_acks() { return stop_acks_; }
  std::array<double,2> &gripper_feedback_stamps() { return feedback_stamps_; }
  Fault fault() const { return fault_; }
  std::string fault_detail() const;
  bool active() const { return active_; }
  bool has_servo_session() const { return servo_session_; }
  bool stop_delivery_ok() const { return stop_delivery_ok_; }
  const Config &config() const { return config_; }
private:
  bool accept_feedback(int64_t now);
  bool trip(Fault reason, std::size_t index = kCommands, const char *cause = nullptr,
            int64_t feedback_checked_ns = 0);
  Config config_;
  std::unique_ptr<Transport> transport_;
  Snapshot snapshot_{};
  std::array<double,2> goal_ids_{}, stop_ids_{}, stop_acks_{}, feedback_stamps_{};
  Positions position_{}, velocity_{}, command_{}, sent_{}, sent_velocity_{};
  std::array<bool, kCommands> claimed_{};
  bool configured_ = false, active_ = false, have_feedback_ = false;
  bool servo_session_ = false, stop_delivery_ok_ = true;
  Fault fault_ = Fault::none;
  std::size_t fault_joint_ = kCommands;
  const char *fault_cause_ = nullptr;
  int64_t fault_feedback_checked_ns_ = 0;
  int64_t fault_ns_ = 0;
  double fault_target_ = 0, fault_position_ = 0, fault_lower_ = 0, fault_upper_ = 0;
  bool fault_claimed_ = false;
  int64_t previous_read_ns_ = 0;
  int overruns_ = 0;
};
int64_t monotonic_now_ns();
}  // namespace coarm7_hardware
