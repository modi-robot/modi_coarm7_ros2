#include "coarm7_hardware/core.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace coarm7_hardware {
namespace {
void require(bool b, const char *s) { if (!b) throw std::invalid_argument(s); }
bool positive(double d) { return std::isfinite(d) && d > 0; }
}
Config Config::load(const std::string &path) {
  const auto n = YAML::LoadFile(path);
  Config c;
  c.transport = n["transport"].as<std::string>();
  c.commissioning_confirmed = n["commissioning_confirmed"].as<bool>();
  if (const auto read_only = n["gripper_read_only"]) c.gripper_read_only = read_only.as<bool>();
  c.deactivate_policy = n["deactivate_policy"].as<std::string>();
  c.buses = n["buses"].as<std::array<std::string, 2>>();
  const auto t = n["timing"];
  c.period_us = t["period_us"].as<int>();
  c.joint_timeout_ms = t["joint_timeout_ms"].as<int>();
  c.feedback_snapshot_timeout_ms = t["feedback_snapshot_timeout_ms"].as<int>(15);
  c.gripper_timeout_ms = t["gripper_timeout_ms"].as<int>();
  c.gripper_poll_ms = t["gripper_poll_ms"].as<int>();
  c.gripper_response_ms = t["gripper_response_ms"].as<int>();
  c.gripper_command_ms = t["gripper_command_ms"].as<int>();
  c.gripper_position_ms = t["gripper_position_ms"].as<int>(std::max(50,c.gripper_command_ms));
  c.startup_timeout_ms = t["startup_timeout_ms"].as<int>();
  c.max_cycle_ms = t["max_cycle_ms"].as<double>();
  c.max_consecutive_overruns = t["max_consecutive_overruns"].as<int>();
  c.sdo_timeout_ms = t["sdo_timeout_ms"].as<int>(500);
  c.disable_settle_ms = t["disable_settle_ms"].as<int>(20);
  c.stop_wait_ms = t["stop_wait_ms"].as<int>(1000);
  c.stop_velocity_tolerance = t["stop_velocity_tolerance"].as<double>(0.02);
  c.device_report_file = n["device_report_file"].as<std::string>("");
  if (const auto rx = n["sdk_rx"]) {
    c.rx_cpu = rx["cpu"].as<int>(c.rx_cpu);
    c.rx_priority = rx["priority"].as<int>(c.rx_priority);
  }
  if (const auto tx = n["sdk_tx"]) {
    c.tx_cpu = tx["cpu"].as<int>(c.tx_cpu);
    c.tx_priority = tx["priority"].as<int>(c.tx_priority);
    c.tx_timestamp_diagnostics = tx["timestamp_diagnostics"].as<bool>(c.tx_timestamp_diagnostics);
    c.tx_trace_diagnostics = tx["trace_diagnostics"].as<bool>(c.tx_trace_diagnostics);
    c.tx_lock_memory = tx["lock_memory"].as<bool>(c.tx_lock_memory);
  }
  require(n["axes"].size() == kArms && n["grippers"].size() == kGrippers, "expected 14 axes and 2 grippers");
  for (std::size_t i = 0; i < kArms; ++i) {
    const auto a = n["axes"][i]; auto &d = c.axes[i];
    d.name = a["name"].as<std::string>(); d.bus = a["bus"].as<int>(); d.id = a["id"].as<int>();
    d.sign = a["sign"].as<double>(); d.offset = a["offset"].as<double>();
    d.lower = a["lower"].as<double>(); d.upper = a["upper"].as<double>();
    d.initial = a["initial"].as<double>();
    d.max_velocity = a["max_velocity"].as<double>(); d.max_acceleration = a["max_acceleration"].as<double>();
    d.following_error = a["following_error"].as<double>();
  }
  for (std::size_t i = 0; i < kGrippers; ++i) {
    const auto g = n["grippers"][i]; auto &d = c.grippers[i];
    d.name = g["name"].as<std::string>(); d.bus = g["bus"].as<int>();
    d.tool_id = g["tool_id"].as<int>(); d.device_id = g["device_id"].as<int>();
    d.angle = g["angle"].as<std::vector<double>>(); d.raw = g["raw"].as<std::vector<double>>();
    if (const auto bounds = g["command_raw"]) {
      require(bounds.IsSequence() && bounds.size() == 2, "command_raw requires [minimum, maximum]");
      d.command_raw_min = bounds[0].as<double>();
      d.command_raw_max = bounds[1].as<double>();
    }
    d.max_velocity = g["max_velocity"].as<double>();
    d.speed_raw = g["speed_raw"].as<int>(500);
    d.raw_endpoint_margin = g["raw_endpoint_margin"].as<double>();
  }
  c.validate(); return c;
}
void Config::validate() const {
  require(transport == "fake" || transport == "sdk", "transport must be fake or sdk");
  require(period_us == 5000, "this integration requires a 5000 us control period");
  require(joint_timeout_ms >= 5 && gripper_timeout_ms > gripper_poll_ms && gripper_poll_ms >= 5,
          "invalid feedback/poll timing");
  require(gripper_command_ms >= 5 && gripper_response_ms > 0 && startup_timeout_ms > 0,
          "invalid serial/startup timing");
  require(gripper_position_ms >= gripper_command_ms, "gripper position writes must respect the serial send gap");
  require(sdo_timeout_ms > 0 && disable_settle_ms >= 2 * period_us / 1000,
          "invalid SDO timeout/disable settling interval");
  require(stop_wait_ms >= 3 * period_us / 1000 && positive(stop_velocity_tolerance),
          "invalid stop wait/velocity threshold");
  require(positive(max_cycle_ms) && max_cycle_ms >= 5 && max_consecutive_overruns > 0,
          "invalid deadline policy");
  require(rx_cpu >= -1 && rx_priority >= 1 && rx_priority <= 99, "invalid SDK Rx settings");
  require(tx_cpu >= -1 && tx_priority >= 1 && tx_priority <= 99, "invalid SDK Tx settings");
  require(!buses[0].empty() && !buses[1].empty() && buses[0] != buses[1], "two distinct buses required");
  if (transport == "sdk") {
    require(commissioning_confirmed, "SDK transport locked: commissioning configuration has not been confirmed");
    require(deactivate_policy == "disable", "SDK transport requires confirmed disable policy");
    require(feedback_snapshot_timeout_ms >= period_us/1000 && sdk_cache_timeout_ms() >= 5,
            "joint_timeout_ms must cover the SDK cache and feedback_snapshot_timeout_ms budgets");
    require(sdk_extensions_available() ||
            stop_wait_ms >= sdk_cache_timeout_ms()+2*joint_timeout_ms+3*period_us/1000,
            "public SDK stop_wait_ms must allow three cache-independent standstill observations");
    require(!tx_timestamp_diagnostics || sdk_extensions_available(),
            "SDK timestamp diagnostics require COARM7_ENABLE_SDK_EXTENSIONS=ON; public SDK does not provide them");
  }
  std::set<std::string> names;
  std::set<std::pair<int, int>> nodes;
  for (const auto &a : axes) {
    require(!a.name.empty() && names.insert(a.name).second, "duplicate/empty joint name");
    require(a.bus >= 0 && a.bus < 2 && a.id >= 1 && a.id <= 62 && nodes.insert({a.bus, a.id}).second,
            "invalid/duplicate axis node");
    require((a.sign == 1 || a.sign == -1) && std::isfinite(a.offset), "invalid coordinate transform");
    require(std::isfinite(a.lower) && std::isfinite(a.upper) && a.lower < a.upper &&
            std::isfinite(a.initial) && a.initial >= a.lower && a.initial <= a.upper, "invalid joint limits/initial state");
    require(positive(a.max_velocity) && positive(a.max_acceleration) && positive(a.following_error), "invalid axis guard");
  }
  for (const auto &g : grippers) {
    require(!g.name.empty() && names.insert(g.name).second, "duplicate gripper name");
    require(g.bus >= 0 && g.bus < 2 && g.tool_id >= 1 && g.tool_id <= 62 &&
            nodes.insert({g.bus, g.tool_id}).second && g.device_id >= 1 && g.device_id <= 254, "invalid tool node/address");
    require(g.angle.size() >= 2 && g.angle.size() == g.raw.size(), "invalid calibration table");
    require(positive(g.max_velocity) && std::isfinite(g.raw_endpoint_margin) && g.raw_endpoint_margin >= 0,
            "invalid gripper guard");
    require(g.speed_raw >= 1 && g.speed_raw <= 1000, "gripper speed_raw must be in 1..1000");
    require(std::isfinite(g.command_raw_min) && std::isfinite(g.command_raw_max) &&
            g.command_raw_min >= 1 && g.command_raw_max <= 1000 &&
            std::floor(g.command_raw_min) == g.command_raw_min &&
            std::floor(g.command_raw_max) == g.command_raw_max &&
            g.command_raw_min < g.command_raw_max &&
            g.command_raw_min >= g.raw.back() && g.command_raw_max <= g.raw.front(),
            "invalid gripper command opening range");
    for (std::size_t i = 0; i < g.angle.size(); ++i) {
      require(std::isfinite(g.angle[i]) && std::isfinite(g.raw[i]) && g.raw[i] >= 0 && g.raw[i] <= 1000,
              "nonfinite/out-of-protocol calibration");
      if (i) require(g.angle[i] > g.angle[i-1] && g.raw[i] < g.raw[i-1], "calibration must be strictly monotonic");
    }
  }
}
namespace {
double interpolate(const std::vector<double> &x, const std::vector<double> &y, double v) {
  for (std::size_t i = 1; i < x.size(); ++i) {
    if (v <= x[i]) return y[i-1] + (v-x[i-1])/(x[i]-x[i-1])*(y[i]-y[i-1]);
  }
  return y.back();
}
}
double to_raw(const GripperConfig &g, double q) {
  if (!std::isfinite(q) || q < g.angle.front() || q > g.angle.back()) throw std::out_of_range("gripper angle");
  return std::clamp(interpolate(g.angle, g.raw, q),g.command_raw_min,g.command_raw_max);
}
double to_angle(const GripperConfig &g, double raw) {
  if (!std::isfinite(raw) || raw < g.raw.back()-g.raw_endpoint_margin || raw > g.raw.front()+g.raw_endpoint_margin)
    throw std::out_of_range("gripper feedback opening");
  raw = std::clamp(raw, g.raw.back(), g.raw.front());
  for (std::size_t i = 1; i < g.raw.size(); ++i) {
    if (raw >= g.raw[i]) return g.angle[i-1] + (raw-g.raw[i-1])/(g.raw[i]-g.raw[i-1])*(g.angle[i]-g.angle[i-1]);
  }
  return g.angle.back();
}
}  // namespace coarm7_hardware
