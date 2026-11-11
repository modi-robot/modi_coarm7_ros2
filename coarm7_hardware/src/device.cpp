#include "coarm7_hardware/device.hpp"
#include "modi_joint_canevo.h"
#include <ostream>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace coarm7_hardware {
bool read_drive_parameters(ParameterReader &r, DriveParameters &out, std::string &error) {
  DriveParameters p;
  auto failed = [&](uint8_t index, uint8_t sub, int rc) {
    std::ostringstream s;
    s << "SDO read 0x" << std::hex << unsigned(index) << "/0x" << unsigned(sub)
      << " failed, rc=" << std::dec << rc;
    error = s.str(); return false;
  };
  auto u16 = [&](uint8_t index, uint8_t sub, uint16_t &v) {
    const int rc = r.u16(index, sub, v); return rc == 0 || failed(index, sub, rc);
  };
  auto f32 = [&](uint8_t index, uint8_t sub, double &v, double factor) {
    float raw = 0;
    const int rc = r.f32(index, sub, raw);
    if (rc != 0) return failed(index, sub, rc);
    v = raw * (r.physical_units() ? 1.0 : factor); return true;
  };
  constexpr double rpm_to_rad_s = 0.10471975511965977;
  constexpr double deg_to_rad = 0.017453292519943295;
  if (!u16(0, 1, p.protocol) || !u16(0, 0x0D, p.firmware) || !u16(0, 0x0E, p.hardware) ||
      !u16(0, 0x0F, p.sync_us) || !u16(0, 0x10, p.comm_timeout_ms) ||
      !u16(1, 0x0D, p.brake_status) ||
      !f32(1, 2, p.max_velocity, rpm_to_rad_s) ||
      !f32(1, 3, p.max_acceleration, rpm_to_rad_s) ||
      !f32(1, 4, p.max_deceleration, rpm_to_rad_s) ||
      !f32(1, 5, p.stop_deceleration, rpm_to_rad_s)) return false;
  // Driver position settings are read for diagnostics only. They neither
  // tighten URDF bounds nor determine whether configuration can proceed.
  float upper = 0, lower = 0;
  p.upper_result = r.f32(1, 6, upper);
  p.lower_result = r.f32(1, 7, lower);
  const double position_factor = r.physical_units() ? 1.0 : deg_to_rad;
  if (p.upper_result == 0) p.upper = upper * position_factor;
  if (p.lower_result == 0) p.lower = lower * position_factor;
  p.position_bounds_valid = p.upper_result == 0 && p.lower_result == 0 &&
    std::isfinite(p.lower) && std::isfinite(p.upper) && p.lower < p.upper;
  // Kt is useful diagnostic information, but is not required for position control.
  // Preserve an unavailable/invalid coefficient as zero, never export fabricated Nm.
  float kt = 0;
  if (r.f32(0x1F, 3, kt) == 0 && std::isfinite(kt) && kt > 0) p.torque_constant = kt;
  const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
  if (!p.protocol || !p.firmware || !p.hardware || !p.sync_us || !p.comm_timeout_ms ||
      p.brake_status > 2 || !positive(p.max_velocity) || !positive(p.max_acceleration) ||
      !positive(p.max_deceleration) || !positive(p.stop_deceleration)) {
    error = "invalid drive parameter snapshot"; return false;
  }
  // Read the persistent fault register before the first SYNC. A session may
  // start after the original EMCY event, so its Rx cache can be empty.
  uint16_t fault_code = 0;
  p.fault_code_result = r.u16(0,0x08,fault_code);
  if (p.fault_code_result == 0) p.fault_code=fault_code;
  p.checked_sdo_results = r.checked_results();
  out = p; error.clear(); return true;
}
bool constrain_axis(AxisConfig &a, const DriveParameters &p, const Config &c, std::string &error) {
  if (p.sync_us != c.period_us || p.comm_timeout_ms * 1000 <= c.period_us) {
    error = "drive SYNC period/watchdog incompatible with configured control period"; return false;
  }
  AxisConfig next = a;  // Position bounds were taken from the loaded URDF.
  next.max_velocity = std::min(a.max_velocity, p.max_velocity);
  next.max_acceleration = std::min(a.max_acceleration, p.max_acceleration);
  a = next; error.clear(); return true;
}
bool axis_feedback_fresh(const AxisFeedback &f, const Config &c, int64_t now) noexcept {
  if (!f.valid || f.pdo_result != 0) return false;
  const auto stamp = f.sdk_checked ? f.checked_ns : f.received_ns;
  const auto limit = f.sdk_checked ? c.feedback_snapshot_timeout_ms : c.joint_timeout_ms;
  return stamp > 0 && now >= stamp && now-stamp <= limit*1000000LL;
}
bool axis_feedback_after(const AxisFeedback &f, const Config &c, int64_t since) noexcept {
  if (f.sdk_checked)
    return f.checked_ns > since+c.sdk_cache_timeout_ms()*1000000LL;
  return f.received_ns > since;
}
bool drive_allows_recovery(uint16_t statusword, bool allow_faults) noexcept {
  const auto state = static_cast<CanEvoServoState>(statusword & 0xF);
  return state == CanEvoServoState::kDisabled || state == CanEvoServoState::kReady ||
         (allow_faults && (state == CanEvoServoState::kFault || state == CanEvoServoState::kInitFault));
}

namespace {
constexpr uint16_t sync_fault = static_cast<uint16_t>(CanEvoFault::kSyncError);

bool read_recovery_faults(DriveFaultRecovery &drives, const Config &config, std::ostream &log,
                          std::array<uint16_t,kArms> &codes, bool before_reset) {
  bool ok = true;
  for (std::size_t i=0; i<kArms; ++i) {
    const auto &axis = config.axes[i];
    const int result = drives.read_drive_fault(i,codes[i]);
    log << "Drive fault " << (before_reset ? "before" : "final") << " joint=" << axis.name
        << " bus=" << config.buses[axis.bus] << " node_id=" << axis.id << " read_result=" << result;
    if (result == 0) log << " code=0x" << std::hex << codes[i] << std::dec;
    log << '\n';
    if (result != 0 || (codes[i] != 0 && (!before_reset || codes[i] != sync_fault))) ok = false;
  }
  return ok;
}
}

bool recover_drive_faults(DriveFaultRecovery &drives, const Config &config, std::ostream &log) {
  std::array<uint16_t,kArms> before{};
  if (!read_recovery_faults(drives,config,log,before,true)) {
    log << "Clear faults refused: every joint must report zero or FF05 before reset\n";
    return false;
  }

  for (std::size_t i=0; i<kArms; ++i) {
    if (before[i] != sync_fault) continue;
    const auto &axis = config.axes[i];
    if (!drives.wait_for_stationary_drives(true)) {
      log << "Clear faults refused: fresh stationary feedback from non-running drives is required"
          << " before reset joint=" << axis.name << '\n';
      return false;
    }

    // SYNC is now active for PDO feedback. Confirm the selected drive still
    // reports FF05 before making any controlword change.
    uint16_t current = 0;
    const int current_result = drives.read_drive_fault(i,current);
    if (current_result != 0 || current != sync_fault) {
      log << "Clear faults refused: FF05 no longer verified joint=" << axis.name
          << " read_result=" << current_result;
      if (current_result == 0) log << " code=0x" << std::hex << current << std::dec;
      log << '\n';
      return false;
    }
    const int clear_result = drives.clear_drive_fault(i);
    uint16_t after = 0;
    const int read_result = drives.read_drive_fault(i,after);
    log << "Clear faults joint=" << axis.name << " bus=" << config.buses[axis.bus]
        << " node_id=" << axis.id << " before=0x" << std::hex << current << std::dec
        << " clear_result=" << clear_result << " read_after_result=" << read_result;
    if (read_result == 0) log << " after=0x" << std::hex << after << std::dec;
    log << '\n';
    if (clear_result != 0 || read_result != 0 || after != 0) return false;
  }

  if (!drives.wait_for_stationary_drives(false)) {
    log << "Clear faults incomplete: drives have not reported fresh stationary disabled/ready feedback\n";
    return false;
  }
  std::array<uint16_t,kArms> after{};
  if (!read_recovery_faults(drives,config,log,after,false)) {
    log << "Clear faults incomplete: every drive must report zero after recovery\n";
    return false;
  }
  return true;
}

bool advance_axis(const AxisConfig &axis, double target, double dt,
                  double &position, double &velocity) noexcept {
  const double dv = axis.max_acceleration*dt;
  // Keep enough room for this step and a bounded stop. An abrupt reversal
  // must decelerate before moving toward the new target.
  const double delta = target-position;
  const double twice_work = 2*axis.max_acceleration*std::abs(delta);
  const double braking_speed = twice_work/(std::sqrt(dv*dv+twice_work)+dv);
  const double desired = std::copysign(std::min(axis.max_velocity,braking_speed),delta);
  const double speed = std::clamp(desired,velocity-dv,velocity+dv);
  const double next = position+speed*dt;
  if (!std::isfinite(next) || next < axis.lower-1e-10 || next > axis.upper+1e-10) return false;
  const double bounded = std::clamp(next,axis.lower,axis.upper);
  velocity = (bounded-position)/dt;
  position = bounded;
  return true;
}
bool request_hold(StopActions &a, const Positions &p) noexcept {
  bool ok = true;
  for (std::size_t i = 0; i < kArms; ++i) if (!a.hold(i, p[i])) ok = false;
  return ok;
}
bool request_quick_stop(StopActions &a, const Positions &p) noexcept {
  bool ok = true;
  // A cached quick-stop bit only reaches the drive in the following PDO.
  // Continue through every axis and tool even if an earlier send fails.
  for (std::size_t i = 0; i < kArms; ++i) {
    if (!a.quick_stop(i)) ok = false;
    if (!a.hold(i, p[i])) ok = false;
  }
  for (std::size_t i = 0; i < kGrippers; ++i) if (!a.stop_gripper(i)) ok = false;
  return ok;
}
bool disable_all(StopActions &a) noexcept {
  bool ok = true;
  for (std::size_t i=0;i<kArms;++i) if (!a.disable(i)) ok = false;
  return ok;
}
bool StandstillMonitor::update(const std::array<AxisFeedback,kArms> &feedback) noexcept {
  for(const auto &f:feedback) if(!f.valid || !std::isfinite(f.velocity) || std::abs(f.velocity)>tolerance_) {
    consecutive_=0; return false;
  }
  return ++consecutive_ >= 3;
}
bool FreshStandstillMonitor::update(const std::array<AxisFeedback,kArms> &feedback, int64_t now) noexcept {
  for (const auto &f:feedback) {
    if (!axis_feedback_fresh(f,config_,now) || !std::isfinite(f.velocity) ||
        std::abs(f.velocity)>config_.stop_velocity_tolerance) {
      monitor_=StandstillMonitor(config_.stop_velocity_tolerance);
      return false;
    }
  }
  for (std::size_t i=0;i<kArms;++i) {
    const auto &f=feedback[i];
    if (!axis_feedback_after(f,config_,after_)) return false;
    const auto stamp=f.sdk_checked ? f.checked_ns : f.received_ns;
    const auto minimum_gap=f.sdk_checked ? config_.joint_timeout_ms*1000000LL : 0;
    if (stamp <= previous_[i]+minimum_gap) return false;
  }
  for (std::size_t i=0;i<kArms;++i)
    previous_[i]=feedback[i].sdk_checked ? feedback[i].checked_ns : feedback[i].received_ns;
  return monitor_.update(feedback);
}
const char *sync_phase_name(SyncPhase phase) noexcept {
  switch (phase) {
    case SyncPhase::configure: return "configure";
    case SyncPhase::control: return "control";
    case SyncPhase::activate: return "activate";
    case SyncPhase::deactivate: return "deactivate";
    case SyncPhase::fault_stop: return "fault_stop";
  }
  return "unknown";
}
void SyncCadence::begin_batch(int64_t now_ns) noexcept {
  first_bus_start_ns_ = 0;
  const auto scheduled_ns = next_send_ns_ ? next_send_ns_ : now_ns;
  const auto lateness_ns = std::max<int64_t>(0,now_ns-scheduled_ns);
  max_lateness_ns_ = std::max(max_lateness_ns_,lateness_ns);
  // Keep the original 5 ms grid after an ordinary late wakeup.
  // Rebase only if the complete cycle reaches the following slot.
  next_send_ns_ = scheduled_ns+period_ns_;
}
void SyncCadence::finish_batch(int64_t now_ns) noexcept {
  if (now_ns >= next_send_ns_) {
    ++reschedules_;
    next_send_ns_ = now_ns+period_ns_;
  }
}
void SyncCadence::record(std::size_t bus, int64_t started_ns, int64_t ended_ns,
                         bool sent, SyncPhase phase) noexcept {
  auto &b=buses_[bus];
  auto &t=b.timing;
  t.max_write_ns=std::max(t.max_write_ns,ended_ns-started_ns);
  if (!sent) {
    ++t.failures;
    return;
  }
  if (bus == 0) {
    first_bus_start_ns_ = started_ns;
  } else if (first_bus_start_ns_ > 0 && started_ns >= first_bus_start_ns_) {
    const auto skew_ns = started_ns-first_bus_start_ns_;
    auto &skew = bus_skew_;
    if (skew.samples++ == 0) skew.shortest_ns = skew.longest_ns = skew_ns;
    else {
      skew.shortest_ns = std::min(skew.shortest_ns,skew_ns);
      skew.longest_ns = std::max(skew.longest_ns,skew_ns);
    }
  }
  if (b.last_sent_ns) {
    const SyncInterval interval{ended_ns-b.last_sent_ns,b.last_phase,phase};
    if (!t.intervals || interval.ns<t.shortest.ns) t.shortest=interval;
    if (!t.intervals || interval.ns>t.longest.ns) t.longest=interval;
    t.recent[t.intervals % t.recent.size()] = interval;
    ++t.intervals;
  }
  b.last_sent_ns=ended_ns;
  b.last_phase=phase;
}
}  // namespace coarm7_hardware
