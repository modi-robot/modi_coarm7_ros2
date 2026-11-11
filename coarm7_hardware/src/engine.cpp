#include "coarm7_hardware/core.hpp"
#include "coarm7_hardware/device.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <thread>

namespace coarm7_hardware {
int64_t monotonic_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
Engine::Engine(Config c, std::unique_ptr<Transport> t) : config_(std::move(c)), transport_(std::move(t)) {}
bool Engine::configure() {
  config_.validate();
  if (configured_ || !transport_) return false;
  fault_ = Fault::none; have_feedback_ = false; previous_read_ns_ = 0; overruns_ = 0;
  fault_joint_ = kCommands;
  fault_cause_ = nullptr;
  fault_feedback_checked_ns_ = 0;
  claimed_.fill(false); sent_velocity_.fill(0);
  goal_ids_.fill(0); stop_ids_.fill(0); stop_acks_.fill(0); feedback_stamps_.fill(0);
  try { if (!transport_->open(config_)) { transport_->close(); return false; } }
  catch (...) { transport_->close(); throw; }
  transport_->apply_runtime_limits(config_);
  servo_session_ = false; stop_delivery_ok_ = true;
  if (transport_->startup_feedback(snapshot_) && !accept_feedback(monotonic_now_ns())) {
    transport_->close();
    return false;
  }
  configured_ = true;
  return true;
}
bool Engine::activate(int64_t now) {
  if (!configured_ || fault_ != Fault::none || !read(now) || !have_feedback_) return false;
  command_ = sent_ = position_; sent_velocity_.fill(0);
  servo_session_ = true;  // Also roll back a partially completed enable.
  if (!transport_->activate(position_)) return trip(Fault::mode);
  // Blocking SDO activation is not a missed normal servo cycle.
  active_ = true; previous_read_ns_ = 0; overruns_ = 0;
  return true;
}
bool Engine::trip(Fault reason, std::size_t index, const char *cause, int64_t feedback_checked_ns) {
  if (fault_ == Fault::none) {
    fault_ = reason;
    fault_joint_ = index;
    fault_cause_ = cause;
    fault_feedback_checked_ns_ = feedback_checked_ns;
    fault_ns_ = monotonic_now_ns();
    if (index < kCommands) {
      fault_claimed_ = claimed_[index];
      fault_target_ = claimed_[index] ? command_[index] : sent_[index];
      fault_position_ = position_[index];
      fault_lower_ = index < kArms ? config_.axes[index].lower : config_.grippers[index-kArms].angle.front();
      fault_upper_ = index < kArms ? config_.axes[index].upper : config_.grippers[index-kArms].angle.back();
    }
  }
  active_ = false; claimed_.fill(false);
  // A transport failure may prevent delivery; callers must not report the
  // physical stop as acknowledged merely because the request was attempted.
  if (servo_session_) stop_delivery_ok_ = transport_->stop(have_feedback_ ? position_ : sent_);
  return false;
}
bool Engine::read(int64_t now) {
  if (!configured_) return false;
  if (fault_ != Fault::none) {
    if (servo_session_) stop_delivery_ok_ = transport_->stop(position_);
    return false;
  }
  if (active_ && previous_read_ns_) {
    const auto elapsed = now-previous_read_ns_;
    if (elapsed <= 0) return trip(Fault::deadline);
    overruns_ = elapsed > config_.max_cycle_ms*1e6 ? overruns_+1 : 0;
    if (overruns_ >= config_.max_consecutive_overruns) return trip(Fault::deadline);
  }
  previous_read_ns_ = now;
  if (!transport_->sample(now, snapshot_)) return trip(Fault::transport);
  return accept_feedback(now);
}
bool Engine::accept_feedback(int64_t now) {
  // SDK Rx may publish a frame after read() began but before sample() ends.
  // Evaluate freshness at sample completion to avoid a false future-time fault.
  now = std::max(now, snapshot_.sampled_ns);
  Positions next = position_, vel = velocity_;
  for (std::size_t i = 0; i < kArms; ++i) {
    const auto &f = snapshot_.axes[i]; const auto &a = config_.axes[i];
    if (!f.valid) return trip(Fault::feedback,i,"PDO snapshot unavailable");
    if (!axis_feedback_fresh(f,config_,now))
      return trip(Fault::feedback,i,"invalid or expired feedback timestamp");
    if (!std::isfinite(f.position) || !std::isfinite(f.velocity))
      return trip(Fault::feedback,i,"nonfinite joint feedback");
    if (f.fault) return trip(Fault::drive,i,"driver fault");
    if (active_ && !f.running_csp) return trip(Fault::mode,i,"drive no longer running CSP");
    next[i] = a.sign*f.position+a.offset; vel[i] = a.sign*f.velocity;
    if (next[i] < a.lower-1e-5 || next[i] > a.upper+1e-5)
      return trip(Fault::feedback,i,"joint position outside effective bounds");
    const auto applied = snapshot_.applied_valid ? snapshot_.applied_position[i] : sent_[i];
    if (active_ && std::abs(next[i]-applied) > a.following_error)
      return trip(Fault::tracking,i,"joint following error");
  }
  for (std::size_t i = 0; i < kGrippers; ++i) {
    const auto &f = snapshot_.grippers[i];
    const auto reject = [this,i,now](const char *reason) {
      return trip(Fault::gripper,kArms+i,reason,now);
    };
    if (!f.valid) return reject("gripper feedback unavailable");
    if (f.received_ns <= 0 || now < f.received_ns)
      return reject("invalid gripper feedback timestamp");
    if (now-f.received_ns > static_cast<int64_t>(config_.gripper_timeout_ms)*1000000)
      return reject("gripper feedback expired");
    if (f.fault) return reject("gripper device fault");
    if (f.command_result) return reject("gripper command rejected or speed setup unverified");
    try { next[kArms+i] = to_angle(config_.grippers[i], f.raw); }
    catch (...) { return reject("gripper opening outside calibration"); }
    // Serial feedback is slower than the servo loop. Do not fabricate a 200 Hz
    // velocity measurement from repeated cached positions.
    vel[kArms+i] = 0;
    feedback_stamps_[i] = f.received_ns/1e9;
    stop_acks_[i] = static_cast<double>(f.stop_ack);
  }
  position_ = next; velocity_ = vel; have_feedback_ = true;
  return true;
}
std::string Engine::fault_detail() const {
  if (fault_joint_ >= kCommands) return {};
  std::ostringstream detail;
  detail << std::setprecision(17);
  if (fault_ == Fault::command) {
    const auto i = fault_joint_;
    const auto bus = i < kArms ? config_.axes[i].bus : config_.grippers[i-kArms].bus;
    detail << (fault_cause_ ? fault_cause_ : "command rejected")
           << " joint=" << (i < kArms ? config_.axes[i].name : config_.grippers[i-kArms].name)
           << " bus=" << config_.buses[bus]
           << " target_rad=" << fault_target_ << " feedback_rad=" << fault_position_
           << " effective_bounds_rad=[" << fault_lower_ << ',' << fault_upper_ << ']'
           << " claimed=" << std::boolalpha << fault_claimed_ << " host_mono_ns=" << fault_ns_;
    return detail.str();
  }
  if (fault_joint_ >= kArms) {
    const auto i = fault_joint_-kArms;
    const auto &gripper = config_.grippers[i];
    const auto &feedback = snapshot_.grippers[i];
    detail << fault_cause_ << " gripper=" << gripper.name
           << " bus=" << config_.buses[gripper.bus]
           << " tool_id=" << gripper.tool_id << " device_id=" << gripper.device_id
           << " feedback_valid=" << std::boolalpha << feedback.valid
           << " read_only=" << !config_.gripper_commands_enabled()
           << " opening_raw=" << feedback.raw
           << " opening_allowed_raw=[" << gripper.raw.back()-gripper.raw_endpoint_margin
           << ',' << gripper.raw.front()+gripper.raw_endpoint_margin << ']'
           << " status=0x" << std::hex << unsigned(feedback.status)
           << " gripper_fault=0x" << unsigned(feedback.fault) << std::dec
           << " command_result=" << feedback.command_result
           << " feedback_age_ms=";
    if (feedback.received_ns > 0 && fault_feedback_checked_ns_ >= feedback.received_ns)
      detail << (fault_feedback_checked_ns_-feedback.received_ns)/1e6;
    else
      detail << "unavailable";
    detail << " feedback_timeout_ms=" << config_.gripper_timeout_ms;
    return detail.str();
  }
  const auto &axis = config_.axes[fault_joint_];
  const auto &feedback = snapshot_.axes[fault_joint_];
  detail << fault_cause_ << " joint=" << axis.name
         << " bus=" << config_.buses[axis.bus] << " node_id=" << axis.id
         << " pdo_result=" << feedback.pdo_result;
  if (feedback.valid) {
    detail << " statusword=0x" << std::hex << feedback.statusword << std::dec
           << " q_sdk_rad=" << feedback.position
           << " q_ros_rad=" << axis.sign*feedback.position+axis.offset
           << " velocity_sdk_rad_s=" << feedback.velocity
           << " effective_bounds_rad=[" << axis.lower << ',' << axis.upper << ']';
  }
  if (feedback.emcy_valid) detail << " emcy=0x" << std::hex << feedback.emcy;
  else detail << " emcy=unavailable";
  detail << " feedback_clock=" << (feedback.sdk_checked ? "sdk_validity_check" : "pdo_receive")
         << " checked_ns=" << feedback.checked_ns << " received_ns=" << feedback.received_ns;
  return detail.str();
}
bool Engine::write() {
  if (!configured_ || fault_ != Fault::none) return false;
  if (!active_) return true;
  const double dt = config_.period_us/1e6;
  Positions next = sent_;
  // Validate every desired target before sending any axis of this cycle.
  for (std::size_t i = 0; i < kCommands; ++i) {
    const double q = claimed_[i] ? command_[i] : sent_[i];
    const double lo = i < kArms ? config_.axes[i].lower : config_.grippers[i-kArms].angle.front();
    const double hi = i < kArms ? config_.axes[i].upper : config_.grippers[i-kArms].angle.back();
    if (!std::isfinite(q) || q < lo || q > hi)
      return trip(Fault::command,i,!std::isfinite(q) ? "nonfinite position target" : "position target outside effective bounds");
  }
  for (std::size_t i = 0; i < kCommands; ++i) {
    if (!claimed_[i]) { sent_velocity_[i] = 0; continue; }
    if (i < kArms) {
      if (transport_->limits_axis_steps()) next[i] = command_[i];
      else if (!advance_axis(config_.axes[i],command_[i],dt,next[i],sent_velocity_[i]))
        return trip(Fault::command,i,"bounded joint step would leave effective limits");
    } else {
      // EG2 executes an endpoint internally; do not stream interpolated openings.
      next[i] = command_[i];
    }
  }
  std::array<double, 2> raw{};
  std::array<bool,kGrippers> gripper_claims{};
  for (std::size_t i = 0; i < kGrippers; ++i) {
    raw[i] = to_raw(config_.grippers[i], next[kArms+i]);
    gripper_claims[i] = claimed_[kArms+i] && config_.gripper_commands_enabled();
  }
  std::array<uint64_t,2> goals{},stops{};
  for (std::size_t i=0;i<kGrippers;++i) {
    const auto valid=[](double n) { return std::isfinite(n) && n>=0 && n<9007199254740992.0 && std::floor(n)==n; };
    if (!valid(goal_ids_[i]) || !valid(stop_ids_[i])) return trip(Fault::command,kArms+i,"invalid gripper request sequence");
    goals[i]=static_cast<uint64_t>(goal_ids_[i]); stops[i]=static_cast<uint64_t>(stop_ids_[i]);
  }
  transport_->gripper_requests(goals,stops);
  if (!transport_->send(next, raw, gripper_claims)) return trip(Fault::transport);
  sent_ = next;
  return true;
}
bool Engine::set_claims(const std::array<bool, kCommands> &c) {
  if (!active_ || fault_ != Fault::none) return false;
  for (std::size_t i=0;i<kGrippers;++i)
    if (c[kArms+i] && !config_.gripper_commands_enabled()) return false;
  for (std::size_t i = 0; i < kCommands; ++i) if (claimed_[i] != c[i]) {
    command_[i] = sent_[i] = position_[i]; sent_velocity_[i] = 0;
  }
  claimed_ = c; return true;
}
bool Engine::deactivate() {
  if (!configured_ || !servo_session_) return true;
  active_ = false; claimed_.fill(false); command_ = position_; sent_velocity_.fill(0);
  const bool ok = transport_->deactivate(position_);
  if (ok) servo_session_ = false;
  else { stop_delivery_ok_ = transport_->stop(position_); if (fault_ == Fault::none) fault_ = Fault::transport; }
  return ok;
}
void Engine::cleanup() noexcept {
  if (configured_) {
    try { if (servo_session_) deactivate(); } catch (...) { transport_->stop(position_); }
    transport_->close();
  }
  configured_ = active_ = have_feedback_ = servo_session_ = false;
}
bool FakeTransport::open(const Config &c) {
  config_ = c; opened = true; previous_ns_ = 0; sync_count.fill(0); feedback = {};
  last_gripper_claims.fill(false); moving_.fill(false);
  goal_ids_.fill(0); stop_ids_.fill(0); handled_goals_.fill(0); handled_stops_.fill(0);
  for (std::size_t i = 0; i < kArms; ++i) {
    last_target[i] = c.axes[i].initial;
    feedback.axes[i].position = (last_target[i]-c.axes[i].offset)/c.axes[i].sign;
  }
  for (std::size_t i = 0; i < kGrippers; ++i) {
    last_target[kArms+i] = c.grippers[i].angle.front(); feedback.grippers[i].raw = c.grippers[i].raw.front();
  }
  return true;
}
bool FakeTransport::sample(int64_t now, Snapshot &s) {
  if (!opened || fail_sample) return false;
  for (auto &v : sync_count) ++v;
  for (std::size_t i = 0; i < kArms; ++i) {
    auto &f = feedback.axes[i];
    if (static_cast<int>(i) != stale_axis) {
      const auto p = (last_target[i]-config_.axes[i].offset)/config_.axes[i].sign;
      f.velocity = previous_ns_ ? (p-f.position)/((now-previous_ns_)/1e9) : 0;
      f.position = p; f.received_ns = now; f.valid = true;
    }
    f.fault = static_cast<int>(i) == fault_axis;
    f.running_csp = static_cast<int>(i) != wrong_mode_axis;
  }
  for (std::size_t i = 0; i < kGrippers; ++i) {
    auto &f = feedback.grippers[i];
    if (static_cast<int>(i) != stale_gripper) {
      if (moving_[i] && previous_ns_ && now>previous_ns_) {
        const auto &g=config_.grippers[i];
        const double q=to_angle(g,f.raw), step=g.max_velocity*(now-previous_ns_)/1e9;
        f.raw=to_raw(g,q+std::clamp(last_target[kArms+i]-q,-step,step));
      }
      f.received_ns = now+sample_delay_ns; f.valid = true;
    }
    f.fault = static_cast<int>(i) == bad_gripper ? 1 : 0;
  }
  previous_ns_ = now; feedback.sampled_ns = now+sample_delay_ns; s = feedback; return true;
}
bool FakeTransport::send(const Positions &p, const std::array<double, 2> &raw,
                         const std::array<bool,kGrippers> &gripper_claims) {
  if (!opened || fail_send) return false;
  for (std::size_t i=0;i<kArms;++i) last_target[i]=p[i];
  for (std::size_t i=0;i<kGrippers;++i) {
    if (!gripper_claims[i] || stop_ids_[i]!=handled_stops_[i]) {
      moving_[i]=false; handled_goals_[i]=goal_ids_[i]; handled_stops_[i]=stop_ids_[i];
      feedback.grippers[i].stop_ack=stop_ids_[i];
    } else if (goal_ids_[i]!=handled_goals_[i]) {
      last_target[kArms+i]=to_angle(config_.grippers[i],raw[i]);
      moving_[i]=true; handled_goals_[i]=goal_ids_[i];
    }
  }
  last_gripper_claims = gripper_claims;
  return true;
}
bool FakeTransport::stop(const Positions &hold) noexcept { ++stops; last_target = hold; moving_.fill(false); return opened && !fail_send; }
}  // namespace coarm7_hardware
