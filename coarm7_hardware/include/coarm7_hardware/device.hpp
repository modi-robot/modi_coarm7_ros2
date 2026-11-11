#pragma once
#include "coarm7_hardware/core.hpp"
#include <atomic>
#include <iosfwd>
#include <type_traits>

namespace coarm7_hardware {
// Blocking SDO reads are used only during configure, before servo activation.
class ParameterReader {
public:
  virtual ~ParameterReader() = default;
  virtual int u16(uint8_t index, uint8_t sub, uint16_t &value) = 0;
  virtual int f32(uint8_t index, uint8_t sub, float &value) = 0;
  virtual bool physical_units() const { return false; }
  virtual bool checked_results() const { return true; }
};
struct DriveParameters {
  uint16_t protocol = 0, firmware = 0, hardware = 0;
  uint16_t sync_us = 0, comm_timeout_ms = 0, brake_status = 0;
  uint16_t fault_code = 0;
  int fault_code_result = -1;
  bool checked_sdo_results = true;
  int lower_result = -1, upper_result = -1;
  bool position_bounds_valid = false;  // Diagnostic only; URDF owns host bounds.
  double lower = 0, upper = 0, max_velocity = 0, max_acceleration = 0;
  double max_deceleration = 0, stop_deceleration = 0, torque_constant = 0;
};
// Commit no partial snapshot on a read/validation error. All values use ROS units.
bool read_drive_parameters(ParameterReader &, DriveParameters &, std::string &error);
bool constrain_axis(AxisConfig &, const DriveParameters &, const Config &, std::string &error);
bool axis_feedback_fresh(const AxisFeedback &, const Config &, int64_t now) noexcept;
bool axis_feedback_after(const AxisFeedback &, const Config &, int64_t since) noexcept;

class DriveFaultRecovery {
public:
  virtual ~DriveFaultRecovery() = default;
  virtual bool wait_for_stationary_drives(bool allow_faults) = 0;
  virtual int read_drive_fault(std::size_t axis, uint16_t &code) = 0;
  virtual int clear_drive_fault(std::size_t axis) = 0;
};
bool drive_allows_recovery(uint16_t statusword, bool allow_faults) noexcept;
bool recover_drive_faults(DriveFaultRecovery &, const Config &, std::ostream &);

bool advance_axis(const AxisConfig &, double target, double dt, double &position, double &velocity) noexcept;

// One producer and one consumer exchange ownership of three preallocated slots.
template<class T>
class LatestSample {
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::atomic<unsigned>::is_always_lock_free);
public:
  void publish(const T &value) noexcept {
    slots_[write_] = value;
    write_ = exchange_.exchange(write_ | updated, std::memory_order_acq_rel) & index;
  }
  bool take(T &value) noexcept {
    if (!(exchange_.load(std::memory_order_acquire) & updated)) return false;
    read_ = exchange_.exchange(read_, std::memory_order_acq_rel) & index;
    value = slots_[read_];
    return true;
  }
  void reset() noexcept {  // Only while both participants are stopped.
    read_ = 0;
    write_ = 2;
    exchange_.store(1, std::memory_order_relaxed);
  }
private:
  static constexpr unsigned updated = 4, index = 3;
  std::array<T,3> slots_{};
  unsigned read_ = 0, write_ = 2;
  std::atomic<unsigned> exchange_{1};
};

// Shared by the real SDK adapter and recording tests. These operations only
// report transmit/SDO results, never an acknowledgement of physical standstill.
class StopActions {
public:
  virtual ~StopActions() = default;
  virtual bool quick_stop(std::size_t axis) noexcept = 0;
  virtual bool hold(std::size_t axis, double position) noexcept = 0;
  virtual bool stop_gripper(std::size_t gripper) noexcept = 0;
  virtual bool disable(std::size_t axis) noexcept = 0;
};
bool request_quick_stop(StopActions &, const Positions &) noexcept;
bool request_hold(StopActions &, const Positions &) noexcept;
bool disable_all(StopActions &) noexcept;
enum class SyncPhase { configure, control, activate, deactivate, fault_stop };
const char *sync_phase_name(SyncPhase phase) noexcept;

struct BusCycleResult {
  bool sync_ok = true, tools_ok = true;
};

template<class SendSync, class StepTools>
BusCycleResult step_can_buses(std::size_t count, SendSync send_sync, StepTools step_tools) noexcept {
  BusCycleResult result;
  for (std::size_t bus=0;bus<count;++bus) {
    if (send_sync(bus)) result.tools_ok = step_tools(bus) && result.tools_ok;
    else result.sync_ok = false;
  }
  return result;
}

// Submit complete arm groups after feedback processing, like the system HAL.
template<class ReadyForBus, class SendAxis>
bool send_position_pdos_by_bus(const std::array<AxisConfig,kArms> &axes,
                              ReadyForBus ready_for_bus, SendAxis send_axis) noexcept {
  for (std::size_t bus=0;bus<2;++bus) {
    if (!ready_for_bus(bus)) return false;
    for (std::size_t i=0;i<axes.size();++i) {
      if (axes[i].bus == static_cast<int>(bus) && !send_axis(i)) return false;
    }
  }
  return true;
}

struct SyncInterval {
  int64_t ns = 0;
  SyncPhase from = SyncPhase::configure, to = SyncPhase::configure;
};

struct SyncBusTiming {
  uint64_t intervals = 0, failures = 0;
  SyncInterval shortest, longest;
  std::array<SyncInterval,8> recent{};
  int64_t max_write_ns = 0;
};

struct SyncBusSkew {
  uint64_t samples = 0;
  int64_t shortest_ns = 0, longest_ns = 0;
};

// One thread owns the fixed schedule across all phases. A full-cycle overrun
// starts a fresh period from completion, matching the system control loop.
class SyncCadence {
public:
  explicit SyncCadence(int period_us) : period_ns_(int64_t(period_us)*1000) {}
  int64_t next_send_ns() const noexcept { return next_send_ns_; }
  bool due(int64_t now_ns) const noexcept { return now_ns >= next_send_ns_; }
  void begin_batch(int64_t now_ns) noexcept;
  void finish_batch(int64_t now_ns) noexcept;
  void record(std::size_t bus, int64_t started_ns, int64_t ended_ns,
              bool sent, SyncPhase phase) noexcept;
  const SyncBusTiming &timing(std::size_t bus) const noexcept { return buses_[bus].timing; }
  const SyncBusSkew &bus_skew() const noexcept { return bus_skew_; }
  uint64_t reschedules() const noexcept { return reschedules_; }
  int64_t max_lateness_ns() const noexcept { return max_lateness_ns_; }
private:
  struct Bus {
    SyncBusTiming timing;
    int64_t last_sent_ns = 0;
    SyncPhase last_phase = SyncPhase::configure;
  };
  int64_t period_ns_, next_send_ns_ = 0, max_lateness_ns_ = 0;
  uint64_t reschedules_ = 0;
  std::array<Bus,2> buses_{};
  int64_t first_bus_start_ns_ = 0;
  SyncBusSkew bus_skew_{};
};
struct SyncWrite {
  int64_t started_ns = 0, returned_ns = 0;
  int result = 0;
  uint8_t counter = 0;
  SyncPhase phase = SyncPhase::configure;
  int64_t write_entered_ns = 0, write_returned_ns = 0;
  int write_bytes = 0, write_errno = 0;
  int64_t timestamp_id = -1;
  bool counter_valid = true;
};

struct SyncTxTimestamps {
  int64_t scheduled_ns = 0, software_ns = 0, hardware_ns = 0;
};

class SyncTxHistory {
public:
  void record(uint32_t id, int64_t SyncTxTimestamps::*kind, int64_t ns) noexcept {
    auto &entry = entries_[id % entries_.size()];
    if (entry.id != id) {
      if (entry.id >= 0 && id-static_cast<uint32_t>(entry.id) > UINT32_MAX/2) return;
      entry = {id,{}};
    }
    if (entry.times.*kind == 0) entry.times.*kind = ns;
  }
  SyncTxTimestamps get(int64_t id) const noexcept {
    if (id < 0) return {};
    const auto &entry = entries_[static_cast<uint32_t>(id) % entries_.size()];
    return entry.id == id ? entry.times : SyncTxTimestamps{};
  }
private:
  struct Entry {
    int64_t id = -1;
    SyncTxTimestamps times;
  };
  // More room than the fault trace so a reader can catch up after a delay.
  std::array<Entry,1024> entries_{};
};
class SyncFaultTrace {
public:
  static constexpr std::size_t capacity = 512;
  void record(std::size_t bus, SyncWrite event) noexcept {
    if (!frozen_) {
      auto &history = buses_[bus];
      history.entries[history.total++ % capacity] = event;
      if (event.timestamp_id >= 0) history.last_timestamp_id = event.timestamp_id;
    }
  }
  void freeze() noexcept { frozen_ = true; }
  bool includes_timestamp(std::size_t bus, uint32_t id) const noexcept {
    const auto latest = buses_[bus].last_timestamp_id;
    return latest >= 0 && static_cast<uint32_t>(latest)-id < capacity;
  }
  std::size_t size(std::size_t bus) const noexcept {
    return buses_[bus].total < capacity ? buses_[bus].total : capacity;
  }
  const SyncWrite &at(std::size_t bus, std::size_t offset) const noexcept {
    const auto &history = buses_[bus];
    return history.entries[(history.total-size(bus)+offset) % capacity];
  }
private:
  struct Bus {
    std::array<SyncWrite,capacity> entries{};
    std::size_t total = 0;
    int64_t last_timestamp_id = -1;
  };
  std::array<Bus,2> buses_{};
  bool frozen_ = false;
};
class StandstillMonitor {
public:
  explicit StandstillMonitor(double tolerance) : tolerance_(tolerance) {}
  bool update(const std::array<AxisFeedback,kArms> &) noexcept;
private:
  double tolerance_;
  int consecutive_ = 0;
};
// Public-SDK observations are spaced farther apart than the complete feedback
// age budget, so reading the same cached PDO cannot count as three fresh sets.
class FreshStandstillMonitor {
public:
  FreshStandstillMonitor(const Config &config, int64_t after)
    : config_(config), after_(after), monitor_(config.stop_velocity_tolerance) {}
  bool update(const std::array<AxisFeedback,kArms> &, int64_t now) noexcept;
  void reset() noexcept {
    previous_.fill(0);
    monitor_=StandstillMonitor(config_.stop_velocity_tolerance);
  }
private:
  const Config &config_;
  int64_t after_;
  std::array<int64_t,kArms> previous_{};
  StandstillMonitor monitor_;
};
}  // namespace coarm7_hardware
