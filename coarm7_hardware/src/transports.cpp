#include "coarm7_hardware/core.hpp"
#include "coarm7_hardware/gripper.hpp"
#include "coarm7_hardware/device.hpp"
#include "modi_bus_canevo.h"
#include "modi_joint_canevo.h"
#include "modi_tool_io_canevo.h"
#if !COARM7_SDK_EXTENSIONS
#include "coarm7_hardware/sdk_public.hpp"
#endif
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <poll.h>
#include <mutex>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <thread>
#include <time.h>
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace coarm7_hardware {
bool sdk_extensions_available() noexcept { return COARM7_SDK_EXTENSIONS != 0; }
namespace {
struct FaultObservation {
  int64_t received_ns = 0;
  uint8_t node_id = 0;
  uint16_t code = 0;
};
struct Targets {
  Positions positions{};
  std::array<double,kGrippers> grippers{};
  int64_t updated_ns = 0;
  std::array<bool,kGrippers> gripper_claims{};
  std::array<uint64_t,kGrippers> goal_ids{}, stop_ids{};
};
#if COARM7_SDK_EXTENSIONS
class SyncEchoTrace {
public:
  void record(const CanEvoSyncEcho &echo) noexcept {
    echoes_[total_++ % echoes_.size()] = echo;
  }
  std::size_t size() const noexcept { return std::min(total_,echoes_.size()); }
  const CanEvoSyncEcho &at(std::size_t i) const noexcept {
    return echoes_[(total_-size()+i) % echoes_.size()];
  }
private:
  std::array<CanEvoSyncEcho,SyncFaultTrace::capacity> echoes_{};
  std::size_t total_ = 0;
};

class CheckedReader final : public ParameterReader {
public:
  explicit CheckedReader(modi_joint_canevo &joint) : joint_(joint) {}
  int u16(uint8_t i,uint8_t s,uint16_t &v) override { return joint_.NrtReadObjectU16(i,s,v); }
  int f32(uint8_t i,uint8_t s,float &v) override { return joint_.NrtReadObjectF32(i,s,v); }
private:
  modi_joint_canevo &joint_;
};
#else
using CheckedReader=PublicParameterReader<modi_joint_canevo>;
#endif
class SdkTransport final : public Transport, private StopActions, private DriveFaultRecovery {
public:
  ~SdkTransport() override { close(); }
  bool open(const Config &c) override { return open_session(c,false); }
  bool clear_faults(const Config &c) {
#if !COARM7_SDK_EXTENSIONS
    (void)c;
    std::cerr << "coarm7_hardware: checked FF05-only recovery is unavailable with the public SDK; "
                 "its value-only fault getter cannot verify a zero result. No CAN bus was opened.\n";
    return false;
#else
    return open_session(c,true) && recover_drive_faults(*this,c,std::cerr);
#endif
  }

private:
  bool open_session(const Config &c, bool recovery_only) {
    c.validate();
#if !COARM7_SDK_EXTENSIONS
    if (recovery_only) return false;
#endif
    config_ = c;
    recovery_only_ = recovery_only;
    phase_.store(SyncPhase::configure);
    stop_requested_.store(false);
    stop_sent_ns_.store(0);
    io_failed_.store(false);
    fault_latched_.store(false);
    completed_phase_.store(SyncPhase::configure);
    start_result_.store(-1);
    targets_.reset();
    feedback_.reset();
    sync_cadence_ = SyncCadence(c.period_us);
    trace_ = {};
    tx_history_ = {};
    tx_counts_ = {};
#if COARM7_SDK_EXTENSIONS
    echo_trace_ = {};
#endif
    echo_counts_ = {};
    echo_hardware_counts_ = {};
    sequences_.fill(0);
    parameters_ = {};
    startup_snapshot_ = {};
    last_feedback_ = {};
    target_ = {};
    request_goals_.fill(0); request_stops_.fill(0); handled_goals_.fill(0); handled_stops_.fill(0);
    output_ = {};
    output_velocity_ = {};
    stop_positions_ = {};
    stop_position_set_ = false;
    tool_stopped_.fill(false);
    tool_claimed_.fill(false);
    tool_tx_counts_ = {};
    tool_tx_failures_ = {};
    cycle_phase_ = SyncPhase::configure;
    fault_reason_ = nullptr;
    fault_ns_ = 0;
    fault_reported_ = false;
    observed_emcy_ = {};
    observed_emcy_bus_ = 0;
    TaskConfig task; task.sync_period_us=c.period_us; task.cpu_affinity=c.rx_cpu;
    task.sched_priority=c.rx_priority;
    for (std::size_t b=0;b<2;++b) {
      buses_[b]=std::make_unique<modi_bus_canevo>();
      const int result = buses_[b]->Open(c.buses[b],task);
      if (result != 0) {
        std::cerr << "coarm7_hardware: cannot open bus=" << c.buses[b]
                  << " sdk_result=" << result << '\n';
        return false;
      }
      buses_[b]->SetPdoTimeoutMs(sdk_extensions_available() ? c.joint_timeout_ms : c.sdk_cache_timeout_ms());
      buses_[b]->SetSdoTimeoutMs(c.sdo_timeout_ms);
    }
    for (std::size_t i=0;i<kArms;++i) {
      joints_[i]=std::make_unique<modi_joint_canevo>();
      const auto &axis = c.axes[i];
      auto &bus = *buses_[axis.bus];
#if COARM7_SDK_EXTENSIONS
      const int result = recovery_only_ ? joints_[i]->NrtAttach(bus,axis.id) : joints_[i]->NrtInit(bus,axis.id);
#else
      const int result = joints_[i]->NrtInit(bus,axis.id);
#endif
      if (result != 0) {
        std::cerr << "coarm7_hardware: cannot initialize joint=" << axis.name
                  << " bus=" << c.buses[axis.bus] << " node_id=" << axis.id
                  << " sdk_result=" << result << '\n';
        return false;
      }
      if (recovery_only_) continue;
      CheckedReader reader(*joints_[i]); std::string error;
      if(!read_drive_parameters(reader,parameters_[i],error) ||
         !constrain_axis(config_.axes[i],parameters_[i],c,error)) {
        std::cerr << "coarm7_hardware: " << c.axes[i].name << ": " << error << '\n';
        return false;
      }
    }
    // In recovery, read all persistent fault codes before starting SYNC.
    if (recovery_only_) return true;
    if (!open_tools(c)) return false;
    const auto deadline = monotonic_now_ns()+c.startup_timeout_ms*1000000LL;
#if COARM7_SDK_EXTENSIONS
    if (c.tx_timestamp_diagnostics) start_timestamp_reader();
#endif
    return start_cycle(deadline) && wait_for_startup_feedback(deadline);
  }

  bool open_tools(const Config &c) {
    write_parameter_report();
    for (std::size_t i=0;i<kGrippers;++i) {
      const auto &gripper = c.grippers[i];
      tools_[i]=std::make_unique<modi_tool_io_canevo>();
      const int result = tools_[i]->NrtInit(*buses_[gripper.bus],gripper.tool_id);
      if (result != 0) {
        std::cerr << "coarm7_hardware: gripper tool initialization failed gripper=" << gripper.name
                  << " bus=" << c.buses[gripper.bus] << " tool_id=" << gripper.tool_id
                  << " device_id=" << gripper.device_id << " sdk_result=" << result << '\n';
        return false;
      }
      grippers_[i]=std::make_unique<GripperProtocol>(c,i);
    }
    return true;
  }

  bool start_cycle(int64_t deadline) {
    rlimit memory_limit{};
    const bool memory_limit_known = ::getrlimit(RLIMIT_MEMLOCK,&memory_limit) == 0;
    int memory_lock_error = 0;
    if (config_.tx_lock_memory && ::mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
      memory_lock_error = errno;
      std::cerr << "coarm7_hardware: memory locking failed: "
                << std::strerror(memory_lock_error) << "; CAN cycle memory may fault in at runtime\n";
    }
    running_.store(true);
    worker_ = std::thread([this] { run_cycle(); });
    while (start_result_.load() < 0 && monotonic_now_ns() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const int rt_result = start_result_.load();
    if (rt_result != 0) {
      std::cerr << "coarm7_hardware: CAN cycle thread configuration failed: "
                << (rt_result < 0 ? "startup timed out" : std::strerror(rt_result)) << '\n';
      return false;
    }
    std::cerr << "coarm7_hardware: CAN cycle running, period_us=" << config_.period_us
              << " FIFO=" << config_.tx_priority << " cpu=" << config_.tx_cpu
              << " wait=clock_nanosleep_absolute cadence=fixed_grid"
              << " communication_profile=system_order_v1"
              << " position_pdo_order=bus0_then_bus1 position_pdo_release=after_feedback"
              << " memory_lock=" << (!config_.tx_lock_memory ? "disabled" :
                                     memory_lock_error ? "failed" : "locked")
              << " memlock_limit_bytes=" << (memory_limit_known ? memory_limit.rlim_cur : 0)
              << " sdk_api=" << (sdk_extensions_available() ? "development_extensions" : "public")
              << " feedback_clock=" << (sdk_extensions_available() ? "pdo_receive" : "sdk_validity_check")
              << " sdk_cache_timeout_ms=" << (sdk_extensions_available() ? config_.joint_timeout_ms : config_.sdk_cache_timeout_ms())
              << " snapshot_timeout_ms=" << config_.feedback_snapshot_timeout_ms
              << " trace_diagnostics=" << config_.tx_trace_diagnostics
              << " timestamp_diagnostics=" << (config_.tx_timestamp_diagnostics ? "true" : "false") << '\n';
    return true;
  }

  bool wait_for_startup_feedback(int64_t deadline) {
    Snapshot s;
    do {
      if (!sample(monotonic_now_ns(),s)) {
        if (io_failed_.load()) return false;
      } else if (std::all_of(s.axes.begin(),s.axes.end(),[](const auto &axis) { return axis.valid; }) &&
                 std::all_of(s.grippers.begin(),s.grippers.end(),[](const auto &tool) { return tool.valid; })) {
        startup_snapshot_ = s;
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (monotonic_now_ns()<deadline);
    for (std::size_t i=0;i<kArms;++i) {
      if (s.axes[i].valid) continue;
      const auto &a=config_.axes[i];
      std::cerr << "coarm7_hardware: startup PDO unavailable joint=" << a.name
                << " bus=" << config_.buses[a.bus] << " node_id=" << a.id
                << " pdo_result=" << s.axes[i].pdo_result << '\n';
    }
    for (std::size_t i=0;i<kGrippers;++i) {
      if (s.grippers[i].valid) continue;
      const auto &g=config_.grippers[i];
      std::cerr << "coarm7_hardware: startup gripper feedback unavailable gripper=" << g.name
                << " bus=" << config_.buses[g.bus] << " tool_id=" << g.tool_id
                << " device_id=" << g.device_id << " timeout_ms=" << config_.startup_timeout_ms << '\n';
    }
    return false;
  }
public:
  void apply_runtime_limits(Config &c) const override { c.axes=config_.axes; }
  bool startup_feedback(Snapshot &s) const override { s=startup_snapshot_; return true; }
  void close() noexcept override {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    timestamp_reader_running_.store(false);
    if (timestamp_reader_.joinable()) {
      timestamp_reader_.join();
#if COARM7_SDK_EXTENSIONS
      report_timestamp_counts();
#endif
    }
    report_fault();
    report_sync_timing();
    report_grippers();
    sync_cadence_ = SyncCadence(config_.period_us);
    for (auto &t:tools_) { if(t) t->NrtDestroy(); t.reset(); }
    for (auto &j:joints_) { if(j) j->NrtDestroy(); j.reset(); }
    for (auto &b:buses_) { if(b) b->Close(); b.reset(); }
    for (auto &g:grippers_) g.reset();
  }
  bool sample(int64_t, Snapshot &s) override {
    feedback_.take(last_feedback_);
    s = last_feedback_;
    return s.sampled_ns > 0 && !io_failed_.load();
  }
  bool limits_axis_steps() const noexcept override { return true; }
  bool activate(const Positions &hold) override {
    if (stop_requested_.load()) return false;
    targets_.publish({hold,{},monotonic_now_ns()});
    phase_.store(SyncPhase::activate);
    const auto primed = [this] { return completed_phase_.load() == SyncPhase::activate || stop_requested_.load(); };
    if (!wait_until(primed,config_.startup_timeout_ms) || stop_requested_.load()) return false;

    // SDK SDO transactions run on the lifecycle caller while the cycle
    // worker continues to send held positions and SYNC at its own cadence.
    for (auto &joint:joints_) {
      if (stop_requested_.load() || joint->NrtEnable(CanEvoMode::kCsp) != 0) return false;
    }
    const auto enabled_ns = monotonic_now_ns();
    const auto ready = [this,enabled_ns] {
      Snapshot s;
      return stop_requested_.load() || (sample(monotonic_now_ns(),s) &&
        std::all_of(s.axes.begin(),s.axes.end(),[this,enabled_ns](const auto &axis) {
          return axis_feedback_fresh(axis,config_,monotonic_now_ns()) &&
            axis_feedback_after(axis,config_,enabled_ns) && axis.running_csp && !axis.fault;
        }));
    };
    if (!wait_until(ready,config_.startup_timeout_ms) || stop_requested_.load()) return false;
    targets_.publish({hold,{},monotonic_now_ns()});
    phase_.store(SyncPhase::control);
    return true;
  }
  void gripper_requests(const std::array<uint64_t,2> &g,const std::array<uint64_t,2> &s) override { request_goals_=g; request_stops_=s; }
  bool send(const Positions &p,const std::array<double,kGrippers> &raw,
            const std::array<bool,kGrippers> &gripper_claims) override {
    if (stop_requested_.load() || phase_.load() != SyncPhase::control) return false;
    targets_.publish({p,raw,monotonic_now_ns(),gripper_claims,request_goals_,request_stops_});
    return true;
  }
  bool stop(const Positions &) noexcept override {
    stop_requested_.store(true);
    return stop_sent_ns_.load() != 0;
  }
  bool deactivate(const Positions &hold) override {
    if (phase_.load() == SyncPhase::configure) return true;
    targets_.publish({hold,{},monotonic_now_ns()});
    phase_.store(SyncPhase::deactivate);
    // The worker closes the motion gate and sends the held/stop PDOs before
    // the lifecycle thread starts blocking disable transactions.
    const auto deactivating = [this] { return completed_phase_.load() == SyncPhase::deactivate || stop_requested_.load(); };
    if (!wait_until(deactivating,config_.startup_timeout_ms)) {
      stop_requested_.store(true);
      return false;
    }
    if (stop_requested_.load() && !wait_for_stop()) {
      report_fault();
      std::cerr << "coarm7_hardware: fault stop has no fresh standstill feedback; "
                   "disable not issued, physical stop unknown\n";
      return false;
    }
    const bool disabled = disable_all(*this);
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.disable_settle_ms));
    if (disabled) phase_.store(SyncPhase::configure);
    else stop_requested_.store(true);
    report_fault();
    return disabled;
  }
private:
  template<class Ready>
  bool wait_until(Ready ready, int timeout_ms) const {
    const auto deadline = monotonic_now_ns()+timeout_ms*1000000LL;
    do {
      if (ready()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (monotonic_now_ns()<deadline);
    return false;
  }
  int configure_thread() const noexcept {
    pthread_setname_np(pthread_self(),"coarm7-can");
    if (config_.tx_cpu >= CPU_SETSIZE) return EINVAL;
    if (config_.tx_cpu >= 0) {
      cpu_set_t affinity;
      CPU_ZERO(&affinity);
      CPU_SET(config_.tx_cpu,&affinity);
      const int rc = pthread_setaffinity_np(pthread_self(),sizeof(affinity),&affinity);
      if (rc != 0) return rc;
    }
    sched_param parameters{};
    parameters.sched_priority = config_.tx_priority;
    int rc = pthread_setschedparam(pthread_self(),SCHED_FIFO,&parameters);
    if (rc != 0) return rc;
    int policy = 0;
    rc = pthread_getschedparam(pthread_self(),&policy,&parameters);
    if (rc != 0) return rc;
    if (policy != SCHED_FIFO || parameters.sched_priority != config_.tx_priority) return EINVAL;
    if (config_.tx_cpu >= 0) {
      cpu_set_t affinity;
      CPU_ZERO(&affinity);
      rc = pthread_getaffinity_np(pthread_self(),sizeof(affinity),&affinity);
      if (rc != 0) return rc;
      if (CPU_COUNT(&affinity) != 1 || !CPU_ISSET(config_.tx_cpu,&affinity)) return EINVAL;
    }
    return 0;
  }
  void run_cycle() noexcept {
    const int result = configure_thread();
    start_result_.store(result);
    if (result != 0) return;
    while (running_.load()) {
      const auto due_ns = sync_cadence_.next_send_ns();
      const timespec due{static_cast<time_t>(due_ns/1000000000LL),
                         static_cast<long>(due_ns%1000000000LL)};
      int wait_result;
      do {
        wait_result = ::clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&due,nullptr);
      } while (wait_result == EINTR && running_.load());
      if (!running_.load()) break;
      if (wait_result != 0) {
        latch_fault("absolute SYNC wait failed",true);
        break;
      }
      tick();
      sync_cadence_.finish_batch(monotonic_now_ns());
    }
  }
  bool send_sync(std::size_t bus, SyncPhase phase) noexcept {
    SyncWrite event;
    event.phase = phase;
#if COARM7_SDK_EXTENSIONS
    CanEvoSyncWriteTiming write;
#endif
    event.started_ns = monotonic_now_ns();
#if COARM7_SDK_EXTENSIONS
    event.result = buses_[bus]->RtStepOnce(event.counter,write);
#else
    event.result = buses_[bus]->RtStepOnce();
    event.counter_valid = false;
#endif
    event.returned_ns = monotonic_now_ns();
#if COARM7_SDK_EXTENSIONS
    event.write_entered_ns = write.entered_ns;
    event.write_returned_ns = write.returned_ns;
    event.write_bytes = write.bytes;
    event.write_errno = write.error_number;
    event.timestamp_id = write.timestamp_id;
#endif
    const bool sent = event.result == 0;
    sync_cadence_.record(bus,event.started_ns,event.returned_ns,sent,phase);
    if (config_.tx_trace_diagnostics) trace_.record(bus,event);
    return sent;
  }
#if COARM7_SDK_EXTENSIONS
  void start_timestamp_reader() {
    std::array<pollfd,2> sockets{};
    for (std::size_t i=0;i<buses_.size();++i) {
      const auto setup = buses_[i]->NrtEnableSyncTimestamps();
      sockets[i].fd = setup.poll_fd;
      sockets[i].events = setup.sync_echo ? POLLIN : 0;
      std::fprintf(stderr,"coarm7_hardware: SYNC TX timestamp setup bus=%s socket_errno=%d "
                   "hwstamp_get_errno=%d hw_tx_mode=%d sync_echo=%d echo_errno=%d\n",
                   config_.buses[i].c_str(),setup.socket_error,setup.interface_error,
                   setup.hardware_tx_mode,setup.sync_echo,setup.echo_error);
    }
    if (std::all_of(sockets.begin(),sockets.end(),[](const auto &socket) { return socket.fd < 0; })) return;
    timestamp_reader_running_.store(true);
    timestamp_reader_ = std::thread([this,sockets]() mutable {
      const sched_param parameters{};
      (void)pthread_setschedparam(pthread_self(),SCHED_OTHER,&parameters);
      while (timestamp_reader_running_.load()) {
        const int ready = ::poll(sockets.data(),sockets.size(),20);
        if (ready < 0) {
          if (errno == EINTR) continue;
          std::fprintf(stderr,"coarm7_hardware: SYNC TX timestamp poll failed errno=%d\n",errno);
          break;
        }
        bool consumed = false;
        for (std::size_t i=0;i<sockets.size();++i) {
          if (sockets[i].revents & POLLERR) consumed = drain_timestamps(i) || consumed;
          if (sockets[i].revents & POLLIN) consumed = drain_sync_echoes(i) || consumed;
        }
        if (ready && !consumed) std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      for (std::size_t i=0;i<sockets.size();++i) {
        if (sockets[i].fd < 0) continue;
        drain_timestamps(i);
        if (sockets[i].events & POLLIN) drain_sync_echoes(i);
      }
    });
  }

  bool drain_timestamps(std::size_t bus) {
    bool consumed = false;
    // Bounded per wake so one channel cannot starve the other or delay join.
    for (int i=0;i<64;++i) {
      CanEvoSyncTimestamp stamp;
      const int result = buses_[bus]->NrtReadSyncTimestamp(stamp);
      if (result == -EINTR) continue;
      if (result < 0) return consumed;
      consumed = true;
      if (result == 0) continue;
      const auto kind = static_cast<std::size_t>(stamp.kind);
      constexpr std::array fields{&SyncTxTimestamps::scheduled_ns,&SyncTxTimestamps::software_ns,
                                  &SyncTxTimestamps::hardware_ns};
      constexpr std::array labels{"scheduled","software","hardware"};
      if (++tx_counts_[bus][kind] == 1) {
        std::fprintf(stderr,"coarm7_hardware: first SYNC TX timestamp bus=%s source=%s id=%u ns=%lld\n",
                     config_.buses[bus].c_str(),labels[kind],stamp.id,static_cast<long long>(stamp.ns));
      }
      // The RT writer publishes an immutable trace before setting fault_latched_.
      if (!fault_latched_.load(std::memory_order_acquire) || trace_.includes_timestamp(bus,stamp.id)) {
        std::lock_guard<std::mutex> lock(tx_history_mutex_);
        tx_history_[bus].record(stamp.id,fields[kind],stamp.ns);
      }
    }
    return consumed;
  }

  bool drain_sync_echoes(std::size_t bus) {
    bool consumed = false;
    for (int i=0;i<64;++i) {
      CanEvoSyncEcho echo;
      const int result = buses_[bus]->NrtReadSyncEcho(echo);
      if (result == -EINTR) continue;
      if (result < 0) return consumed;
      consumed = true;
      if (result == 0) continue;
      if (++echo_counts_[bus] == 1) {
        std::fprintf(stderr,"coarm7_hardware: first SYNC echo bus=%s counter=%u hw_ns=%lld\n",
                     config_.buses[bus].c_str(),unsigned(echo.counter),static_cast<long long>(echo.hardware_ns));
      }
      if (echo.hardware_ns && ++echo_hardware_counts_[bus] == 1) {
        std::fprintf(stderr,"coarm7_hardware: first SYNC TX timestamp bus=%s source=hardware_echo "
                     "counter=%u ns=%lld\n",config_.buses[bus].c_str(),unsigned(echo.counter),
                     static_cast<long long>(echo.hardware_ns));
      }
      // Keep the pre-fault window and allow in-flight completions to arrive.
      if (!fault_latched_.load(std::memory_order_acquire) ||
          echo.received_ns <= fault_ns_ + 2*config_.period_us*1000LL) {
        std::lock_guard<std::mutex> lock(tx_history_mutex_);
        echo_trace_[bus].record(echo);
      }
    }
    return consumed;
  }

#endif
  FaultObservation first_emcy(std::size_t &bus) const noexcept {
#if COARM7_SDK_EXTENSIONS
    FaultObservation first;
    for (std::size_t i=0;i<buses_.size();++i) {
      CanEvoFirstEmcy next;
      if (buses_[i] && buses_[i]->RtGetFirstEmcy(next) &&
          (first.code == 0 || next.received_ns < first.received_ns)) {
        first = {next.received_ns,next.node_id,next.code};
        bus = i;
      }
    }
    return first;
#else
    bus=observed_emcy_bus_;
    return observed_emcy_;
#endif
  }
  void observe_emcy(std::size_t i,CanEvoFault fault) noexcept {
    // Freeze this non-atomic snapshot before publishing fault_latched_. The
    // lifecycle reporter may read it while the cycle continues its stop stream.
    if (!fault_latched_.load(std::memory_order_relaxed) &&
        !observed_emcy_.code && fault != CanEvoFault::kNone) {
      observed_emcy_={monotonic_now_ns(),uint8_t(config_.axes[i].id),uint16_t(fault)};
      observed_emcy_bus_=config_.axes[i].bus;
    }
  }
  void latch_fault(const char *reason, bool transport_failure = false) noexcept {
    if (!fault_latched_.load()) {
      trace_.freeze();
      fault_reason_ = reason;
      fault_ns_ = monotonic_now_ns();
      fault_latched_.store(true,std::memory_order_release);
    }
    if (transport_failure) io_failed_.store(true);
    stop_requested_.store(true);
  }
  void read_axes(Snapshot &snapshot) noexcept {
    for (std::size_t i=0;i<kArms;++i) {
      JointStatus status{};
      auto &axis = snapshot.axes[i];
#if COARM7_SDK_EXTENSIONS
      axis.pdo_result = joints_[i]->RtGetJointStatus(status,axis.received_ns);
#else
      axis.sdk_checked = true;
      axis.checked_ns = monotonic_now_ns();
      axis.pdo_result = joints_[i]->RtGetJointStatus(status);
#endif
      axis.valid = axis.pdo_result == 0;
      CanEvoFault fault{};
      axis.emcy_valid = joints_[i]->RtGetEmcy(fault);
      axis.emcy = static_cast<uint16_t>(fault);
      if (axis.emcy_valid && fault != CanEvoFault::kNone) observe_emcy(i,fault);
      axis.fault = axis.emcy_valid && axis.emcy != 0;
      if (!axis.valid) continue;
      axis.position = status.actual_pos;
      axis.velocity = status.actual_vel;
      const auto mode = static_cast<unsigned>((status.statusword>>12)&0xF);
      const auto state = status.statusword&0xF;
      axis.running_csp = mode == 1 && state == 3;
      axis.statusword = status.statusword;
      axis.fault = axis.fault || state == 6 || state == 7 || state == 8;
    }
  }
  bool axes_ready_for_motion(const Snapshot &snapshot) const noexcept {
    for (std::size_t i=0;i<kArms;++i) {
      const auto &axis = snapshot.axes[i];
      const auto &config = config_.axes[i];
      const double position = config.sign*axis.position+config.offset;
      if (!axis_feedback_fresh(axis,config_,monotonic_now_ns()) || !axis.running_csp || !std::isfinite(position) ||
          !std::isfinite(axis.velocity) || position < config.lower-1e-5 ||
          position > config.upper+1e-5 ||
          std::abs(position-snapshot.applied_position[i]) > config.following_error) return false;
    }
    return true;
  }
  bool send_tool_frame(std::size_t i,const SerialFrame &frame) noexcept {
    if (!frame.size) return true;
    const int result=tools_[i]->RtSendToolRs485(frame.data.data(),frame.size);
    if (result != 0) { ++tool_tx_failures_[i]; return false; }
    const int index=frame.data[4]==0x41 ? 0 : (frame.data[4]==0x54 ? 1 :
      (frame.data[4]==0x16 ? 2 : (frame.data[4]==0x51 ? 3 : 4)));
    ++tool_tx_counts_[i][index];
    return true;
  }
  void report_grippers() const {
    // Called only after joining the worker. No stream logging in the RT path.
    for (std::size_t i=0;i<kGrippers;++i) {
      if (!grippers_[i]) continue;
      const auto &d=grippers_[i]->diagnostics();
      const auto &f=grippers_[i]->feedback();
      std::cerr << "coarm7_hardware: gripper summary joint=" << config_.grippers[i].name
                << " bus=" << config_.buses[config_.grippers[i].bus]
                << " read_only=" << config_.gripper_read_only
                << " sdk_queries_sent=" << tool_tx_counts_[i][0]
                << " sdk_openings_sent=" << tool_tx_counts_[i][1]
                << " sdk_stops_sent=" << tool_tx_counts_[i][2]
                << " sdk_speed_settings_sent=" << tool_tx_counts_[i][3]
                << " sdk_speed_reads_sent=" << tool_tx_counts_[i][4]
                << " sdk_send_failures=" << tool_tx_failures_[i]
                << " state_replies=" << d.feedback_replies
                << " opening_replies=" << d.opening_replies
                << " query_timeouts=" << d.query_timeouts
                << " opening_timeouts=" << d.opening_timeouts
                << " rejected_replies=" << d.rejected_replies
                << " max_query_reply_us=" << d.max_query_reply_ns/1000.0
                << " max_opening_reply_us=" << d.max_opening_reply_ns/1000.0
                << " last_target_raw=" << d.last_target_raw
                << " last_opening_reply=" << d.last_opening_reply
                << " configured_speed_raw=" << d.configured_speed_raw
                << " readback_speed_raw=" << d.readback_speed_raw
                << " first_opening_raw=" << d.first_opening_raw
                << " last_opening_raw=" << d.last_opening_raw
                << " opening_raw_range=[" << d.min_opening_raw << ',' << d.max_opening_raw << ']'
                << " opening_rejections=" << d.opening_rejections
                << " failed_command=0x" << std::hex << unsigned(d.failed_command) << std::dec
                << " command_result=" << f.command_result
                << " feedback_raw=" << f.raw << " status=" << unsigned(f.status)
                << " fault=" << unsigned(f.fault) << '\n';
    }
  }
  bool service_tools(Snapshot &s, int bus = -1) noexcept {
    const auto now = monotonic_now_ns();
    bool ok = true;
    for (std::size_t i=0;i<kGrippers;++i) {
      if (bus >= 0 && config_.grippers[i].bus != bus) continue;
      uint8_t data[60]{};
      std::size_t size = 0;
      uint32_t seq = 0;
      int64_t stamp = 0;
      if (tools_[i]->RtReceiveToolRs485(data,60,size,&seq,&stamp) == 0 && seq != sequences_[i]) {
        sequences_[i] = seq;
        grippers_[i]->received(data,size,stamp);
      }
      const auto frame = grippers_[i]->next(now,false);
      if (!send_tool_frame(i,frame)) ok = false;
      s.grippers[i] = grippers_[i]->feedback();
      s.grippers[i].stop_ack = handled_stops_[i];
    }
    return ok;
  }
  bool send_tool_commands(const Snapshot &snapshot) noexcept {
    for (std::size_t i=0;i<kGrippers;++i) {
      if (!tool_claimed_[i]) continue;
      const auto now = monotonic_now_ns();
      const auto &feedback = snapshot.grippers[i];
      const auto &config = config_.grippers[i];
      if (!feedback.valid || feedback.fault || feedback.command_result || feedback.received_ns <= 0 ||
          feedback.received_ns > now ||
          now-feedback.received_ns > config_.gripper_timeout_ms*1000000LL ||
          !std::isfinite(feedback.raw) || feedback.raw < config.raw.back()-config.raw_endpoint_margin ||
          feedback.raw > config.raw.front()+config.raw_endpoint_margin) {
        latch_fault("unsafe gripper feedback");
        return false;
      }
      if (!position_pdos_allowed()) return false;
      const auto frame = grippers_[i]->next(now,true);
      if (!send_tool_frame(i,frame)) {
        latch_fault("gripper command transmit failed",true);
        return false;
      }
    }
    return true;
  }
  bool take_pending_emcy() noexcept {
    bool pending = false;
#if COARM7_SDK_EXTENSIONS
    for (auto &bus:buses_) pending = bus->RtTakePendingEmcy() || pending;
#else
    for (std::size_t i=0;i<kArms;++i) {
      CanEvoFault fault{};
      if (joints_[i] && joints_[i]->RtGetEmcy(fault) && fault != CanEvoFault::kNone) {
        observe_emcy(i,fault);
        pending=true;
      }
    }
#endif
    return pending;
  }
  bool position_pdos_allowed() noexcept {
    if (take_pending_emcy()) latch_fault("drive EMCY");
    if (!running_.load() || stop_requested_.load() || phase_.load() != SyncPhase::control)
      return false;
    if (sync_cadence_.due(monotonic_now_ns())) {
      latch_fault("position PDO missed next SYNC slot",true);
      return false;
    }
    return true;
  }
  void send_targets() noexcept {
    const auto max_age_ns = config_.max_cycle_ms*config_.max_consecutive_overruns*1e6;
    const auto command_fresh = [this,max_age_ns] {
      if (monotonic_now_ns()-target_.updated_ns <= max_age_ns) return true;
      latch_fault("ROS command stream expired",true);
      return false;
    };
    if (!command_fresh()) return;

    auto next = output_;
    auto velocity = output_velocity_;
    const double dt = config_.period_us/1e6;
    // Validate both arms before sending any position frame.
    for (std::size_t i=0;i<kArms;++i) {
      if (!advance_axis(config_.axes[i],target_.positions[i],dt,next[i],velocity[i])) {
        latch_fault("bounded position PDO failed",true);
        return;
      }
    }
    const bool sent = send_position_pdos_by_bus(config_.axes,
      [this,&command_fresh](std::size_t) {
        return command_fresh() && position_pdos_allowed();
      },
      [this,&next,&velocity](std::size_t i) {
        if (!hold(i,next[i])) {
          latch_fault("bounded position PDO failed",true);
          return false;
        }
        output_[i] = next[i];
        output_velocity_[i] = velocity[i];
        return true;
      });
    if (!sent) return;
    for (std::size_t i=0;i<kGrippers;++i) {
      const bool claimed = config_.gripper_commands_enabled() && target_.gripper_claims[i];
      if (claimed != tool_claimed_[i]) {
        if (claimed) {
          grippers_[i]->resume();
          tool_stopped_[i] = false;
        } else if (!stop_gripper(i)) {
          latch_fault("gripper controller stop transmit failed",true);
          return;
        }
        tool_claimed_[i] = claimed;
      }
      if (!claimed) {
        handled_goals_[i]=target_.goal_ids[i];
        handled_stops_[i]=target_.stop_ids[i];
        continue;
      }
      // Persistent stop sequences cannot be lost if ROS and SDK cycles overlap.
      // A stop also discards a goal not yet consumed by this worker.
      if (target_.stop_ids[i]!=handled_stops_[i]) {
        if (!stop_gripper(i)) { latch_fault("gripper stop transmit failed",true); return; }
        handled_stops_[i]=target_.stop_ids[i];
        handled_goals_[i]=target_.goal_ids[i];
        continue;
      }
      if (target_.goal_ids[i]!=handled_goals_[i]) {
        if (tool_stopped_[i]) { grippers_[i]->resume(); tool_stopped_[i]=false; }
        if (!grippers_[i]->target(target_.grippers[i])) {
          latch_fault("invalid gripper target",true); return;
        }
        handled_goals_[i]=target_.goal_ids[i];
      }
    }
  }
  void send_stop(const Snapshot &snapshot) noexcept {
    if (stop_sent_ns_.load() == 0 && !stop_position_set_) {
      stop_positions_ = output_;
      for (std::size_t i=0;i<kArms;++i) {
        const auto &axis = snapshot.axes[i];
        const auto &config = config_.axes[i];
        const double q = config.sign*axis.position+config.offset;
        if (axis.valid && q >= config.lower && q <= config.upper) stop_positions_[i] = q;
      }
      stop_position_set_ = true;
    }
    // Keep the stop bit in every PDO until disable, without adding an extra SYNC.
    if (request_quick_stop(*this,stop_positions_) && stop_sent_ns_.load() == 0)
      stop_sent_ns_.store(monotonic_now_ns());
  }
  void tick() noexcept {
    const auto phase = phase_.load();
    const bool stopping = stop_requested_.load();
    const auto sync_phase = stopping ? SyncPhase::fault_stop : phase;
    // Keep the proven per-arm SYNC/query order in both gripper modes.
    // The per-bus callback can only poll; opening writes wait for all guards.
    const bool interleave_tools = !recovery_only_;
    Snapshot snapshot;
    sync_cadence_.begin_batch(monotonic_now_ns());
    const auto bus_result = step_can_buses(buses_.size(),
      [this,sync_phase](std::size_t bus) {
        return send_sync(bus,sync_phase);
      },
      [this,&snapshot,interleave_tools](std::size_t bus) {
        return !interleave_tools || service_tools(snapshot,static_cast<int>(bus));
      });
    if (!bus_result.sync_ok) latch_fault("SYNC transmit failed",true);
    read_axes(snapshot);
    if (recovery_only_) {
      snapshot.sampled_ns = monotonic_now_ns();
      feedback_.publish(snapshot);
      return;
    }
    // SYNC applied the preceding PDO, so pair this feedback with that
    // previously transmitted target rather than the next target below.
    snapshot.applied_valid = cycle_phase_ != SyncPhase::configure;
    snapshot.applied_position = output_;
    if (take_pending_emcy() || std::any_of(snapshot.axes.begin(),snapshot.axes.end(),[](const auto &axis) {
          return axis.emcy_valid && axis.emcy != 0;
        })) {
      latch_fault("drive EMCY");
    } else if (std::any_of(snapshot.axes.begin(),snapshot.axes.end(),[](const auto &axis) { return axis.fault; })) {
      latch_fault("drive fault state");
    } else if (stopping) {
      latch_fault("motion stop requested");
    }
    if (!bus_result.tools_ok) latch_fault("tool query transmit failed",true);

    targets_.take(target_);
    if (phase != cycle_phase_) {
      if (phase == SyncPhase::activate) {
        output_ = target_.positions;
        output_velocity_.fill(0);
        tool_stopped_.fill(false);
        tool_claimed_.fill(false);
        for (auto &joint:joints_) {
          if (joint->RtSetControlMode(CanEvoMode::kCsp) != 0) latch_fault("cannot prepare CSP",true);
        }
      }
      cycle_phase_ = phase;
    }
    if (phase == SyncPhase::control && !stop_requested_.load()) {
      if (!axes_ready_for_motion(snapshot)) latch_fault("unsafe joint feedback");
      else send_targets();
    } else if (phase != SyncPhase::configure && !stop_requested_.load()) {
      bool ok = request_hold(*this,target_.positions);
      if (phase == SyncPhase::deactivate) {
        for (std::size_t i=0;i<kGrippers;++i) if (!stop_gripper(i)) ok = false;
      }
      if (ok) {
        output_ = target_.positions;
        completed_phase_.store(phase);
      } else {
        latch_fault("hold PDO failed",true);
      }
    }
    if (phase == SyncPhase::control && !stop_requested_.load()) send_tool_commands(snapshot);
    if (phase != SyncPhase::configure && stop_requested_.load()) send_stop(snapshot);
    snapshot.sampled_ns = monotonic_now_ns();
    feedback_.publish(snapshot);
  }
  bool wait_for_stationary_drives(bool allow_faults) override {
    // The cycle worker supplies fresh PDOs while the recovery caller handles SDOs.
    const auto deadline = monotonic_now_ns()+config_.startup_timeout_ms*1000000LL;
    if (!worker_.joinable() && !start_cycle(deadline)) return false;
    const auto started_ns = monotonic_now_ns();
    FreshStandstillMonitor monitor(config_,started_ns);
    return wait_until([&] {
      Snapshot snapshot;
      if (!sample(monotonic_now_ns(),snapshot)) return false;
      const auto now = monotonic_now_ns();
      for (std::size_t i=0; i<kArms; ++i) {
        const auto &axis = snapshot.axes[i];
        if (!axis_feedback_fresh(axis,config_,now) ||
            !drive_allows_recovery(axis.statusword,allow_faults) || (!allow_faults && axis.fault)) {
          monitor.reset();
          return false;
        }
      }
      return monitor.update(snapshot.axes,now);
    },config_.startup_timeout_ms);
  }

  int read_drive_fault(std::size_t i, uint16_t &code) override {
#if COARM7_SDK_EXTENSIONS
    return joints_[i]->NrtReadObjectU16(0,0x08,code);
#else
    (void)i; (void)code;
    return int(CanEvoError::kNotSupported);
#endif
  }

  int clear_drive_fault(std::size_t i) override {
#if COARM7_SDK_EXTENSIONS
    // Abort further writes if the periodic transport has already failed.
    if (io_failed_.load()) return static_cast<int>(CanEvoError::kSendFailed);
    auto &joint = *joints_[i];
    uint16_t controlword = 0;
    int result = joint.NrtReadObjectU16(0x21,0,controlword);
    if (result != 0) return result;
    // The SDK clear pulse preserves the enable bit. Remove any leftover
    // enable request from the stopped session before pulsing fault reset.
    constexpr uint16_t enable_bit = 0x0002;
    if (controlword & enable_bit) {
      result = joint.NrtDisable();
      if (result != 0) return result;
      result = joint.NrtReadObjectU16(0x21,0,controlword);
      if (result != 0) return result;
      if (controlword & enable_bit) {
        std::cerr << "Clear faults refused: enable bit remains set joint=" << config_.axes[i].name << '\n';
        return static_cast<int>(CanEvoError::kInvalidParam);
      }
    }
    return joint.NrtClearFault();
#else
    (void)i;
    return int(CanEvoError::kNotSupported);
#endif
  }

  bool wait_for_stop() {
    FreshStandstillMonitor monitor(config_,0);
    return wait_until([this,&monitor] {
      const auto sent = stop_sent_ns_.load();
      if (!sent) return false;
      feedback_.take(last_feedback_);
      const auto &axes = last_feedback_.axes;
      for (std::size_t i=0;i<kArms;++i) {
        if (!axis_feedback_after(axes[i],config_,sent)) { monitor.reset(); return false; }
      }
      return monitor.update(axes,monotonic_now_ns());
    },config_.stop_wait_ms);
  }
  bool quick_stop(std::size_t i) noexcept override { return joints_[i]->RtEstop() == 0; }
  bool hold(std::size_t i,double p) noexcept override {
    return joints_[i]->RtSetCspTargetPosition(device_position(i,p)) == 0;
  }
  bool stop_gripper(std::size_t i) noexcept override {
    if (!tool_stopped_[i]) {
      const auto frame = grippers_[i]->stop();
      tool_stopped_[i] = send_tool_frame(i,frame);
    }
    return tool_stopped_[i];
  }
  bool disable(std::size_t i) noexcept override { return joints_[i]->NrtDisable() == 0; }

  // The cycle thread never formats or writes logs. The frozen trace is immutable.
  void report_fault() noexcept {
    if (fault_reported_ || !fault_latched_.load(std::memory_order_acquire)) return;
    fault_reported_ = true;
    std::size_t bus = 0;
    const auto emcy = first_emcy(bus);
    std::fprintf(stderr,"coarm7_hardware: cycle fault reason=%s host_mono_ns=%lld stop_PDO_submitted=%s\n",
                 fault_reason_,static_cast<long long>(fault_ns_),stop_sent_ns_.load() ? "true" : "false");
    if (emcy.code != 0) {
      const auto axis = std::find_if(config_.axes.begin(),config_.axes.end(),[bus,&emcy](const auto &a) {
        return a.bus == static_cast<int>(bus) && a.id == emcy.node_id;
      });
      std::fprintf(stderr,"coarm7_hardware: first EMCY joint=%s bus=%s node_id=%u code=0x%04X clock=%s mono_ns=%lld\n",
                   axis == config_.axes.end() ? "unknown" : axis->name.c_str(),config_.buses[bus].c_str(),
                   unsigned(emcy.node_id),unsigned(emcy.code),
                   sdk_extensions_available() ? "sdk_receive" : "first_host_observation",
                   static_cast<long long>(emcy.received_ns));
    }
    for (std::size_t i=0;i<buses_.size();++i) {
      std::array<SyncTxTimestamps,SyncFaultTrace::capacity> timestamps{};
      {
        std::lock_guard<std::mutex> lock(tx_history_mutex_);
        for (std::size_t n=0;n<trace_.size(i);++n) timestamps[n]=tx_history_[i].get(trace_.at(i,n).timestamp_id);
      }
      std::array<unsigned,3> matched{};
      for (const auto &stamp:timestamps) {
        matched[0] += stamp.scheduled_ns != 0;
        matched[1] += stamp.software_ns != 0;
        matched[2] += stamp.hardware_ns != 0;
      }
      std::fprintf(stderr,"coarm7_hardware: pre-fault SYNC trace bus=%s samples=%zu "
                   "matched_sched=%u matched_sw=%u matched_hw=%u "
                   "columns=counter,counter_valid,start_mono_ns,return_mono_ns,result,phase,"
                   "write_enter_mono_ns,write_return_mono_ns,write_bytes,write_errno,"
                   "timestamp_id,tx_sched_realtime_ns,tx_sw_realtime_ns,tx_hw_raw_ns (0=missing timestamp)\n",
                   config_.buses[i].c_str(),trace_.size(i),matched[0],matched[1],matched[2]);
      for (std::size_t n=0;n<trace_.size(i);++n) {
        const auto &entry = trace_.at(i,n);
        const auto &stamp = timestamps[n];
        std::fprintf(stderr,"coarm7_hardware: SYNC trace %s %u,%u,%lld,%lld,%d,%s,%lld,%lld,%d,%d,%lld,%lld,%lld,%lld\n",
                     config_.buses[i].c_str(),unsigned(entry.counter),unsigned(entry.counter_valid),static_cast<long long>(entry.started_ns),
                     static_cast<long long>(entry.returned_ns),entry.result,sync_phase_name(entry.phase),
                     static_cast<long long>(entry.write_entered_ns),static_cast<long long>(entry.write_returned_ns),
                     entry.write_bytes,entry.write_errno,static_cast<long long>(entry.timestamp_id),
                     static_cast<long long>(stamp.scheduled_ns),static_cast<long long>(stamp.software_ns),
                     static_cast<long long>(stamp.hardware_ns));
      }
#if COARM7_SDK_EXTENSIONS
      report_sync_echo_trace(i);
#endif
    }
  }

#if COARM7_SDK_EXTENSIONS
  void report_sync_echo_trace(std::size_t bus) {
    SyncEchoTrace snapshot;
    {
      std::lock_guard<std::mutex> lock(tx_history_mutex_);
      snapshot = echo_trace_[bus];
    }
    if (!snapshot.size()) return;
    unsigned hardware = 0;
    for (std::size_t i=0;i<snapshot.size();++i) hardware += snapshot.at(i).hardware_ns != 0;
    std::fprintf(stderr,"coarm7_hardware: pre-fault SYNC echo bus=%s samples=%zu hardware=%u "
                 "columns=counter,echo_read_mono_ns,tx_hw_raw_ns (0=missing hardware timestamp)\n",
                 config_.buses[bus].c_str(),snapshot.size(),hardware);
    for (std::size_t i=0;i<snapshot.size();++i) {
      const auto &echo = snapshot.at(i);
      std::fprintf(stderr,"coarm7_hardware: SYNC echo trace %s %u,%lld,%lld\n",
                   config_.buses[bus].c_str(),unsigned(echo.counter),static_cast<long long>(echo.received_ns),
                   static_cast<long long>(echo.hardware_ns));
    }
  }
#endif

  // Called only after the timestamp reader has joined.
  void report_timestamp_counts() const noexcept {
    for (std::size_t i=0;i<buses_.size();++i) {
      const auto &counts = tx_counts_[i];
      if (counts[0] || counts[1] || counts[2])
        std::fprintf(stderr,"coarm7_hardware: SYNC TX timestamp counts bus=%s scheduled=%llu software=%llu hardware=%llu\n",
                     config_.buses[i].c_str(),static_cast<unsigned long long>(counts[0]),
                     static_cast<unsigned long long>(counts[1]),static_cast<unsigned long long>(counts[2]));
      if (echo_counts_[i])
        std::fprintf(stderr,"coarm7_hardware: SYNC echo counts bus=%s received=%llu hardware=%llu\n",
                     config_.buses[i].c_str(),static_cast<unsigned long long>(echo_counts_[i]),
                     static_cast<unsigned long long>(echo_hardware_counts_[i]));
    }
  }

  void report_sync_timing() const noexcept {
    const auto &skew = sync_cadence_.bus_skew();
    if (skew.samples) {
      std::fprintf(stderr,"coarm7_hardware: SYNC interbus host start gap %s->%s "
                   "samples=%llu min_us=%.1f max_us=%.1f\n",
                   config_.buses[0].c_str(),config_.buses[1].c_str(),
                   static_cast<unsigned long long>(skew.samples),
                   skew.shortest_ns/1000.0,skew.longest_ns/1000.0);
    }
    for (std::size_t i=0;i<buses_.size();++i) {
      const auto &t = sync_cadence_.timing(i);
      if (!t.intervals && !t.failures) continue;
      std::fprintf(stderr,
        "coarm7_hardware: SYNC host timing bus=%s expected_us=%d intervals=%llu failures=%llu "
        "min_interval_us=%.1f (%s->%s) max_interval_us=%.1f (%s->%s) max_write_us=%.1f "
        "reschedules=%llu max_batch_late_us=%.1f\n",
        config_.buses[i].c_str(),config_.period_us,
        static_cast<unsigned long long>(t.intervals),static_cast<unsigned long long>(t.failures),
        t.shortest.ns/1000.0,sync_phase_name(t.shortest.from),sync_phase_name(t.shortest.to),
        t.longest.ns/1000.0,sync_phase_name(t.longest.from),sync_phase_name(t.longest.to),t.max_write_ns/1000.0,
        static_cast<unsigned long long>(sync_cadence_.reschedules()),sync_cadence_.max_lateness_ns()/1000.0);
    }
  }
  void write_parameter_report() const {
    YAML::Node report;
    report["sdk_api"]=sdk_extensions_available() ? "development_extensions" : "public";
    report["period_us"]=config_.period_us;
    report["host_pdo_cache_timeout_ms"]=sdk_extensions_available() ? config_.joint_timeout_ms : config_.sdk_cache_timeout_ms();
    report["feedback_max_age_ms"]=config_.joint_timeout_ms;
    report["feedback_snapshot_timeout_ms"]=config_.feedback_snapshot_timeout_ms;
    report["sdo_timeout_ms"]=config_.sdo_timeout_ms;
    for(std::size_t i=0;i<kArms;++i) {
      const auto &a=config_.axes[i]; const auto &p=parameters_[i];
      YAML::Node n;
      n["joint"]=a.name; n["bus"]=config_.buses[a.bus]; n["id"]=a.id;
      n["protocol_raw"]=p.protocol; n["firmware_raw"]=p.firmware; n["hardware_raw"]=p.hardware;
      n["sync_us"]=p.sync_us; n["drive_comm_timeout_ms"]=p.comm_timeout_ms;
      n["drive_fault_code_read_result"]=p.fault_code_result;
      n["checked_sdo_results"]=p.checked_sdo_results;
      if (p.fault_code_result == 0) n["drive_fault_code_raw"]=p.fault_code;
      n["brake_status"]=p.brake_status;
      n["position_limits_source"]="urdf";
      n["drive_lower_read_result"]=p.lower_result; n["drive_upper_read_result"]=p.upper_result;
      n["drive_position_bounds_valid"]=p.position_bounds_valid;
      n["drive_lower_rad"]=p.lower; n["drive_upper_rad"]=p.upper;
      n["drive_velocity_rad_s"]=p.max_velocity; n["drive_acceleration_rad_s2"]=p.max_acceleration;
      n["drive_deceleration_rad_s2"]=p.max_deceleration; n["stop_deceleration_rad_s2"]=p.stop_deceleration;
      n["torque_constant_Nm_A"]=p.torque_constant;
      n["effective_lower_rad"]=a.lower; n["effective_upper_rad"]=a.upper;
      n["effective_velocity_rad_s"]=a.max_velocity; n["effective_acceleration_rad_s2"]=a.max_acceleration;
      report["axes"].push_back(n);
    }
    std::cerr << "coarm7_hardware: non-RT device parameter snapshot\n" << report << '\n';
    for (std::size_t i=0;i<kArms;++i) {
      const auto &p=parameters_[i];
      if (p.fault_code_result == 0 && p.fault_code == 0) continue;
      const auto &a=config_.axes[i];
      std::cerr << "coarm7_hardware: pre-SYNC fault register joint=" << a.name
                << " bus=" << config_.buses[a.bus] << " node_id=" << a.id
                << " object=0x0008 read_result=" << p.fault_code_result;
      if (p.fault_code_result == 0)
        std::cerr << " code=0x" << std::hex << p.fault_code << std::dec;
      std::cerr << '\n';
    }
    if(!config_.device_report_file.empty()) {
      std::ofstream out(config_.device_report_file);
      out << report << '\n'; out.close();
      if(!out) throw std::runtime_error("Cannot save device parameter snapshot: "+config_.device_report_file);
    }
  }
  float device_position(std::size_t i,double p) const { return (p-config_.axes[i].offset)/config_.axes[i].sign; }
  Config config_;
  std::array<std::unique_ptr<modi_bus_canevo>,2> buses_;
  std::array<std::unique_ptr<modi_joint_canevo>,kArms> joints_;
  std::array<std::unique_ptr<modi_tool_io_canevo>,2> tools_;
  std::array<std::unique_ptr<GripperProtocol>,2> grippers_;
  std::array<uint32_t,2> sequences_{};
  std::array<DriveParameters,kArms> parameters_{};
  Snapshot startup_snapshot_{}, last_feedback_{};
  LatestSample<Snapshot> feedback_;
  LatestSample<Targets> targets_;
  Targets target_{};
  Positions output_{}, output_velocity_{}, stop_positions_{};
  std::array<bool,kGrippers> tool_stopped_{};
  std::array<bool,kGrippers> tool_claimed_{};
  std::array<std::array<uint64_t,5>,kGrippers> tool_tx_counts_{};
  std::array<uint64_t,kGrippers> tool_tx_failures_{};
  SyncPhase cycle_phase_ = SyncPhase::configure;
  std::atomic<SyncPhase> phase_{SyncPhase::configure};
  std::atomic<bool> running_{false}, stop_requested_{false}, io_failed_{false};
  std::array<uint64_t,2> request_goals_{},request_stops_{},handled_goals_{},handled_stops_{};
  std::atomic<SyncPhase> completed_phase_{SyncPhase::configure};
  std::atomic<bool> fault_latched_{false};
  std::atomic<int> start_result_{-1};
  std::atomic<int64_t> stop_sent_ns_{0};
  bool stop_position_set_ = false, fault_reported_ = false, recovery_only_ = false;
  const char *fault_reason_ = nullptr;
  int64_t fault_ns_ = 0;
  FaultObservation observed_emcy_{};
  std::size_t observed_emcy_bus_ = 0;
  SyncCadence sync_cadence_{5000};
  SyncFaultTrace trace_;
  std::mutex tx_history_mutex_;  // Timestamp reader and fault reporting only.
  std::array<SyncTxHistory,2> tx_history_{};
  std::array<std::array<uint64_t,3>,2> tx_counts_{};
#if COARM7_SDK_EXTENSIONS
  std::array<SyncEchoTrace,2> echo_trace_{};
#endif
  std::array<uint64_t,2> echo_counts_{}, echo_hardware_counts_{};
  std::atomic<bool> timestamp_reader_running_{false};
  std::thread worker_, timestamp_reader_;
};
}
std::unique_ptr<Transport> make_transport(const Config &c) {
  c.validate();
  if(c.transport=="fake") return std::make_unique<FakeTransport>();
  return std::make_unique<SdkTransport>();
}

bool clear_sdk_drive_faults(const Config &config) {
  if (config.transport != "sdk") throw std::invalid_argument("Drive fault recovery requires an SDK configuration");
  SdkTransport transport;
  return transport.clear_faults(config);
}
}
