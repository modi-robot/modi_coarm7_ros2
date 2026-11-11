#include "coarm7_hardware/device.hpp"
#include "modi_joint_canevo.h"
#if COARM7_SDK_EXTENSIONS
#include "canevo_impl.h"
#endif
#include <cerrno>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/errqueue.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include "coarm7_hardware/gripper.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <algorithm>
#include <map>
#include <sstream>
using namespace coarm7_hardware;
namespace {
struct RecoveryStub : DriveFaultRecovery {
  bool stationary_before = true, stationary_after = true;
  bool fail_post_read = false;
  int fail_pre_axis = -1, persistent_axis = -1;
  int change_before_reset_axis = -1, fault_after_reset_axis = -1;
  std::array<uint16_t,kArms> faults{};
  std::vector<std::size_t> reads, writes;
  std::vector<bool> stationary_checks;
  std::size_t reads_before_first_feedback = 0;

  bool wait_for_stationary_drives(bool allow_faults) override {
    if (stationary_checks.empty()) reads_before_first_feedback = reads.size();
    stationary_checks.push_back(allow_faults);
    if (allow_faults && change_before_reset_axis >= 0) faults[change_before_reset_axis] = 0xff07;
    if (!allow_faults && fault_after_reset_axis >= 0) faults[fault_after_reset_axis] = 0xff07;
    return allow_faults ? stationary_before : stationary_after;
  }
  int read_drive_fault(std::size_t axis, uint16_t &code) override {
    reads.push_back(axis);
    if (static_cast<int>(axis) == fail_pre_axis || (fail_post_read && !writes.empty())) return -1;
    code = faults[axis];
    return 0;
  }
  int clear_drive_fault(std::size_t axis) override {
    writes.push_back(axis);
    if (static_cast<int>(axis) != persistent_axis) faults[axis] = 0;
    return 0;
  }
};

TEST(DriveRecovery, AllowsOnlyNonRunningDriveStates) {
  EXPECT_TRUE(drive_allows_recovery(0x1001,false));
  EXPECT_TRUE(drive_allows_recovery(0x1002,false));
  EXPECT_TRUE(drive_allows_recovery(0x1007,true));
  EXPECT_FALSE(drive_allows_recovery(0x1007,false));
  for (uint16_t state : {0x1003,0x1004,0x1006,0x1000,0x100F}) {
    EXPECT_FALSE(drive_allows_recovery(state,true));
  }
}

TEST(DriveRecovery, ReadsAllJointsBeforeFeedbackAndClearsOnlyFf05OnBothArms) {
  const auto config=Config::load(CONFIG_PATH);
  std::ostringstream log;
  RecoveryStub drives;
  drives.faults[3]=0xff05;
  drives.faults[kArms-1]=0xff05;
  ASSERT_TRUE(recover_drive_faults(drives,config,log)) << log.str();
  EXPECT_EQ(drives.reads_before_first_feedback,kArms);
  EXPECT_EQ(drives.writes,(std::vector<std::size_t>{3,kArms-1}));
  EXPECT_EQ(drives.stationary_checks,(std::vector<bool>{true,true,false}));
  EXPECT_NE(log.str().find("joint=L_joint4 bus=can0 node_id=4 before=0xff05 clear_result=0 read_after_result=0 after=0x0"),
            std::string::npos);
  EXPECT_EQ(drives.faults[3],0);
  EXPECT_EQ(drives.faults[kArms-1],0);
}

TEST(DriveRecovery, HealthyDrivesRequireFreshFeedbackWithoutWrites) {
  const auto config=Config::load(CONFIG_PATH);
  std::ostringstream log;
  RecoveryStub drives;
  EXPECT_TRUE(recover_drive_faults(drives,config,log)) << log.str();
  EXPECT_EQ(drives.reads_before_first_feedback,kArms);
  EXPECT_EQ(drives.reads.size(),2*kArms);
  EXPECT_EQ(drives.stationary_checks,(std::vector<bool>{false}));
  EXPECT_TRUE(drives.writes.empty());
}

TEST(DriveRecovery, RefusesAllWritesWhenAnyInitialCodeIsUnknownOrUnreadable) {
  const auto config=Config::load(CONFIG_PATH);
  std::ostringstream log;

  RecoveryStub mixed;
  mixed.faults[0]=0xff05;
  mixed.faults[kArms-2]=0xff07;
  EXPECT_FALSE(recover_drive_faults(mixed,config,log));
  EXPECT_EQ(mixed.reads.size(),kArms);
  EXPECT_TRUE(mixed.stationary_checks.empty());
  EXPECT_TRUE(mixed.writes.empty());

  RecoveryStub unreadable;
  unreadable.faults[0]=0xff05;
  unreadable.fail_pre_axis=kArms-2;
  EXPECT_FALSE(recover_drive_faults(unreadable,config,log));
  EXPECT_EQ(unreadable.reads.size(),kArms);
  EXPECT_TRUE(unreadable.stationary_checks.empty());
  EXPECT_TRUE(unreadable.writes.empty());
}

TEST(DriveRecovery, RefusesResetWithoutStandstillOrIfTargetFaultChanges) {
  const auto config=Config::load(CONFIG_PATH);
  std::ostringstream log;

  RecoveryStub moving;
  moving.faults[0]=0xff05;
  moving.stationary_before=false;
  EXPECT_FALSE(recover_drive_faults(moving,config,log));
  EXPECT_TRUE(moving.writes.empty());

  RecoveryStub changed;
  changed.faults[0]=0xff05;
  changed.change_before_reset_axis=0;
  EXPECT_FALSE(recover_drive_faults(changed,config,log));
  EXPECT_TRUE(changed.writes.empty());
}

TEST(DriveRecovery, ReportsFailureWhenResetCannotBeVerified) {
  const auto config=Config::load(CONFIG_PATH);
  std::ostringstream log;

  RecoveryStub failed_readback;
  failed_readback.faults[0]=0xff05;
  failed_readback.faults[1]=0xff05;
  failed_readback.fail_post_read=true;
  EXPECT_FALSE(recover_drive_faults(failed_readback,config,log));
  EXPECT_EQ(failed_readback.writes,(std::vector<std::size_t>{0}));

  RecoveryStub persistent;
  persistent.faults[0]=0xff05;
  persistent.persistent_axis=0;
  EXPECT_FALSE(recover_drive_faults(persistent,config,log));
  EXPECT_EQ(persistent.writes,(std::vector<std::size_t>{0}));

  RecoveryStub no_fresh_pdo;
  no_fresh_pdo.faults[0]=0xff05;
  no_fresh_pdo.stationary_after=false;
  EXPECT_FALSE(recover_drive_faults(no_fresh_pdo,config,log));
  EXPECT_EQ(no_fresh_pdo.stationary_checks,(std::vector<bool>{true,false}));

  RecoveryStub new_fault;
  new_fault.faults[0]=0xff05;
  new_fault.fault_after_reset_axis=kArms-1;
  EXPECT_FALSE(recover_drive_faults(new_fault,config,log));
  EXPECT_EQ(new_fault.writes,(std::vector<std::size_t>{0}));
}

struct Reader : ParameterReader {
  std::pair<int,int> fail{-1,-1};
  std::map<std::pair<int,int>,float> values{{{1,2},60},{{1,3},120},{{1,4},120},
    {{1,5},240},{{1,6},180},{{1,7},-180},{{0x1F,3},4.23f}};
  int u16(uint8_t i,uint8_t s,uint16_t &v) override {
    if(fail==std::make_pair(int(i),int(s))) return -4;
    v = i==1 ? 2 : s==0x08 ? 0 : s==0x0F ? 5000 : s==0x10 ? 100 : 0x0124;
    return 0;
  }
  int f32(uint8_t i,uint8_t s,float &v) override {
    if(fail==std::make_pair(int(i),int(s))) return -4;
    v=values.at({i,s}); return 0;
  }
};
struct Actions : StopActions {
  std::vector<std::string> calls;
  std::string fail;
  bool add(const std::string &v) { calls.push_back(v); return v!=fail; }
  bool quick_stop(std::size_t i) noexcept override { return add("stop"+std::to_string(i)); }
  bool hold(std::size_t i,double) noexcept override { return add("pdo"+std::to_string(i)); }
  bool stop_gripper(std::size_t i) noexcept override { return add("tool"+std::to_string(i)); }
  bool disable(std::size_t i) noexcept override { return add("disable"+std::to_string(i)); }
};
}
TEST(Device, CheckedReadConvertsUnitsAndAcceptsUnsupportedBrake) {
  Reader r; DriveParameters p; std::string error;
  ASSERT_TRUE(read_drive_parameters(r,p,error)) << error;
  EXPECT_NEAR(p.max_velocity,6.28318530718,1e-9);
  EXPECT_NEAR(p.lower,-3.14159265359,1e-9);
  EXPECT_NEAR(p.stop_deceleration,25.1327412287,1e-8);
  EXPECT_EQ(p.sync_us,5000); EXPECT_EQ(p.brake_status,2);
}
TEST(Device, FailedCriticalReadsNeverCommitPartialOrPlausibleZeroValues) {
  for(auto object : {std::pair<int,int>{0,1},{0,0x0D},{0,0x0E},{0,0x0F},{0,0x10},
                    {1,0x0D},{1,2},{1,3},{1,4},{1,5}}) {
    Reader r; r.fail=object; DriveParameters p; p.sync_us=123; std::string error;
    EXPECT_FALSE(read_drive_parameters(r,p,error)); EXPECT_EQ(p.sync_us,123);
    EXPECT_NE(error.find("rc=-4"),std::string::npos);
  }
}
TEST(Device, InvalidNumericParametersAreRejected) {
  for(float value : {0.f,-1.f,std::numeric_limits<float>::quiet_NaN()}) {
    Reader r; r.values[{1,5}]=value; DriveParameters p; std::string error;
    EXPECT_FALSE(read_drive_parameters(r,p,error));
  }
  Reader r; r.values[{1,6}]=-200; DriveParameters p; std::string error;
  EXPECT_TRUE(read_drive_parameters(r,p,error));
  EXPECT_FALSE(p.position_bounds_valid);
}
TEST(Device, KtFailureDoesNotBlockPositionOnlyControlOrFabricateTorque) {
  Reader r; r.fail={0x1F,3}; DriveParameters p; std::string error;
  ASSERT_TRUE(read_drive_parameters(r,p,error)); EXPECT_EQ(p.torque_constant,0);
}
TEST(Device, FaultRegisterIsCheckedWithoutTreatingReadFailureAsNoFault) {
  struct FaultReader : Reader {
    int u16(uint8_t index,uint8_t sub,uint16_t &value) override {
      if (index==0 && sub==0x08) { value=0xFF05; return 0; }
      return Reader::u16(index,sub,value);
    }
  } reader;
  DriveParameters parameters;
  std::string error;
  ASSERT_TRUE(read_drive_parameters(reader,parameters,error));
  EXPECT_EQ(parameters.fault_code_result,0);
  EXPECT_EQ(parameters.fault_code,0xFF05);

  Reader failed;
  failed.fail={0,0x08};
  ASSERT_TRUE(read_drive_parameters(failed,parameters,error));
  EXPECT_EQ(parameters.fault_code_result,-4);
  EXPECT_EQ(parameters.fault_code,0);
}
TEST(Device, DriverPositionBoundsDoNotChangeUrdfLimitsIncludingReverseAxes) {
  auto c=Config::load(CONFIG_PATH); auto a=c.axes[0];
  Reader r; DriveParameters p; std::string error; ASSERT_TRUE(read_drive_parameters(r,p,error));
  p.lower=-0.3; p.upper=0.6; p.max_velocity=0.2; p.max_acceleration=0.7;
  a.sign=-1; a.offset=0.1;
  ASSERT_TRUE(constrain_axis(a,p,c,error));
  EXPECT_DOUBLE_EQ(a.lower,c.axes[0].lower); EXPECT_DOUBLE_EQ(a.upper,c.axes[0].upper);
  EXPECT_EQ(a.max_velocity,0.2); EXPECT_EQ(a.max_acceleration,0.7);
  p.lower=-10; p.upper=10; p.max_velocity=20; p.max_acceleration=100;
  ASSERT_TRUE(constrain_axis(a,p,c,error));
  EXPECT_DOUBLE_EQ(a.lower,c.axes[0].lower); EXPECT_EQ(a.max_velocity,0.2);
}
TEST(Device, SyncMismatchAndWatchdogStillRejectButDisjointDriverLimitsDoNot) {
  const auto c=Config::load(CONFIG_PATH); auto a=c.axes[0];
  Reader r; DriveParameters p; std::string error; ASSERT_TRUE(read_drive_parameters(r,p,error));
  p.sync_us=1000; EXPECT_FALSE(constrain_axis(a,p,c,error));
  p.sync_us=5000; p.comm_timeout_ms=5; EXPECT_FALSE(constrain_axis(a,p,c,error));
  p.comm_timeout_ms=100; p.lower=10; p.upper=20; EXPECT_TRUE(constrain_axis(a,p,c,error));
  EXPECT_DOUBLE_EQ(a.lower,c.axes[0].lower); EXPECT_DOUBLE_EQ(a.upper,c.axes[0].upper);
}
TEST(Stop, CachedBitIsFollowedByPdoForEveryAxisThenBothTools) {
  Actions a; Positions p{}; ASSERT_TRUE(request_quick_stop(a,p));
  std::vector<std::string> expected;
  for(std::size_t i=0;i<kArms;++i) { expected.push_back("stop"+std::to_string(i)); expected.push_back("pdo"+std::to_string(i)); }
  expected.insert(expected.end(),{"tool0","tool1"});
  EXPECT_EQ(a.calls,expected);
}
TEST(Stop, FailedBusOrAxisNeverSkipsStoppingTheOtherArm) {
  for(const auto &fail : {"stop0","pdo0","tool0"}) {
    Actions a; a.fail=fail; EXPECT_FALSE(request_quick_stop(a,{}));
    ASSERT_EQ(a.calls.size(),30u); EXPECT_EQ(a.calls.back(),"tool1");
    EXPECT_NE(std::find(a.calls.begin(),a.calls.end(),"pdo13"),a.calls.end());
  }
}
TEST(Stop, OrdinaryDisableDoesNotLatchQuickStopAndContinuesAfterFailure) {
  Actions a; a.fail="disable0"; EXPECT_FALSE(disable_all(a));
  ASSERT_EQ(a.calls.size(),14u); EXPECT_EQ(a.calls[0],"disable0");
  EXPECT_EQ(a.calls[1],"disable1"); EXPECT_EQ(a.calls.back(),"disable13");
  for(const auto &call:a.calls) EXPECT_EQ(call.find("stop"),std::string::npos);
}
TEST(Stop, MovingOrMissingFeedbackPreventsPrematureDisableAfterFaultStop) {
  StandstillMonitor monitor(0.02); std::array<AxisFeedback,kArms> f{};
  for(auto &a:f) a.valid=true;
  EXPECT_FALSE(monitor.update(f)); EXPECT_FALSE(monitor.update(f));
  f[7].velocity=0.2; EXPECT_FALSE(monitor.update(f));
  f[7].velocity=0; EXPECT_FALSE(monitor.update(f));
  f[0].valid=false; EXPECT_FALSE(monitor.update(f));
  f[0].valid=true; EXPECT_FALSE(monitor.update(f)); EXPECT_FALSE(monitor.update(f));
  EXPECT_TRUE(monitor.update(f));
  f[3].velocity=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(monitor.update(f));
}
TEST(CanCycle, ServicesEachBusBeforeTheNextSyncAndStillStepsOnFailure) {
  std::vector<int> calls;
  auto sync=[&](std::size_t bus) {
    calls.push_back(2*static_cast<int>(bus));
    return true;
  };
  auto tools=[&](std::size_t bus) {
    calls.push_back(2*static_cast<int>(bus)+1);
    return true;
  };
  auto result=step_can_buses(2,sync,tools);
  EXPECT_EQ(calls,(std::vector<int>{0,1,2,3}));
  EXPECT_TRUE(result.sync_ok);
  EXPECT_TRUE(result.tools_ok);

  calls.clear();
  result=step_can_buses(2,[&](std::size_t bus) {
    sync(bus);
    return bus!=0;
  },tools);
  EXPECT_EQ(calls,(std::vector<int>{0,2,3}));
  EXPECT_FALSE(result.sync_ok);
  EXPECT_TRUE(result.tools_ok);

  calls.clear();
  result=step_can_buses(2,sync,[&](std::size_t bus) {
    tools(bus);
    return bus!=0;
  });
  EXPECT_EQ(calls,(std::vector<int>{0,1,2,3}));
  EXPECT_TRUE(result.sync_ok);
  EXPECT_FALSE(result.tools_ok);
}

TEST(SyncCadence, MeasuresSameCycleGapSeparatelyFromFiveMillisecondIntervals) {
  SyncCadence cadence(5000);
  constexpr auto phase=SyncPhase::control;
  for (const int64_t time : {1000000000LL,1005000000LL}) {
    cadence.begin_batch(time);
    cadence.record(0,time,time+10000,true,phase);
    cadence.record(1,time+45000,time+55000,true,phase);
  }
  EXPECT_EQ(cadence.bus_skew().samples,2u);
  EXPECT_EQ(cadence.bus_skew().shortest_ns,45000);
  EXPECT_EQ(cadence.bus_skew().longest_ns,45000);
  EXPECT_EQ(cadence.timing(0).longest.ns,5000000);
  EXPECT_EQ(cadence.timing(1).longest.ns,5000000);

  constexpr int64_t next=1010000000;
  cadence.begin_batch(next);
  cadence.record(0,next,next+10000,false,phase);
  cadence.record(1,next+85000,next+95000,true,phase);
  EXPECT_EQ(cadence.bus_skew().samples,2u);
}

TEST(SyncCadence, PhaseChangesKeepFiveMillisecondCadenceIndependentOfWriteDuration) {
  constexpr int64_t first=1000000000, period=5000000, write_ns=12000;
  SyncCadence cadence(5000);
  int64_t slot=first;
  for (auto phase : {SyncPhase::configure,SyncPhase::control,SyncPhase::activate,
                     SyncPhase::control,SyncPhase::deactivate,SyncPhase::fault_stop}) {
    ASSERT_TRUE(cadence.due(slot));
    cadence.begin_batch(slot);
    cadence.record(0,slot,slot+write_ns,true,phase);
    cadence.record(1,slot+write_ns,slot+2*write_ns,true,phase);
    slot+=period;
    EXPECT_FALSE(cadence.due(slot-1));
    EXPECT_EQ(cadence.next_send_ns(),slot);
  }
  EXPECT_EQ(cadence.reschedules(),0u);
  for (std::size_t bus=0;bus<2;++bus) {
    const auto &timing=cadence.timing(bus);
    EXPECT_EQ(timing.intervals,5u);
    EXPECT_EQ(timing.shortest.ns,period);
    EXPECT_EQ(timing.longest.ns,period);
    EXPECT_EQ(timing.shortest.from,SyncPhase::configure);
    EXPECT_EQ(timing.shortest.to,SyncPhase::control);
    EXPECT_EQ(timing.max_write_ns,write_ns);
  }
}
TEST(SyncCadence, SmallLatenessRecoversPhaseWithoutAccumulatingDrift) {
  constexpr int64_t first=1000000000, period=5000000;
  SyncCadence cadence(5000);
  cadence.begin_batch(first);
  for (int i=1;i<=1000;++i) {
    const auto slot=first+i*period;
    EXPECT_EQ(cadence.next_send_ns(),slot);
    const auto late=i%2 ? 10000 : 0;
    cadence.begin_batch(slot+late);
    cadence.record(0,slot+late,slot+late+13000,true,SyncPhase::control);
    cadence.record(1,slot+late+13000,slot+late+30000,true,SyncPhase::control);
  }
  EXPECT_EQ(cadence.reschedules(),0u);
  EXPECT_EQ(cadence.max_lateness_ns(),10000);
  EXPECT_EQ(cadence.timing(0).shortest.ns,period-10000);
  EXPECT_EQ(cadence.timing(0).longest.ns,period+10000);
  EXPECT_EQ(cadence.timing(0).recent[(999-1)%8].ns,period-10000);
}
TEST(SyncCadence, WakeupsLaterThanTwentyMicrosecondsKeepTheSystemTimeGrid) {
  constexpr int64_t first=1000000000, period=5000000;
  SyncCadence cadence(5000);
  cadence.begin_batch(first);
  cadence.record(0,first,first+13000,true,SyncPhase::control);
  cadence.finish_batch(first+300000);
  for (int i=1;i<=1000;++i) {
    const auto slot=first+i*period;
    const int64_t late=i%2 ? 100000 : 0;
    ASSERT_EQ(cadence.next_send_ns(),slot);
    cadence.begin_batch(slot+late);
    cadence.record(0,slot+late,slot+late+13000,true,SyncPhase::control);
    cadence.finish_batch(slot+late+300000);
    EXPECT_EQ(cadence.next_send_ns(),slot+period);
  }
  EXPECT_EQ(cadence.reschedules(),0u);
  EXPECT_EQ(cadence.max_lateness_ns(),100000);
  EXPECT_EQ(cadence.timing(0).shortest.ns,period-100000);
  EXPECT_EQ(cadence.timing(0).longest.ns,period+100000);
}

TEST(PositionPdoGroups, SendsCompleteConfiguredBusGroupsWithoutReleaseTimers) {
  constexpr int64_t start=1000000000, period=5000000;
  auto axes = Config::load(CONFIG_PATH).axes;
  // The grouping follows the configured bus even if the axes are interleaved.
  std::swap(axes[0],axes[7]);
  SyncCadence cadence(5000);
  cadence.begin_batch(start);
  std::vector<int64_t> calls;
  ASSERT_TRUE(send_position_pdos_by_bus(axes,
    [&](std::size_t bus) { calls.push_back(-1-static_cast<int64_t>(bus)); return true; },
    [&](std::size_t axis) { calls.push_back(static_cast<int64_t>(axis)); return true; }));
  EXPECT_EQ(calls,(std::vector<int64_t>{-1,1,2,3,4,5,6,7,-2,0,8,9,10,11,12,13}));
  cadence.finish_batch(start+650000);
  EXPECT_EQ(cadence.next_send_ns(),start+period);
  EXPECT_EQ(cadence.reschedules(),0u);
}

TEST(PositionPdoGroups, StopBeforeSecondGroupPreventsRightArmTransmission) {
  const auto axes = Config::load(CONFIG_PATH).axes;
  int checks = 0;
  std::vector<std::size_t> sent;
  EXPECT_FALSE(send_position_pdos_by_bus(axes,
    [&](std::size_t) { return ++checks == 1; },
    [&](std::size_t axis) { sent.push_back(axis); return true; }));
  EXPECT_EQ(checks,2);
  EXPECT_EQ(sent,(std::vector<std::size_t>{0,1,2,3,4,5,6}));
}

TEST(SyncCadence, SlowFeedbackOrPdoWorkRebasesAfterTheWholeCycle) {
  constexpr int64_t first=1000000000, period=5000000;
  SyncCadence cadence(5000);
  cadence.begin_batch(first);
  cadence.record(0,first,first+10000,true,SyncPhase::control);
  cadence.record(1,first+20000,first+30000,true,SyncPhase::control);
  const auto completed=first+period+200000;
  cadence.finish_batch(completed);
  EXPECT_EQ(cadence.next_send_ns(),completed+period);
  EXPECT_EQ(cadence.reschedules(),1u);
  EXPECT_FALSE(cadence.due(completed));
}

TEST(SyncCadence, CompleteCycleOverrunStartsFreshPeriod) {
  constexpr int64_t first=1000000000, period=5000000, write_ns=10000;
  SyncCadence cadence(5000);
  cadence.begin_batch(first);
  cadence.record(0,first,first+write_ns,true,SyncPhase::control);
  const auto late=first+period+4900000;
  ASSERT_TRUE(cadence.due(late));
  cadence.begin_batch(late);
  cadence.record(0,late,late+write_ns,true,SyncPhase::control);
  const auto completed=late+100000;
  cadence.finish_batch(completed);
  EXPECT_FALSE(cadence.due(completed));
  const auto next=cadence.next_send_ns();
  EXPECT_EQ(next,completed+period);
  EXPECT_EQ(cadence.reschedules(),1u);
  cadence.begin_batch(next);
  cadence.record(0,next,next+write_ns,true,SyncPhase::control);
  const auto &timing=cadence.timing(0);
  EXPECT_EQ(timing.longest.ns,period+4900000);
  EXPECT_EQ(timing.shortest.ns,period+100000);
}
TEST(SyncCadence, WriteOverrunDoesNotCauseImmediateRetryOrFabricateAnInterval) {
  constexpr int64_t first=1000000000, period=5000000;
  SyncCadence cadence(5000);
  cadence.begin_batch(first);
  cadence.record(0,first,first+1000,true,SyncPhase::control);
  const auto failed=cadence.next_send_ns();
  cadence.begin_batch(failed);
  cadence.record(0,failed,failed+period+2000,false,SyncPhase::fault_stop);
  cadence.finish_batch(failed+period+2000);
  EXPECT_FALSE(cadence.due(failed+period+3000));
  EXPECT_EQ(cadence.reschedules(),1u);
  EXPECT_EQ(cadence.timing(0).failures,1u);
  EXPECT_EQ(cadence.timing(0).intervals,0u);
  const auto retry=cadence.next_send_ns();
  cadence.begin_batch(retry);
  cadence.record(0,retry,retry+1000,true,SyncPhase::deactivate);
  EXPECT_EQ(cadence.timing(0).longest.ns,3*period+2000);
  EXPECT_EQ(cadence.timing(0).longest.to,SyncPhase::deactivate);
}
TEST(Protocol, ExplicitActivationRestartsPollingWithoutReplayingOldOpening) {
  GripperProtocol p(Config::load(CONFIG_PATH),0); p.target(100); p.stop(); p.resume();
  EXPECT_EQ(p.next(1000000000).data[4],0x41);
  EXPECT_NE(p.next(1300000000).data[4],0x54);
  p.target(600); EXPECT_EQ(p.next(1600000000).data[4],0x54);
}


TEST(SyncFaultTrace, PreservesLastWritesInOrderAcrossWrapAndFreeze) {
  SyncFaultTrace trace;
  for (std::size_t n=0;n<SyncFaultTrace::capacity+5;++n) {
    const auto start=static_cast<int64_t>(n)*5000000;
    trace.record(0,{start,start+1000,0,static_cast<uint8_t>(n),SyncPhase::control});
  }
  trace.record(1,{10,20,-3,8,SyncPhase::activate,12,18,-1,ENOBUFS});
  trace.freeze();
  trace.record(0,{0,0,-3,0,SyncPhase::fault_stop});
  ASSERT_EQ(trace.size(0),SyncFaultTrace::capacity);
  EXPECT_EQ(trace.at(0,0).started_ns,25000000);
  EXPECT_EQ(trace.at(0,trace.size(0)-1).counter,4u);
  ASSERT_EQ(trace.size(1),1u);
  EXPECT_EQ(trace.at(1,0).result,-3);
  EXPECT_EQ(trace.at(1,0).write_entered_ns,12);
  EXPECT_EQ(trace.at(1,0).write_returned_ns,18);
  EXPECT_EQ(trace.at(1,0).write_bytes,-1);
  EXPECT_EQ(trace.at(1,0).write_errno,ENOBUFS);
}

TEST(LatestSample, ConcurrentConsumerNeverSeesATornSample) {
  struct Message { unsigned sequence; std::array<unsigned,16> fields; };
  constexpr unsigned count = 50000;
  LatestSample<Message> mailbox;
  std::atomic<bool> done{false};
  std::thread producer([&] {
    for (unsigned i=1;i<=count;++i) {
      Message message{};
      message.sequence = i;
      message.fields.fill(i);
      mailbox.publish(message);
    }
    done.store(true);
  });
  unsigned observed = 0;
  while (true) {
    const bool producer_finished = done.load();
    Message message{};
    if (mailbox.take(message)) {
      EXPECT_GT(message.sequence,observed);
      for (const auto field:message.fields) EXPECT_EQ(field,message.sequence);
      observed = message.sequence;
    } else if (producer_finished) {
      break;
    }
  }
  producer.join();
  EXPECT_EQ(observed,count);
}

#if COARM7_SDK_EXTENSIONS
TEST(SdkDiagnostics, DecodesOnlyValidTimestampEventsAndKeepsClockSourcesSeparate) {
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(scm_timestamping)) +
                                CMSG_SPACE(sizeof(sock_extended_err))]{};
  msghdr message{};
  message.msg_control=control;
  message.msg_controllen=sizeof(control);
  auto *first=CMSG_FIRSTHDR(&message);
  first->cmsg_level=SOL_SOCKET;
  first->cmsg_type=SCM_TIMESTAMPING;
  first->cmsg_len=CMSG_LEN(sizeof(scm_timestamping));
  auto *second=CMSG_NXTHDR(&message,first);
  ASSERT_NE(second,nullptr);
  second->cmsg_level=SOL_CAN_RAW;
  second->cmsg_type=SCM_CAN_RAW_ERRQUEUE;
  second->cmsg_len=CMSG_LEN(sizeof(sock_extended_err));

  scm_timestamping clocks{};
  clocks.ts[0]={12,345};
  sock_extended_err event{};
  event.ee_errno=ENOMSG;
  event.ee_origin=SO_EE_ORIGIN_TIMESTAMPING;
  event.ee_info=SCM_TSTAMP_SCHED;
  event.ee_data=257;
  const auto write_controls=[&] {
    std::memcpy(CMSG_DATA(first),&clocks,sizeof(clocks));
    std::memcpy(CMSG_DATA(second),&event,sizeof(event));
  };
  CanEvoSyncTimestamp stamp;
  write_controls();
  ASSERT_TRUE(canevo::DecodeSyncTimestamp(message,stamp));
  EXPECT_EQ(stamp.id,257u);
  EXPECT_EQ(stamp.ns,12000000345LL);
  EXPECT_EQ(stamp.kind,CanEvoSyncTimestampKind::scheduled);

  event.ee_info=SCM_TSTAMP_SND;
  write_controls();
  ASSERT_TRUE(canevo::DecodeSyncTimestamp(message,stamp));
  EXPECT_EQ(stamp.kind,CanEvoSyncTimestampKind::software);

  clocks.ts[2]={7,456};
  write_controls();
  ASSERT_TRUE(canevo::DecodeSyncTimestamp(message,stamp));
  EXPECT_EQ(stamp.kind,CanEvoSyncTimestampKind::hardware);
  EXPECT_EQ(stamp.ns,7000000456LL);

  message.msg_flags=MSG_CTRUNC;
  EXPECT_FALSE(canevo::DecodeSyncTimestamp(message,stamp));
  message.msg_flags=0;
  event.ee_origin=0;
  write_controls();
  EXPECT_FALSE(canevo::DecodeSyncTimestamp(message,stamp));
  event.ee_origin=SO_EE_ORIGIN_TIMESTAMPING;
  clocks={};
  write_controls();
  EXPECT_FALSE(canevo::DecodeSyncTimestamp(message,stamp));
}

TEST(SdkDiagnostics, AcceptsOnlyOwnSyncEchoAndUsesRawHardwareTime) {
  alignas(cmsghdr) char control[CMSG_SPACE(sizeof(scm_timestamping))]{};
  msghdr message{};
  message.msg_control=control;
  message.msg_controllen=sizeof(control);
  auto *cmsg=CMSG_FIRSTHDR(&message);
  cmsg->cmsg_level=SOL_SOCKET;
  cmsg->cmsg_type=SCM_TIMESTAMPING;
  cmsg->cmsg_len=CMSG_LEN(sizeof(scm_timestamping));
  scm_timestamping clocks{};
  clocks.ts[0]={10,123};
  std::memcpy(CMSG_DATA(cmsg),&clocks,sizeof(clocks));
  canfd_frame frame{};
  frame.can_id=canevo::kCobSync;
  frame.len=2;
  frame.data[0]=0x3c;
  frame.data[1]=255;

  CanEvoSyncEcho echo;
  message.msg_flags=MSG_DONTROUTE;
  EXPECT_FALSE(canevo::DecodeSyncEcho(message,frame,echo));
  message.msg_flags=MSG_DONTROUTE | MSG_CONFIRM;
  ASSERT_TRUE(canevo::DecodeSyncEcho(message,frame,echo));
  EXPECT_EQ(echo.counter,255);
  EXPECT_GT(echo.received_ns,0);
  EXPECT_EQ(echo.hardware_ns,0);

  clocks.ts[2]={20,456};
  std::memcpy(CMSG_DATA(cmsg),&clocks,sizeof(clocks));
  frame.data[1]=0;
  ASSERT_TRUE(canevo::DecodeSyncEcho(message,frame,echo));
  EXPECT_EQ(echo.counter,0);
  EXPECT_EQ(echo.hardware_ns,20000000456LL);

  message.msg_flags|=MSG_CTRUNC;
  EXPECT_FALSE(canevo::DecodeSyncEcho(message,frame,echo));
  message.msg_flags=MSG_CONFIRM;
  frame.can_id=canevo::kCobRxPdo0Base;
  EXPECT_FALSE(canevo::DecodeSyncEcho(message,frame,echo));
}

#endif
TEST(SyncTxHistory, JoinsOutOfOrderSourcesByKernelIdAcrossCounterWrap) {
  SyncTxHistory history;
  history.record(256,&SyncTxTimestamps::hardware_ns,17);
  history.record(0,&SyncTxTimestamps::software_ns,100);
  history.record(256,&SyncTxTimestamps::scheduled_ns,200);
  history.record(256,&SyncTxTimestamps::hardware_ns,999);
  EXPECT_EQ(history.get(256).hardware_ns,17);
  EXPECT_EQ(history.get(256).software_ns,0);
  EXPECT_EQ(history.get(256).scheduled_ns,200);
  EXPECT_EQ(history.get(0).software_ns,100);
  EXPECT_EQ(history.get(-1).hardware_ns,0);
  history.record(1280,&SyncTxTimestamps::software_ns,777);
  history.record(256,&SyncTxTimestamps::hardware_ns,18);
  EXPECT_EQ(history.get(256).hardware_ns,0);
  EXPECT_EQ(history.get(1280).software_ns,777);
}

TEST(SyncFaultTrace, AdmitsLateCompletionsForFrozenWindowOnly) {
  SyncFaultTrace trace;
  SyncWrite entry;
  for (uint32_t id : {UINT32_MAX-1,UINT32_MAX,0u,1u}) {
    entry.timestamp_id=id;
    trace.record(0,entry);
  }
  trace.freeze();
  entry.timestamp_id=2;
  trace.record(0,entry);
  EXPECT_TRUE(trace.includes_timestamp(0,UINT32_MAX));
  EXPECT_TRUE(trace.includes_timestamp(0,1));
  EXPECT_FALSE(trace.includes_timestamp(0,2));
  EXPECT_FALSE(trace.includes_timestamp(0,UINT32_MAX-1024));
  EXPECT_FALSE(trace.includes_timestamp(1,0));
}

#if COARM7_SDK_EXTENSIONS
TEST(SdkDiagnostics, FirstNonzeroEmcySurvivesLaterReports) {
  modi_bus_canevo::Impl bus;
  CanEvoFirstEmcy event;
  bus.RecordEmcy(9,0);
  EXPECT_FALSE(bus.GetFirstEmcy(event));
  EXPECT_FALSE(bus.TakePendingEmcy());
  bus.RecordEmcy(9,0xff05);
  EXPECT_TRUE(bus.TakePendingEmcy());
  EXPECT_FALSE(bus.TakePendingEmcy());
  ASSERT_TRUE(bus.GetFirstEmcy(event));
  const auto first_stamp = event.received_ns;
  EXPECT_GT(first_stamp,0);
  bus.RecordEmcy(10,0xff07);
  bus.RecordEmcy(9,0);
  EXPECT_TRUE(bus.TakePendingEmcy());
  EXPECT_FALSE(bus.TakePendingEmcy());
  ASSERT_TRUE(bus.GetFirstEmcy(event));
  EXPECT_EQ(event.received_ns,first_stamp);
  EXPECT_EQ(event.node_id,9);
  EXPECT_EQ(event.code,0xff05);
}

TEST(SdkDiagnostics, SendFrameMeasuresWriteAndPreservesFrameContents) {
  // A local datagram pair exercises the actual write without any CAN socket.
  int sockets[2];
  ASSERT_EQ(socketpair(AF_UNIX,SOCK_DGRAM,0,sockets),0);
  canevo::CanFrame frame{};
  frame.id = canevo::kCobSync;
  frame.len = 2;
  frame.data[0] = 0x3c;
  frame.data[1] = 84;
  CanEvoSyncWriteTiming timing;
  EXPECT_EQ(canevo::SendFrame(sockets[0],frame,&timing),0);
  EXPECT_GT(timing.entered_ns,0);
  EXPECT_GE(timing.returned_ns,timing.entered_ns);
  EXPECT_EQ(timing.bytes,sizeof(canfd_frame));
  EXPECT_EQ(timing.error_number,0);
  canfd_frame received{};
  EXPECT_EQ(recv(sockets[1],&received,sizeof(received),MSG_DONTWAIT),sizeof(received));
  EXPECT_EQ(received.can_id,canevo::kCobSync);
  EXPECT_EQ(received.flags,CANFD_BRS);
  EXPECT_EQ(received.len,2);
  EXPECT_EQ(received.data[0],0x3c);
  EXPECT_EQ(received.data[1],84);
  close(sockets[0]);
  close(sockets[1]);
}

TEST(SdkDiagnostics, FailedWriteRetainsErrnoAndTimestamp) {
  CanEvoSyncWriteTiming timing;
  EXPECT_EQ(canevo::SendFrame(-1,{},&timing),-1);
  EXPECT_GT(timing.entered_ns,0);
  EXPECT_GE(timing.returned_ns,timing.entered_ns);
  EXPECT_EQ(timing.bytes,-1);
  EXPECT_EQ(timing.error_number,EBADF);
}

TEST(SdkDiagnostics, SdkRejectionWithoutWriteLeavesNoSyscallRecord) {
  modi_bus_canevo bus;
  CanEvoSyncWriteTiming timing{1,2,-1,EBADF};
  uint8_t counter = 0;
  EXPECT_EQ(bus.RtStepOnce(counter,timing),static_cast<int>(CanEvoError::kBusNotOpen));
  EXPECT_EQ(timing.entered_ns,0);
  EXPECT_EQ(timing.returned_ns,0);
  EXPECT_EQ(timing.bytes,0);
  EXPECT_EQ(timing.error_number,0);
}
#endif

TEST(Device, UnreadableDriverPositionsAreDiagnosticOnly) {
  for (int sub : {6,7}) {
    Reader r; r.fail={1,sub}; DriveParameters p; std::string error;
    ASSERT_TRUE(read_drive_parameters(r,p,error));
    EXPECT_FALSE(p.position_bounds_valid);
    EXPECT_EQ(sub==6?p.upper_result:p.lower_result,-4);
    EXPECT_TRUE(error.empty());
  }
}
