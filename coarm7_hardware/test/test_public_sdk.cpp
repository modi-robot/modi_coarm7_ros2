#include "coarm7_hardware/sdk_public.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace coarm7_hardware;
namespace {
// Value-only public API facade: no SDK objects, sockets or bus are opened.
struct PublicJoint {
  std::string version="1.3.5";
  uint16_t sync=5000,watchdog=100,brake=0;
  CanEvoFault fault=CanEvoFault::kNone;
  float speed=4,accel=5,decel=6,stop=7,lower=-2,upper=2,kt=3;
  std::string NrtGetProtocolVersion() { return version; }
  std::string NrtGetFwVersion() { return version; }
  std::string NrtGetHwVersion() { return version; }
  uint16_t NrtGetSyncPeriod() { return sync; }
  uint16_t NrtGetCommTimeout() { return watchdog; }
  uint16_t NrtGetBrakeStatus() { return brake; }
  CanEvoFault NrtGetFaultCode() { return fault; }
  float NrtGetMaxSpeed() { return speed; }
  float NrtGetMaxAccel() { return accel; }
  float NrtGetMaxDecel() { return decel; }
  float NrtGetEstopDecel() { return stop; }
  float NrtGetMaxPos() { return upper; }
  float NrtGetMinPos() { return lower; }
  float NrtGetTorqueCoefficient() { return kt; }
};

TEST(PublicSdkParameters, UsesPhysicalUnitsWithoutDoubleConversion) {
  PublicJoint joint; PublicParameterReader<PublicJoint> reader(joint);
  DriveParameters p; std::string error;
  ASSERT_TRUE(read_drive_parameters(reader,p,error)) << error;
  EXPECT_DOUBLE_EQ(p.max_velocity,4);
  EXPECT_DOUBLE_EQ(p.max_acceleration,5);
  EXPECT_DOUBLE_EQ(p.max_deceleration,6);
  EXPECT_DOUBLE_EQ(p.stop_deceleration,7);
  EXPECT_DOUBLE_EQ(p.lower,-2); EXPECT_DOUBLE_EQ(p.upper,2);
  EXPECT_EQ(p.firmware,0x1185); EXPECT_EQ(p.protocol,0x1185);
  EXPECT_FALSE(p.checked_sdo_results);
  EXPECT_EQ(p.brake_status,2);  // Zero is ambiguous, not acknowledged closed.
  EXPECT_EQ(p.fault_code_result,kPublicValueUnverified);
}
TEST(PublicSdkParameters, RejectsAmbiguousRequiredZeroAndNonfiniteValues) {
  for (int field=0;field<3;++field) {
    PublicJoint joint;
    if (field==0) joint.sync=0;
    if (field==1) joint.watchdog=0;
    if (field==2) joint.speed=0;
    PublicParameterReader<PublicJoint> reader(joint);
    DriveParameters p; p.protocol=42; std::string error;
    EXPECT_FALSE(read_drive_parameters(reader,p,error));
    EXPECT_EQ(p.protocol,42);  // No partial snapshot accepted.
    EXPECT_FALSE(error.empty());
  }
}
TEST(PublicSdkParameters, DoesNotTreatZeroFaultAsVerifiedRecovery) {
  PublicJoint joint; PublicParameterReader<PublicJoint> reader(joint);
  uint16_t code=0xABCD;
  EXPECT_EQ(reader.u16(0,8,code),kPublicValueUnverified);
  EXPECT_EQ(code,0xABCD);
  joint.fault=CanEvoFault::kSyncError;
  EXPECT_EQ(reader.u16(0,8,code),0); EXPECT_EQ(code,0xFF05);
  EXPECT_EQ(reader.u16(0x21,0,code),int(CanEvoError::kNotSupported));
}
TEST(PublicSdkParameters, ValidatesVersionAndPreservesUnknownOptionalCoefficient) {
  uint16_t out=42;
  for (const auto *text:{"", "0.0.0", "16.0.1", "1.32.0", "1.1.128", "1.2.3x"}) {
    EXPECT_EQ(public_version(text,out),kPublicValueUnverified); EXPECT_EQ(out,42);
  }
  PublicJoint joint; joint.kt=0;
  PublicParameterReader<PublicJoint> reader(joint); DriveParameters p; std::string error;
  ASSERT_TRUE(read_drive_parameters(reader,p,error)); EXPECT_EQ(p.torque_constant,0);
}

std::array<AxisFeedback,kArms> checked_axes(int64_t now) {
  std::array<AxisFeedback,kArms> axes{};
  for (auto &a:axes) { a.valid=true; a.sdk_checked=true; a.checked_ns=now; }
  return axes;
}
TEST(PublicSdkFeedback, BoundsSdkCacheAndSnapshotDeliveryWithinTotalAge) {
  const auto c=Config::load(CONFIG_PATH); auto a=checked_axes(1000000000)[0];
  ASSERT_EQ(c.sdk_cache_timeout_ms(),35);
  EXPECT_TRUE(axis_feedback_fresh(a,c,1015000000));
  EXPECT_FALSE(axis_feedback_fresh(a,c,1015000001));
  EXPECT_FALSE(axis_feedback_fresh(a,c,999999999));
  a.pdo_result=-1; EXPECT_FALSE(axis_feedback_fresh(a,c,1000000000));
  a.pdo_result=0; a.checked_ns=0; EXPECT_FALSE(axis_feedback_fresh(a,c,1000000000));
}
TEST(PublicSdkCapabilities, FaultTraceRemainsEnabledWithoutChangingTimestampSocketMode) {
  const auto c=Config::load(CONFIG_PATH);
  EXPECT_TRUE(c.tx_trace_diagnostics);
  EXPECT_FALSE(c.tx_timestamp_diagnostics);
  EXPECT_EQ(c.period_us,5000);
}
TEST(PublicSdkFeedback, RequiresDataNewerThanEnableOrStopDespiteUnknownReceiveTime) {
  const auto c=Config::load(CONFIG_PATH); auto a=checked_axes(1035000000)[0];
  EXPECT_FALSE(axis_feedback_after(a,c,1000000000));
  ++a.checked_ns; EXPECT_TRUE(axis_feedback_after(a,c,1000000000));
  a.sdk_checked=false; a.received_ns=1000000000;
  EXPECT_FALSE(axis_feedback_after(a,c,1000000000));
  ++a.received_ns; EXPECT_TRUE(axis_feedback_after(a,c,1000000000));
}
TEST(PublicSdkFeedback, RepeatedFiveMsReadsDoNotBecomeThreeStandstillConfirmations) {
  const auto c=Config::load(CONFIG_PATH); FreshStandstillMonitor monitor(c,1000000000);
  for (int ms=5;ms<=135;ms+=5) {
    const int64_t now=1000000000LL+ms*1000000LL;
    EXPECT_FALSE(monitor.update(checked_axes(now),now)) << ms;
  }
  // Accepted observations at 40,95,150ms cannot refer to the same cached PDO.
  EXPECT_TRUE(monitor.update(checked_axes(1150000000),1150000000));
}
TEST(PublicSdkFeedback, MovingMissingOrExpiredAxisResetsStopConfirmation) {
  const auto c=Config::load(CONFIG_PATH);
  for (int fault=0;fault<3;++fault) {
    FreshStandstillMonitor monitor(c,0);
    EXPECT_FALSE(monitor.update(checked_axes(1000000000),1000000000));
    EXPECT_FALSE(monitor.update(checked_axes(1060000000),1060000000));
    auto bad=checked_axes(1120000000);
    if (fault==0) bad.back().velocity=1;
    if (fault==1) bad.back().valid=false;
    if (fault==2) bad.back().checked_ns=1100000000;
    EXPECT_FALSE(monitor.update(bad,1120000000));
    EXPECT_FALSE(monitor.update(checked_axes(1180000000),1180000000));
    EXPECT_FALSE(monitor.update(checked_axes(1240000000),1240000000));
    EXPECT_TRUE(monitor.update(checked_axes(1300000000),1300000000));
  }
}
TEST(PublicSdkCapabilities, UnsupportedExtensionsAndUnsafeBudgetsRefusedBeforeBusAccess) {
  auto c=Config::load(CONFIG_PATH);
  c.transport="sdk"; c.commissioning_confirmed=true; c.deactivate_policy="disable";
  EXPECT_NO_THROW(c.validate());
  c.feedback_snapshot_timeout_ms=49; EXPECT_THROW(c.validate(),std::invalid_argument);
  c.feedback_snapshot_timeout_ms=15;
  if (!sdk_extensions_available()) {
    c.stop_wait_ms=100; EXPECT_THROW(c.validate(),std::invalid_argument);
    c.stop_wait_ms=1000;
    c.tx_timestamp_diagnostics=true; EXPECT_THROW(make_transport(c),std::invalid_argument);
    c.tx_timestamp_diagnostics=false;
    // Even deliberately nonexistent bus names must produce a recovery refusal,
    // not an Open attempt or a reset request.
    c.buses={{"offline_no_can_a","offline_no_can_b"}};
    testing::internal::CaptureStderr();
    EXPECT_FALSE(clear_sdk_drive_faults(c));
    const auto message=testing::internal::GetCapturedStderr();
    EXPECT_NE(message.find("No CAN bus was opened"),std::string::npos);
    EXPECT_EQ(message.find("cannot open bus"),std::string::npos);
  }
}

class CheckedSnapshotTransport final : public Transport {
public:
  bool open(const Config &c) override {
    config=c; return true;
  }
  void close() noexcept override {}
  bool sample(int64_t now,Snapshot &out) override {
    out={}; out.sampled_ns=now; out.axes=checked_axes(now-delay_ns);
    for (std::size_t i=0;i<kArms;++i) out.axes[i].position=config.axes[i].initial;
    for (std::size_t i=0;i<kGrippers;++i) {
      out.grippers[i].valid=true; out.grippers[i].received_ns=now;
      out.grippers[i].raw=config.grippers[i].raw.front();
    }
    out.axes[0].pdo_result=result;
    out.axes[0].fault=fault;
    return true;
  }
  bool activate(const Positions &) override { return true; }
  bool send(const Positions &,const std::array<double,2>&,const std::array<bool,kGrippers>&) override { return true; }
  bool stop(const Positions &) noexcept override { ++stops; return true; }
  bool deactivate(const Positions &) override { return true; }
  Config config; int64_t delay_ns=0; int result=0,stops=0; bool fault=false;
};
TEST(PublicSdkFeedback, EngineLatchesStopOnStaleCheckedSnapshotSdkFailureOrDriveFault) {
  const auto c=Config::load(CONFIG_PATH);
  for (int fault=0;fault<3;++fault) {
    auto t=std::make_unique<CheckedSnapshotTransport>(); auto *raw=t.get();
    Engine engine(c,std::move(t)); ASSERT_TRUE(engine.configure());
    ASSERT_TRUE(engine.activate(1000000000));
    if (fault==0) raw->delay_ns=16000000;
    if (fault==1) raw->result=-1;
    if (fault==2) raw->fault=true;
    EXPECT_FALSE(engine.read(1005000000)); EXPECT_GT(raw->stops,0);
    EXPECT_EQ(engine.fault(),fault==2 ? Fault::drive : Fault::feedback);
    EXPECT_FALSE(engine.write()); EXPECT_FALSE(engine.activate(1010000000));
  }
}
}  // namespace

TEST(PublicSdkParameters, UnverifiedDriverPositionBoundsDoNotBlockConfiguration) {
  PublicJoint joint; joint.lower=0; joint.upper=std::numeric_limits<float>::quiet_NaN();
  PublicParameterReader<PublicJoint> reader(joint); DriveParameters p; std::string error;
  ASSERT_TRUE(read_drive_parameters(reader,p,error));
  EXPECT_FALSE(p.position_bounds_valid);
  EXPECT_EQ(p.lower_result,kPublicValueUnverified); EXPECT_EQ(p.upper_result,kPublicValueUnverified);
}
