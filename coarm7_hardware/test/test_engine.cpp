#include "coarm7_hardware/core.hpp"
#include "coarm7_hardware/gripper.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
using namespace coarm7_hardware;
struct Rig : testing::Test {
  Config c=Config::load(CONFIG_PATH);
  FakeTransport *fake=nullptr;
  std::unique_ptr<Engine> e;
  int64_t now=1000000000;
  void SetUp() override {
    auto f=std::make_unique<FakeTransport>(); fake=f.get();
    e=std::make_unique<Engine>(c,std::move(f));
    ASSERT_TRUE(e->configure()); ASSERT_TRUE(e->activate(now));
    std::array<bool,kCommands> claims; claims.fill(true); ASSERT_TRUE(e->set_claims(claims));
  }
  bool step(int64_t dt=5000000) { now+=dt; return e->read(now)&&e->write(); }
};
TEST_F(Rig, SeedFromFeedbackRatherThanZero) {
  EXPECT_DOUBLE_EQ(e->commands()[1],-0.5); EXPECT_DOUBLE_EQ(e->commands()[10],1.0);
}
TEST_F(Rig, SingleSyncPerBusPerRead) {
  ASSERT_TRUE(step()); EXPECT_EQ(fake->sync_count[0],2); EXPECT_EQ(fake->sync_count[1],2);
}
TEST_F(Rig, BothArmPositionAndCoordinateTransform) {
  e->commands()[0]=0.1; e->commands()[7]=-0.1;
  for(int i=0;i<100;++i) ASSERT_TRUE(step());
  EXPECT_NEAR(e->positions()[0],0.1,1e-6); EXPECT_NEAR(e->positions()[7],-0.1,1e-6);
}
TEST_F(Rig, VelocityAndAccelerationGuard) {
  e->commands()[0]=1.0; double previous=0,velocity=0;
  for(int i=0;i<100;++i) {
    ASSERT_TRUE(step()); const auto p=fake->last_target[0]; const auto v=(p-previous)/0.005;
    EXPECT_LE(std::abs(v),c.axes[0].max_velocity+1e-9);
    EXPECT_LE(std::abs(v-velocity),c.axes[0].max_acceleration*0.005+1e-9);
    previous=p; velocity=v;
  }
}
TEST_F(Rig, ReversingTargetBrakesWithoutSkippingVelocityOrAccelerationLimits) {
  constexpr double dt = 0.005;
  const auto axis = c.axes[0];
  e->commands()[0] = 0.4;
  for (int i=0;i<30;++i) ASSERT_TRUE(step());
  double p = fake->last_target[0];
  ASSERT_TRUE(step());
  double velocity = (fake->last_target[0]-p)/dt;
  ASSERT_GT(velocity,0.2);
  p = fake->last_target[0];
  e->commands()[0] = -0.05;
  for (int i=0;i<160;++i) {
    ASSERT_TRUE(step());
    const double next = fake->last_target[0];
    const double v = (next-p)/dt;
    EXPECT_LE(std::abs(v),axis.max_velocity+1e-9);
    EXPECT_LE(std::abs(v-velocity),axis.max_acceleration*dt+1e-9);
    p=next;
    velocity=v;
  }
  EXPECT_NEAR(p,-0.05,1e-8);
  EXPECT_NEAR(velocity,0,1e-8);
}
TEST_F(Rig, NonfiniteCommandStopsBothArmsAndLatches) {
  e->commands()[8]=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(e->write());
  EXPECT_EQ(e->fault(),Fault::command); EXPECT_GT(fake->stops,0); EXPECT_FALSE(e->active());
  e->commands()[8]=0; EXPECT_FALSE(e->write()); EXPECT_FALSE(e->activate(now));
}
TEST_F(Rig, OutOfRangeCommandStopsBeforeAnyNewTarget) {
  const auto before=fake->last_target;
  e->commands()[13]=100; EXPECT_FALSE(e->write()); EXPECT_EQ(fake->last_target,before);
}
TEST_F(Rig, JointFeedbackTimeExpiry) {
  fake->stale_axis=0;
  for(int i=0;i<10;++i) ASSERT_TRUE(step());
  EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::feedback);
}
TEST_F(Rig, GripperHasIndependentLongerFeedbackExpiry) {
  fake->stale_gripper=1;
  for(int i=0;i<100;++i) ASSERT_TRUE(step());
  EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::gripper);
}
TEST_F(Rig, AsyncFeedbackDuringReadIsNotAFutureTimestampFault) {
  fake->sample_delay_ns=1000000;
  ASSERT_TRUE(step()); EXPECT_EQ(e->fault(),Fault::none);
}
TEST_F(Rig, ServoFaultStopsBothArms) { fake->fault_axis=7; EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::drive); }
TEST_F(Rig, CspModeLossStopsBothArms) { fake->wrong_mode_axis=0; EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::mode); }
TEST_F(Rig, BusFailureStopsBothArms) { fake->fail_sample=true; EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::transport); }
TEST_F(Rig, SendFailureStopsBothArms) {
  fake->fail_send=true; EXPECT_FALSE(step()); EXPECT_GT(fake->stops,0);
  EXPECT_FALSE(e->stop_delivery_ok());
  EXPECT_FALSE(e->deactivate());  // Stop delivery cannot be reported as success.
}
TEST_F(Rig, NormalDeactivateReactivateDoesNotLatchFaultStopOrReplayCommands) {
  e->commands()[0]=0.2; ASSERT_TRUE(step()); ASSERT_TRUE(step());
  const int stops=fake->stops;
  ASSERT_TRUE(e->deactivate()); ASSERT_TRUE(e->deactivate());
  EXPECT_EQ(fake->deactivations,1); EXPECT_EQ(fake->stops,stops);
  ASSERT_TRUE(e->activate(now+5000000));
  EXPECT_EQ(e->commands()[0],e->positions()[0]); EXPECT_EQ(e->fault(),Fault::none);
}
TEST_F(Rig, CleanupAfterNormalDisableDoesNotIssueAnExtraQuickStop) {
  ASSERT_TRUE(e->deactivate()); const int stops=fake->stops;
  e->cleanup(); EXPECT_EQ(fake->stops,stops); EXPECT_EQ(fake->deactivations,1);
}
TEST_F(Rig, FailedDeactivateLatchesAndPreventsReactivation) {
  fake->fail_send=true; EXPECT_FALSE(e->deactivate());
  EXPECT_EQ(e->fault(),Fault::transport); EXPECT_FALSE(e->stop_delivery_ok());
  fake->fail_send=false; EXPECT_FALSE(e->activate(now+5000000));
}
TEST_F(Rig, GripperDeviceFaultStopsWholeComponent) { fake->bad_gripper=0; EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::gripper); }
TEST_F(Rig, GripperCommandRejectionStopsWholeComponentAndLatches) {
  fake->feedback.grippers[0].command_result=-2;
  EXPECT_FALSE(step());EXPECT_EQ(e->fault(),Fault::gripper);EXPECT_GT(fake->stops,0);
  EXPECT_NE(e->fault_detail().find("command_result=-2"),std::string::npos);
  fake->feedback.grippers[0].command_result=0;EXPECT_FALSE(e->activate(now+5000000));
}
TEST_F(Rig, TrackingErrorDetected) { fake->last_target[0]=0.6; EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::tracking); }
TEST_F(Rig, ConsecutiveDeadlinePolicy) {
  ASSERT_TRUE(step());  // First read establishes the post-activation timestamp.
  ASSERT_TRUE(step(101000000)); ASSERT_TRUE(step(101000000)); EXPECT_FALSE(step(101000000));
  EXPECT_EQ(e->fault(),Fault::deadline);
}
TEST_F(Rig, DeadlineCounterResetsOnTimelyCycle) {
  ASSERT_TRUE(step());
  ASSERT_TRUE(step(101000000)); ASSERT_TRUE(step()); ASSERT_TRUE(step(101000000)); EXPECT_EQ(e->fault(),Fault::none);
}
TEST_F(Rig, StoppedControllerCannotReplayOldCommand) {
  e->commands()[0]=0.2; ASSERT_TRUE(step()); ASSERT_TRUE(step());
  std::array<bool,kCommands> claims{}; ASSERT_TRUE(e->set_claims(claims));
  const auto held=e->positions()[0]; e->commands()[0]=0.8;
  ASSERT_TRUE(step()); EXPECT_DOUBLE_EQ(fake->last_target[0],held);
  // One target was already in flight when claims were released. Reactivation
  // must seed the freshest measured state rather than replay either old goal.
  claims[0]=true; ASSERT_TRUE(e->set_claims(claims)); EXPECT_DOUBLE_EQ(e->commands()[0],e->positions()[0]);
}
TEST_F(Rig, CleanupConfigureRestartsOnlyAtMeasuredState) {
  ASSERT_TRUE(e->deactivate()); e->cleanup(); EXPECT_FALSE(fake->opened);
  ASSERT_TRUE(e->configure()); ASSERT_TRUE(e->activate(now+5000000)); EXPECT_DOUBLE_EQ(e->commands()[1],-0.5);
}
namespace {
class StartupTransport final : public Transport {
public:
  bool open(const Config &) override { return true; }
  void close() noexcept override { ++closes; }
  bool startup_feedback(Snapshot &out) const override { out=initial; return true; }
  bool sample(int64_t,Snapshot &) override { ++samples; return false; }
  bool activate(const Positions &) override { ++activations; return false; }
  bool send(const Positions &,const std::array<double,2> &,
            const std::array<bool,kGrippers> &) override { ++sends; return false; }
  bool stop(const Positions &) noexcept override { ++stops; return false; }
  bool deactivate(const Positions &) override { return false; }
  Snapshot initial{};
  int closes=0, samples=0, activations=0, sends=0, stops=0;
};
}
TEST(Engine, StartupRejectsDriveFaultWithDiagnosticBeforeEnabling) {
  auto c=Config::load(CONFIG_PATH);
  auto transport=std::make_unique<StartupTransport>();
  auto *stub=transport.get();
  const auto now=monotonic_now_ns();
  auto &first=stub->initial.axes[0];
  first.valid=true; first.received_ns=now; first.fault=true;
  first.statusword=0x1007; first.emcy_valid=true; first.emcy=0x3220;
  Engine engine(c,std::move(transport));
  ASSERT_FALSE(engine.configure());
  EXPECT_EQ(engine.fault(),Fault::drive);
  EXPECT_NE(engine.fault_detail().find("joint=L_joint1 bus=can0 node_id=1"),std::string::npos);
  EXPECT_NE(engine.fault_detail().find("statusword=0x1007"),std::string::npos);
  EXPECT_NE(engine.fault_detail().find("emcy=0x3220"),std::string::npos);
  EXPECT_FALSE(engine.activate(now));
  EXPECT_EQ(stub->closes,1);
  EXPECT_EQ(stub->samples+stub->activations+stub->sends+stub->stops,0);
}
TEST(Engine, StartupDistinguishesUnavailablePdoFromOutOfBoundsPosition) {
  auto c=Config::load(CONFIG_PATH);
  auto transport=std::make_unique<StartupTransport>();
  auto *stub=transport.get();
  stub->initial.axes[0].pdo_result=-10;
  Engine engine(c,std::move(transport));
  ASSERT_FALSE(engine.configure());
  EXPECT_NE(engine.fault_detail().find("PDO snapshot unavailable"),std::string::npos);
  EXPECT_NE(engine.fault_detail().find("pdo_result=-10"),std::string::npos);

  auto &first=stub->initial.axes[0];
  first.valid=true; first.received_ns=monotonic_now_ns(); first.position=c.axes[0].upper+0.1;
  first.pdo_result=0;
  ASSERT_FALSE(engine.configure());
  EXPECT_EQ(engine.fault(),Fault::feedback);
  EXPECT_NE(engine.fault_detail().find("joint position outside effective bounds"),std::string::npos);
  EXPECT_NE(engine.fault_detail().find("effective_bounds_rad="),std::string::npos);
  EXPECT_EQ(stub->samples+stub->activations+stub->sends+stub->stops,0);
}
TEST(Engine, StartupGripperRejectionReportsSideAndCauseWithoutEnabling) {
  auto c=Config::load(CONFIG_PATH);
  auto transport=std::make_unique<StartupTransport>();
  auto *stub=transport.get();
  const auto now=monotonic_now_ns();
  for (std::size_t i=0; i<kArms; ++i) {
    auto &axis=stub->initial.axes[i];
    axis.valid=true;
    axis.received_ns=now;
    axis.position=(c.axes[i].initial-c.axes[i].offset)/c.axes[i].sign;
  }
  for (std::size_t i=0; i<kGrippers; ++i) {
    auto &gripper=stub->initial.grippers[i];
    gripper.valid=true;
    gripper.received_ns=now;
    gripper.raw=c.grippers[i].raw.front();
  }
  auto &left=stub->initial.grippers[0];
  auto &right=stub->initial.grippers[1];
  left.status=0x02;
  left.fault=0x04;
  Engine engine(c,std::move(transport));

  ASSERT_FALSE(engine.configure());
  EXPECT_EQ(engine.fault(),Fault::gripper);
  auto detail=engine.fault_detail();
  EXPECT_NE(detail.find("gripper device fault gripper="+c.grippers[0].name),std::string::npos);
  EXPECT_NE(detail.find("status=0x2 gripper_fault=0x4"),std::string::npos);

  left.fault=0;
  right.raw=2000;
  ASSERT_FALSE(engine.configure());
  detail=engine.fault_detail();
  EXPECT_NE(detail.find("gripper opening outside calibration gripper="+c.grippers[1].name),
            std::string::npos);
  EXPECT_NE(detail.find("opening_raw=2000 opening_allowed_raw="),std::string::npos);

  right.raw=c.grippers[1].raw.front();
  right.received_ns=monotonic_now_ns()-(c.gripper_timeout_ms+50)*1000000LL;
  ASSERT_FALSE(engine.configure());
  detail=engine.fault_detail();
  EXPECT_NE(detail.find("gripper feedback expired gripper="+c.grippers[1].name),std::string::npos);
  EXPECT_NE(detail.find("feedback_age_ms="),std::string::npos);
  EXPECT_NE(detail.find("feedback_timeout_ms="+std::to_string(c.gripper_timeout_ms)),std::string::npos);
  EXPECT_FALSE(engine.activate(now));
  EXPECT_EQ(stub->closes,3);
  EXPECT_EQ(stub->samples+stub->activations+stub->sends+stub->stops,0);
}
TEST(Engine, FeedbackFaultBeforeEnableDoesNotSendMotionOrStop) {
  auto c = Config::load(CONFIG_PATH);
  auto transport = std::make_unique<FakeTransport>();
  auto *fake = transport.get();
  Engine engine(c, std::move(transport));
  ASSERT_TRUE(engine.configure());
  fake->fault_axis = 0;
  EXPECT_FALSE(engine.read(1000000000));
  EXPECT_FALSE(engine.read(1005000000));
  EXPECT_FALSE(engine.activate(1010000000));
  engine.cleanup();
  EXPECT_EQ(fake->stops, 0);
  EXPECT_EQ(fake->deactivations, 0);
}
TEST(Config, RejectsSdkBeforeOpeningAnyBus) {
  auto c=Config::load(CONFIG_PATH); c.transport="sdk";
  EXPECT_THROW(make_transport(c),std::invalid_argument);
  c.commissioning_confirmed=true; EXPECT_THROW(make_transport(c),std::invalid_argument);
}
TEST(Config, RejectsWrongPeriodDuplicateNodeAndMalformedCalibration) {
  auto c=Config::load(CONFIG_PATH); c.period_us=10000; EXPECT_THROW(c.validate(),std::invalid_argument);
  c.period_us=5000; c.axes[1].id=c.axes[0].id; EXPECT_THROW(c.validate(),std::invalid_argument);
  c=Config::load(CONFIG_PATH); c.grippers[0].raw={10,1000}; EXPECT_THROW(c.validate(),std::invalid_argument);
}
TEST(Config, CoordinateOffsetAndDirectionAreConfigurable) {
  auto c=Config::load(CONFIG_PATH); c.axes[0].sign=-1; c.axes[0].offset=0.2; c.validate();
  auto f=std::make_unique<FakeTransport>(); auto *p=f.get(); Engine e(c,std::move(f));
  ASSERT_TRUE(e.configure()); ASSERT_TRUE(e.activate(1000000000)); EXPECT_NEAR(p->feedback.axes[0].position,0.2,1e-9);
  EXPECT_NEAR(e.positions()[0],0,1e-9);
}
TEST(Gripper, ConvertsRawOpeningWithoutPercentFactorError) {
  const auto g=Config::load(CONFIG_PATH).grippers[0];
  EXPECT_DOUBLE_EQ(to_raw(g,0),1000); EXPECT_DOUBLE_EQ(to_raw(g,0.864),10);
  EXPECT_NEAR(to_raw(g,0.85536),10,1e-9);
  EXPECT_NEAR(to_raw(g,0.432),500,1e-9);
  EXPECT_NEAR(to_angle(g,500),0.432,1e-12); EXPECT_DOUBLE_EQ(to_angle(g,1003),0);
  EXPECT_NEAR(to_angle(g,10),0.85536,1e-12);
  EXPECT_DOUBLE_EQ(to_angle(g,0),0.864);
  EXPECT_THROW(to_angle(g,1200),std::out_of_range);
  EXPECT_THROW(to_raw(g,std::numeric_limits<double>::quiet_NaN()),std::out_of_range);
}
TEST(Gripper, PiecewiseCalibrationInterpolation) {
  auto g=Config::load(CONFIG_PATH).grippers[0]; g.angle={0,0.3,0.864};g.raw={1000,700,10};
  EXPECT_NEAR(to_raw(g,0.15),850,1e-9); EXPECT_NEAR(to_angle(g,850),0.15,1e-9);
}
static SerialFrame response(uint8_t cmd,const std::vector<uint8_t> &data) {
  auto f=gripper_request(1,cmd,data.data(),data.size()); f.data[0]=0xEE;f.data[1]=0x16; return f;
}
TEST(Protocol, RequestChecksumAndLittleEndian) {
  uint8_t p[2]={0xF4,0x01};auto f=gripper_request(1,0x54,p,2);
  EXPECT_EQ(f.size,8u);EXPECT_EQ(f.data[5],0xF4);EXPECT_EQ(f.data[6],1);
  EXPECT_EQ(f.data[7],static_cast<uint8_t>(1+3+0x54+0xF4+1));
  EXPECT_EQ(gripper_request(0,0x41).size,0u);
}
TEST(Protocol, RejectsBadChecksumLengthAndAddress) {
  auto f=response(0x41,{0,0,20,0xEB,3,0,0}); SerialFrame payload;uint8_t cmd=0;
  ASSERT_TRUE(gripper_response(f.data.data(),f.size,1,cmd,payload));
  EXPECT_FALSE(gripper_response(f.data.data(),f.size,2,cmd,payload));
  EXPECT_FALSE(gripper_response(f.data.data(),f.size-1,1,cmd,payload));
  f.data[f.size-1]^=1;EXPECT_FALSE(gripper_response(f.data.data(),f.size,1,cmd,payload));
}
TEST(Protocol, PollFeedbackAndLatestTargetCoalescing) {
  auto c=Config::load(CONFIG_PATH);GripperProtocol p(c,0);int64_t now=1000000000;
  EXPECT_EQ(p.next(now).data[4],0x41);
  auto f=response(0x41,{0,0,20,0xEB,3,0,0});p.received(f.data.data(),f.size,now+1000000);
  EXPECT_TRUE(p.feedback().valid);EXPECT_EQ(p.feedback().raw,1003);
  p.target(300);p.target(600);auto command=p.next(now+5000000);
  EXPECT_EQ(command.data[4],0x54);EXPECT_EQ(command.data[5],600&255);EXPECT_EQ(command.data[6],600>>8);
}
TEST(Protocol, StopDiscardsPendingOpeningPermanently) {
  auto c=Config::load(CONFIG_PATH);GripperProtocol p(c,0);p.target(300);
  EXPECT_EQ(p.stop().data[4],0x16);p.target(600);
  auto f=p.next(1000000000);EXPECT_NE(f.data[4],0x54);
}
TEST(Protocol, ReadOnlySdkPollsBothGrippersThroughoutTheLifecycle) {
  auto c=Config::load(CONFIG_PATH);
  c.transport="sdk";
  c.gripper_read_only=true;
  for (std::size_t i=0; i<kGrippers; ++i) {
    GripperProtocol p(c,i);
    int64_t now=1000000000;
    const auto feedback=response(0x41,{0,0,20,0xEB,3,0,0});
    const auto poll=[&]() {
      const auto frame=p.next(now);
      ASSERT_GT(frame.size,0u);
      EXPECT_EQ(frame.data[4],0x41);
      p.received(feedback.data.data(),feedback.size,now+1000000);
      EXPECT_TRUE(p.feedback().valid);
      EXPECT_EQ(p.feedback().raw,1003);
      now+=c.gripper_poll_ms*1000000LL;
    };
    poll();  // Configure.
    p.resume();
    p.target(1000);  // First hold would clamp the measured 1003.
    poll();
    p.target(10);  // Even an explicit target must never queue an opening frame.
    for (int tick=0; tick<5; ++tick) {
      EXPECT_EQ(p.next(now-c.gripper_poll_ms*1000000LL+(tick+1)*5000000LL).size,0u);
    }
    poll();
    EXPECT_EQ(p.stop().size,0u);
    poll();  // Feedback continues during arm deactivation.
    p.resume();
    poll();  // Reactivation does not replay old commands.
  }
}
TEST(Protocol, ReadOnlyOptionPreservesFakeAndExplicitSdkCommands) {
  auto c=Config::load(CONFIG_PATH);
  c.gripper_read_only=true;
  EXPECT_TRUE(c.gripper_commands_enabled());
  c.transport="sdk";
  c.gripper_read_only=false;
  GripperProtocol p(c,0);
  const int64_t now=1000000000;
  ASSERT_EQ(p.next(now).data[4],0x41);
  const auto feedback=response(0x41,{0,0,20,0xEB,3,0,0});
  p.received(feedback.data.data(),feedback.size,now+1000000);
  p.target(500);
  EXPECT_EQ(p.next(now+5000000).data[4],0x51);
  auto ack=response(0x51,{1});
  p.received(ack.data.data(),ack.size,now+6000000);
  EXPECT_EQ(p.next(now+10000000).data[4],0xD2);
  auto speed=response(0xD2,{0xF4,1});
  p.received(speed.data.data(),speed.size,now+11000000);
  EXPECT_EQ(p.next(now+15000000).data[4],0x54);
  EXPECT_EQ(p.stop().data[4],0x16);
}
TEST(Protocol, DuplicateReplyDoesNotRefreshTimestamp) {
  auto c=Config::load(CONFIG_PATH);GripperProtocol p(c,0);auto f=response(0x41,{0,0,20,0xE8,3,0,0});
  p.received(f.data.data(),f.size,100);p.received(f.data.data(),f.size,99);EXPECT_EQ(p.feedback().received_ns,100);
}
TEST(Gripper, CommandLimitsAreIndependentOfFeedbackCalibration) {
  auto c=Config::load(CONFIG_PATH);
  auto &g=c.grippers[0];
  g.command_raw_min=100; g.command_raw_max=900; c.validate();
  EXPECT_DOUBLE_EQ(to_raw(g,0),900);
  EXPECT_DOUBLE_EQ(to_raw(g,0.864),100);
  EXPECT_DOUBLE_EQ(to_angle(g,1000),0);
  EXPECT_DOUBLE_EQ(to_angle(g,0),0.864);
  g.command_raw_min=1001; EXPECT_THROW(c.validate(),std::invalid_argument);
  g.command_raw_min=10.5; EXPECT_THROW(c.validate(),std::invalid_argument);
}
TEST_F(Rig, OnlyClaimedGrippersForwardTargetsAndUnclaimDiscardsTheOldGoal) {
  std::array<bool,kCommands> claims{};
  claims[0]=true;
  ASSERT_TRUE(e->set_claims(claims));
  e->commands()[kArms]=0.7;
  ASSERT_TRUE(step());
  EXPECT_FALSE(fake->last_gripper_claims[0]);
  EXPECT_FALSE(fake->last_gripper_claims[1]);
  EXPECT_DOUBLE_EQ(fake->feedback.grippers[0].raw,1000);
  claims[kArms]=true;
  ASSERT_TRUE(e->set_claims(claims));
  EXPECT_DOUBLE_EQ(e->commands()[kArms],e->positions()[kArms]);
  e->commands()[kArms]=0.7; e->gripper_goal_ids()[0]=1;
  for (int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_TRUE(fake->last_gripper_claims[0]);
  EXPECT_FALSE(fake->last_gripper_claims[1]);
  claims[kArms]=false;
  ASSERT_TRUE(e->set_claims(claims));
  ASSERT_TRUE(step());
  const auto held=fake->feedback.grippers[0].raw;
  e->commands()[kArms]=0.8;
  for (int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(fake->feedback.grippers[0].raw,held);
  claims[kArms]=true;
  ASSERT_TRUE(e->set_claims(claims));
  EXPECT_DOUBLE_EQ(e->commands()[kArms],e->positions()[kArms]);
}
TEST(Protocol, PollOnlyStageNeverSendsAnOpeningAndChecksTheRawRange) {
  const auto c=Config::load(CONFIG_PATH);
  for (std::size_t i=0;i<kGrippers;++i) {
    GripperProtocol p(c,i);
    const int64_t now=1000000000;
    ASSERT_EQ(p.next(now,false).data[4],0x41);
    const auto feedback=response(0x41,{3,0,20,0xF4,1,0,0});
    p.received(feedback.data.data(),feedback.size,now+1000000);
    p.resume();
    ASSERT_TRUE(p.target(500));
    EXPECT_EQ(p.next(now+5000000,true).size,0u); // Measured hold.
    EXPECT_FALSE(p.target(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(p.target(0)); EXPECT_FALSE(p.target(1001));
    ASSERT_TRUE(p.target(600));
    EXPECT_EQ(p.next(now+5000000,false).size,0u);
    const auto opening=p.next(now+5000000,true);
    ASSERT_EQ(opening.data[4],0x54);
    EXPECT_EQ(opening.data[5],600&255); EXPECT_EQ(opening.data[6],600>>8);
    ASSERT_EQ(p.stop().data[4],0x16);
    EXPECT_FALSE(p.target(700));
    for (int tick=2;tick<60;++tick) EXPECT_NE(p.next(now+tick*5000000LL,true).data[4],0x54);
    p.resume();
    EXPECT_NE(p.next(now+300000000,true).data[4],0x54);
  }
}
TEST(Protocol, ActivationAtSlightlyOverOpenFeedbackDoesNotWriteAnOpening) {
  GripperProtocol p(Config::load(CONFIG_PATH),0);
  const int64_t now=1000000000;
  ASSERT_EQ(p.next(now,false).data[4],0x41);
  const auto feedback=response(0x41,{3,0,20,0xEB,3,0,0});
  p.received(feedback.data.data(),feedback.size,now+1000000);
  p.resume();
  ASSERT_TRUE(p.target(1000));
  EXPECT_EQ(p.next(now+5000000,true).size,0u);
}
TEST(Protocol, SlowQueryReplyReservesTheGuardedWriteTurn) {
  for (int delay_ms : {50,75,150,200}) {
    GripperProtocol p(Config::load(CONFIG_PATH),0);
    const int64_t start=1000000000;
    ASSERT_EQ(p.next(start,false).data[4],0x41);
    ASSERT_TRUE(p.target(500));
    const auto feedback=response(0x41,{3,0,20,0xE3,3,0,0});
    const auto now=start+delay_ms*1000000LL;
    p.received(feedback.data.data(),feedback.size,now);
    EXPECT_EQ(p.next(now,false).size,0u) << delay_ms;
    const auto command=p.next(now+100000,true);
    ASSERT_EQ(command.size,8u) << delay_ms;
    EXPECT_EQ(command.data[4],0x54) << delay_ms;
    EXPECT_EQ(command.data[5],500&255);
    EXPECT_EQ(command.data[6],500>>8);
    const auto ack=response(0x54,{1});
    p.received(ack.data.data(),ack.size,now+1000000);
    EXPECT_EQ(p.diagnostics().feedback_replies,1u);
    EXPECT_EQ(p.diagnostics().opening_replies,1u);
    EXPECT_EQ(p.diagnostics().max_query_reply_ns,delay_ms*1000000LL);
    EXPECT_EQ(p.diagnostics().last_opening_reply,1);
    EXPECT_EQ(p.next(now+10000000,false).data[4],0x41);
  }
}
TEST(Protocol, MalformedStateCannotReleasePendingQueryAndTimeoutsAreRecorded) {
  GripperProtocol p(Config::load(CONFIG_PATH),0);
  const int64_t now=1000000000;
  ASSERT_EQ(p.next(now,false).data[4],0x41);
  ASSERT_TRUE(p.target(500));
  const auto malformed=response(0x41,{3});
  p.received(malformed.data.data(),malformed.size,now+1000000);
  EXPECT_EQ(p.next(now+5000000,true).size,0u);
  EXPECT_EQ(p.diagnostics().rejected_replies,1u);
  EXPECT_FALSE(p.feedback().valid);
  const auto opening=p.next(now+250000000,true);
  ASSERT_EQ(opening.data[4],0x54);
  EXPECT_EQ(p.diagnostics().query_timeouts,1u);
  EXPECT_EQ(p.next(now+500000000,false).data[4],0x41);
  EXPECT_EQ(p.diagnostics().opening_timeouts,1u);
}
TEST(Protocol, SlowFeedbackAndContinuousTargetsCannotStarveEitherDirection) {
  for (int delay_ms : {25,50,75,150}) {
    GripperProtocol p(Config::load(CONFIG_PATH),0);
    SerialFrame pending_reply;
    int64_t due=0;
    int polls=0,writes=0;
    unsigned measured=995;
    auto transmit=[&](const SerialFrame &frame,int64_t now) {
      if (!frame.size) return;
      if (frame.data[4]==0x41) {
        ++polls;
        pending_reply=response(0x41,{3,0,20,uint8_t(measured),uint8_t(measured>>8),0,0});
        due=now+delay_ms*1000000LL;
      } else {
        ASSERT_EQ(frame.data[4],0x54);
        ++writes;
        measured=unsigned(frame.data[5])|(unsigned(frame.data[6])<<8);
        pending_reply=response(0x54,{1});
        due=now+1000000;
      }
    };
    for (int tick=0;tick<400;++tick) {
      const int64_t now=1000000000LL+tick*5000000LL;
      if (due && now>=due) {
        p.received(pending_reply.data.data(),pending_reply.size,due);
        due=0;
      }
      transmit(p.next(now,false),now);
      ASSERT_TRUE(p.target(500+tick/2));
      transmit(p.next(now+100000,true),now+100000);
    }
    EXPECT_GT(polls,5) << delay_ms;
    EXPECT_GT(writes,5) << delay_ms;
    EXPECT_TRUE(p.feedback().valid);
  }
}

TEST_F(Rig, EndpointSubmitsFinalTargetOnceAndDoesNotStreamInterpolatedOpenings) {
  e->commands()[kArms]=0.8; e->gripper_goal_ids()[0]=1;
  ASSERT_TRUE(step());
  EXPECT_NEAR(fake->last_target[kArms],0.8,1e-12);
  EXPECT_NEAR(e->positions()[kArms],0,1e-12);
  for(int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_GT(e->positions()[kArms],0.05);
  EXPECT_LT(e->positions()[kArms],0.2);
  // Changes without a new action transaction must not retarget the device.
  e->commands()[kArms]=0.4;
  ASSERT_TRUE(step()); EXPECT_NEAR(fake->last_target[kArms],0.8,1e-12);
}
TEST_F(Rig, StopDiscardsEvenAnUnconsumedEndpointAndRequiresAFreshGoal) {
  e->commands()[kArms]=0.8; e->gripper_goal_ids()[0]=1; e->gripper_stop_ids()[0]=1;
  ASSERT_TRUE(step()); ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(e->gripper_stop_acks()[0],1);
  EXPECT_DOUBLE_EQ(e->positions()[kArms],0);
  for(int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(e->positions()[kArms],0);
  e->gripper_goal_ids()[0]=2;
  ASSERT_TRUE(step()); ASSERT_TRUE(step()); EXPECT_GT(e->positions()[kArms],0);
  e->gripper_stop_ids()[0]=2; ASSERT_TRUE(step()); ASSERT_TRUE(step());
  const double stopped=e->positions()[kArms];
  for(int i=0;i<30;++i) ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(e->positions()[kArms],stopped);
  EXPECT_DOUBLE_EQ(e->gripper_stop_acks()[0],2);
}
TEST_F(Rig, InvalidEndpointSequenceStopsTheComponentBeforeAnySend) {
  e->gripper_goal_ids()[0]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(step()); EXPECT_EQ(e->fault(),Fault::command); EXPECT_GT(fake->stops,0);
}
TEST_F(Rig, EndpointUnclaimAndHardwareDeactivatePreventOldGoalReplay) {
  e->commands()[kArms]=0.8; e->gripper_goal_ids()[0]=1;
  for(int i=0;i<20;++i) ASSERT_TRUE(step());
  std::array<bool,kCommands> claims{}; ASSERT_TRUE(e->set_claims(claims)); ASSERT_TRUE(step());
  const auto q=e->positions()[kArms];
  for(int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(e->positions()[kArms],q);
  claims[kArms]=true; ASSERT_TRUE(e->set_claims(claims));
  for(int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(e->positions()[kArms],q);
  e->commands()[kArms]=0.8; e->gripper_goal_ids()[0]=2;
  for(int i=0;i<10;++i) ASSERT_TRUE(step());
  ASSERT_TRUE(e->deactivate()); ASSERT_TRUE(e->activate(now+5000000)); now+=5000000;
  const auto held=e->positions()[kArms];
  for(int i=0;i<20;++i) ASSERT_TRUE(step());
  EXPECT_DOUBLE_EQ(e->positions()[kArms],held);
}
TEST(Protocol, OneEndpointWriteThenFeedbackPollingWithoutRepeatedMotionWrites) {
  const auto c=Config::load(CONFIG_PATH); GripperProtocol protocol(c,0);
  int openings=0,polls=0;
  ASSERT_TRUE(protocol.target(10));
  for(int tick=0;tick<400;++tick) {
    const auto now=1000000000LL+tick*5000000LL;
    const auto frame=protocol.next(now,true);
    if(!frame.size) continue;
    SerialFrame reply;
    if(frame.data[4]==0x41) { ++polls; reply=response(0x41,{3,0,20,0xE8,3,0,0}); }
    else { ASSERT_EQ(frame.data[4],0x54); ++openings; EXPECT_EQ(frame.data[5],10); reply=response(0x54,{1}); }
    protocol.received(reply.data.data(),reply.size,now+1000000);
  }
  EXPECT_EQ(openings,1); EXPECT_GT(polls,20);
}


TEST(ConfigPortability, DefaultCpuSelectionIsUnboundAndPeriodUnchanged) {
  Config defaults;
  EXPECT_EQ(defaults.rx_cpu,-1); EXPECT_EQ(defaults.tx_cpu,-1);
  const auto config=Config::load(CONFIG_PATH);
  EXPECT_EQ(config.rx_cpu,-1); EXPECT_EQ(config.tx_cpu,-1);
  EXPECT_EQ(config.rx_priority,90); EXPECT_EQ(config.tx_priority,90);
  EXPECT_EQ(config.period_us,5000);
}


TEST_F(Rig, FirstRejectedTargetKeepsJointBoundsAndOriginalValue) {
  const auto before=fake->last_target;
  const auto axis=c.axes[4];
  e->commands()[4]=axis.upper+0.125;
  EXPECT_FALSE(e->write());
  EXPECT_EQ(fake->last_target,before);
  const auto first=e->fault_detail();
  EXPECT_NE(first.find("joint=L_joint5"),std::string::npos);
  EXPECT_NE(first.find("target_rad="),std::string::npos);
  EXPECT_NE(first.find("effective_bounds_rad=["),std::string::npos);
  EXPECT_NE(first.find("feedback_rad="),std::string::npos);
  EXPECT_NE(first.find("host_mono_ns="),std::string::npos);
  e->commands()[4]=0; e->commands()[0]=100;
  EXPECT_FALSE(e->write()); EXPECT_FALSE(e->read(now+5000000));
  EXPECT_EQ(e->fault_detail(),first);
}
TEST_F(Rig, NonfiniteTargetRetainsSpecificJointAndCause) {
  e->commands()[8]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(e->write());
  EXPECT_NE(e->fault_detail().find("nonfinite position target joint=R_joint2"),std::string::npos);
  EXPECT_NE(e->fault_detail().find("target_rad=nan"),std::string::npos);
}
TEST_F(Rig, ExactBoundaryIsNotSilentlyWidenedOrClamped) {
  e->commands()[0]=c.axes[0].upper;
  EXPECT_TRUE(e->write());
  e->commands()[0]=std::nextafter(c.axes[0].upper,std::numeric_limits<double>::infinity());
  EXPECT_FALSE(e->write());
  EXPECT_EQ(e->fault(),Fault::command);
  EXPECT_NE(e->fault_detail().find("joint=L_joint1"),std::string::npos);
}
