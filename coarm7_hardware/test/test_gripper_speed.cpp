#include "coarm7_hardware/gripper.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
using namespace coarm7_hardware;
namespace {
SerialFrame reply(uint8_t command,std::vector<uint8_t> data) {
  auto f=gripper_request(1,command,data.data(),data.size());f.data[0]=0xEE;f.data[1]=0x16;return f;
}
Config sdk_config() {
  auto c=Config::load(CONFIG_PATH);c.transport="sdk";c.gripper_read_only=false;return c;
}
void receive(GripperProtocol &p,const SerialFrame &f,int64_t now) {
  p.received(f.data.data(),f.size,now);
}
void seed(GripperProtocol &p,int64_t now) {
  ASSERT_EQ(p.next(now,false).data[4],0x41);
  receive(p,reply(0x41,{1,0,25,0xEB,3,0xF4,1}),now+1000000);
  p.resume();
}
TEST(GripperSpeed, ConfigureActivateAndMeasuredHoldIssueOnlyQueries) {
  for (bool read_only:{false,true}) {
    auto c=sdk_config();c.gripper_read_only=read_only;GripperProtocol p(c,0);
    constexpr int64_t now=1000000000;seed(p,now);
    EXPECT_EQ(p.target(1000),!read_only);
    for (int tick=1;tick<=20;++tick) {
      auto frame=p.next(now+tick*5000000LL,true);
      if (frame.size) {
        EXPECT_EQ(frame.data[4],0x41);
        receive(p,reply(0x41,{1,0,25,0xEB,3,0xF4,1}),now+tick*5000000LL+1000000);
      }
    }
    EXPECT_EQ(p.diagnostics().speed_writes,0u);
    EXPECT_EQ(p.diagnostics().first_opening_raw,-1);
  }
}
TEST(GripperSpeed, ExplicitGoalRequiresSpeedAckAndMatchingReadbackBeforePosition) {
  auto c=sdk_config();c.grippers[0].speed_raw=375;GripperProtocol p(c,0);
  constexpr int64_t now=1000000000;seed(p,now);ASSERT_TRUE(p.target(500));
  EXPECT_EQ(p.next(now+5000000,false).size,0u);
  const auto speed=p.next(now+5000000,true);
  ASSERT_EQ(speed.size,18u);EXPECT_EQ(speed.data[4],0x51);
  for (int i=0;i<6;++i) EXPECT_EQ(unsigned(speed.data[5+2*i])|(unsigned(speed.data[6+2*i])<<8),375u);
  EXPECT_EQ(p.next(now+10000000,true).size,0u);
  receive(p,reply(0x51,{1}),now+11000000);
  EXPECT_EQ(p.next(now+15000000,true).data[4],0xD2);
  EXPECT_EQ(p.next(now+20000000,true).size,0u);
  std::vector<uint8_t> words(12,0);words[0]=375&255;words[1]=375>>8;
  receive(p,reply(0xD2,words),now+21000000);
  auto opening=p.next(now+25000000,true);
  ASSERT_EQ(opening.data[4],0x54);EXPECT_EQ(unsigned(opening.data[5])|(unsigned(opening.data[6])<<8),500u);
  EXPECT_EQ(p.feedback().command_result,0);
  EXPECT_EQ(p.diagnostics().configured_speed_raw,375);EXPECT_EQ(p.diagnostics().readback_speed_raw,375);
}
TEST(GripperSpeed, RejectionTimeoutAndMismatchedReadbackNeverReleaseAnOpening) {
  for (int fault=0;fault<4;++fault) {
    auto c=sdk_config();GripperProtocol p(c,0);constexpr int64_t now=1000000000;
    seed(p,now);ASSERT_TRUE(p.target(500));ASSERT_EQ(p.next(now+5000000,true).data[4],0x51);
    if (fault==0) receive(p,reply(0x51,{0x55}),now+6000000);
    if (fault==1) p.next(now+206000000,true);
    if (fault>=2) {
      receive(p,reply(0x51,{1}),now+6000000);
      ASSERT_EQ(p.next(now+10000000,true).data[4],0xD2);
      if (fault==2) receive(p,reply(0xD2,{1,0}),now+11000000);
      else p.next(now+211000000,true);
    }
    EXPECT_NE(p.feedback().command_result,0);EXPECT_FALSE(p.target(600));
    EXPECT_EQ(p.diagnostics().first_opening_raw,-1);
    EXPECT_NE(p.next(now+400000000,true).data[4],0x54);
    p.stop();p.resume();EXPECT_FALSE(p.target(500)); // No automatic error reset.
  }
}
TEST(GripperSpeed, StopDuringSetupDiscardsPositionAndLateAck) {
  auto c=sdk_config();GripperProtocol p(c,0);constexpr int64_t now=1000000000;
  seed(p,now);ASSERT_TRUE(p.target(500));ASSERT_EQ(p.next(now+5000000,true).data[4],0x51);
  ASSERT_EQ(p.stop().data[4],0x16);receive(p,reply(0x51,{1}),now+6000000);
  EXPECT_NE(p.next(now+10000000,true).data[4],0x54);
  p.resume();ASSERT_TRUE(p.target(1000));
  EXPECT_NE(p.next(now+300000000,true).data[4],0x54);
  EXPECT_EQ(p.diagnostics().first_opening_raw,-1);
}
TEST(GripperSpeed, ContinuousTargetsAreCoalescedAtTwentyHzAndPollingContinues) {
  auto c=sdk_config();GripperProtocol p(c,0);
  SerialFrame pending;int64_t due=0;unsigned measured=1000;
  std::vector<int64_t> writes;int polls=0,settings=0,reads=0;
  auto tx=[&](const SerialFrame &f,int64_t now) {
    if (!f.size)return;
    const auto command=f.data[4];
    if (command==0x41) {++polls;pending=reply(0x41,{3,0,25,uint8_t(measured),uint8_t(measured>>8),0xF4,1});}
    else if (command==0x51) {++settings;pending=reply(command,{1});}
    else if (command==0xD2) {++reads;pending=reply(command,{0xF4,1});}
    else {
      ASSERT_EQ(command,0x54);writes.push_back(now);
      measured=unsigned(f.data[5])|(unsigned(f.data[6])<<8);pending=reply(command,{1});
    }
    due=now+1000000;
  };
  for (int tick=0;tick<240;++tick) {
    const int64_t now=1000000000LL+tick*5000000LL;
    if (due && now>=due) {receive(p,pending,due);due=0;}
    tx(p.next(now,false),now);
    if (tick==1)p.resume();
    ASSERT_TRUE(p.target(900-tick));
    tx(p.next(now+100000,true),now+100000);
  }
  EXPECT_EQ(settings,1);EXPECT_EQ(reads,1);EXPECT_GE(polls,15);
  ASSERT_GE(writes.size(),15u);EXPECT_LE(writes.size(),24u);
  for (std::size_t i=1;i<writes.size();++i) EXPECT_GE(writes[i]-writes[i-1],50000000);
  EXPECT_LE(p.diagnostics().last_opening_raw,680);EXPECT_EQ(p.feedback().command_result,0);
}
TEST(GripperSpeed, MalformedAndStaleAcksDoNotSatisfySetup) {
  auto c=sdk_config();GripperProtocol p(c,0);constexpr int64_t now=1000000000;
  seed(p,now);ASSERT_TRUE(p.target(500));ASSERT_EQ(p.next(now+5000000,true).data[4],0x51);
  receive(p,reply(0x51,{}),now+6000000);
  receive(p,reply(0x51,{1}),now+4000000);
  EXPECT_EQ(p.next(now+10000000,true).size,0u);
  EXPECT_EQ(p.diagnostics().rejected_replies,1u);
  receive(p,reply(0x51,{1}),now+11000000);
  EXPECT_EQ(p.next(now+15000000,true).data[4],0xD2);
}
TEST(GripperSpeed, OpeningFailureIsNotReportedAsAnAcknowledgedTarget) {
  auto c=Config::load(CONFIG_PATH);GripperProtocol p(c,0);constexpr int64_t now=1000000000;
  seed(p,now);ASSERT_TRUE(p.target(500));ASSERT_EQ(p.next(now+5000000,true).data[4],0x54);
  receive(p,reply(0x54,{0x55}),now+6000000);
  EXPECT_NE(p.feedback().command_result,0);EXPECT_EQ(p.diagnostics().opening_rejections,1u);
  EXPECT_FALSE(p.target(400));EXPECT_EQ(p.next(now+10000000,true).size,0u);
}
TEST(GripperSpeed, RejectsInvalidSpeedAndSerialSchedulingConfiguration) {
  auto c=Config::load(CONFIG_PATH);c.grippers[0].speed_raw=0;EXPECT_THROW(c.validate(),std::invalid_argument);
  c.grippers[0].speed_raw=1001;EXPECT_THROW(c.validate(),std::invalid_argument);
  c.grippers[0].speed_raw=500;c.gripper_position_ms=4;EXPECT_THROW(c.validate(),std::invalid_argument);
}
}  // namespace
