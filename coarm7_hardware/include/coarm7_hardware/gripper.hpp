#pragma once
#include "coarm7_hardware/core.hpp"
#include <array>
#include <cstddef>
namespace coarm7_hardware {
struct SerialFrame { std::array<uint8_t, 60> data{}; std::size_t size = 0; };
SerialFrame gripper_request(uint8_t device, uint8_t command, const uint8_t *data = nullptr, std::size_t size = 0);
bool gripper_response(const uint8_t *data, std::size_t size, uint8_t device, uint8_t &command, SerialFrame &payload);
struct GripperDiagnostics {
  uint64_t feedback_replies = 0, opening_replies = 0, rejected_replies = 0;
  uint64_t query_timeouts = 0, opening_timeouts = 0;
  int64_t max_query_reply_ns = 0, max_opening_reply_ns = 0;
  int last_target_raw = -1, last_opening_reply = -1;
  uint64_t speed_writes = 0, speed_reads = 0, opening_rejections = 0;
  int configured_speed_raw = -1, readback_speed_raw = -1;
  int first_opening_raw = -1, last_opening_raw = -1, min_opening_raw = -1, max_opening_raw = -1;
  uint8_t failed_command = 0;
};
class GripperProtocol {
public:
  GripperProtocol(const Config &c, std::size_t i);
  bool target(double raw);
  // Polls may precede drive checks; opening commands must pass the motion gate.
  SerialFrame next(int64_t now, bool allow_motion = true);
  void received(const uint8_t *, std::size_t size, int64_t stamp);
  SerialFrame stop();
  void resume();  // Seed from feedback after activation or a controller claim.
  const GripperFeedback &feedback() const { return feedback_; }
  const GripperDiagnostics &diagnostics() const { return diagnostics_; }
private:
  enum class SpeedSetup { needed, awaiting_ack, readback_needed, awaiting_readback, ready, failed };
  int device_, poll_ms_, response_ms_, command_ms_;
  int position_ms_, speed_raw_;
  int command_min_, command_max_;
  const bool commands_enabled_;
  const bool needs_speed_setup_;
  SpeedSetup speed_setup_;
  int target_raw_ = -1, sent_raw_ = -1;
  int64_t next_send_ = 0, next_poll_ = 0, pending_since_ = 0;
  int64_t next_position_ = 0;
  uint8_t pending_ = 0;
  bool stopped_ = false, poll_next_ = true;
  GripperFeedback feedback_{};
  GripperDiagnostics diagnostics_{};
};
}
