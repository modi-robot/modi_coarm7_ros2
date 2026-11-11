#include "coarm7_hardware/gripper.hpp"
#include <algorithm>
#include <cmath>
namespace coarm7_hardware {
namespace {
uint8_t checksum(const uint8_t *p, std::size_t n) { uint8_t v = 0; while (n--) v += *p++; return v; }
}
SerialFrame gripper_request(uint8_t device, uint8_t command, const uint8_t *p, std::size_t n) {
  SerialFrame f;
  if (!device || device == 255 || n > 54 || (n && !p)) return f;
  f.data[0]=0xEB; f.data[1]=0x90; f.data[2]=device; f.data[3]=n+1; f.data[4]=command;
  for (std::size_t i=0;i<n;++i) f.data[5+i]=p[i];
  f.data[5+n]=checksum(f.data.data()+2,n+3); f.size=n+6; return f;
}
bool gripper_response(const uint8_t *p, std::size_t n, uint8_t device, uint8_t &cmd, SerialFrame &payload) {
  if (!p || n > 60) return false;
  for (std::size_t off=0; off+6<=n; ++off) {
    const auto *f=p+off;
    if (f[0]!=0xEE || f[1]!=0x16 || f[2]!=device || f[3]<1) continue;
    const std::size_t size=f[3]+5u;
    if (size>n-off || checksum(f+2,size-3)!=f[size-1]) continue;
    cmd=f[4]; payload.size=f[3]-1;
    for (std::size_t i=0;i<payload.size;++i) payload.data[i]=f[5+i];
    return true;
  }
  return false;
}
GripperProtocol::GripperProtocol(const Config &c, std::size_t i)
 : device_(c.grippers[i].device_id), poll_ms_(c.gripper_poll_ms),
   response_ms_(c.gripper_response_ms), command_ms_(c.gripper_command_ms),
   position_ms_(c.transport=="sdk" ? c.gripper_position_ms : c.gripper_command_ms),
   speed_raw_(c.grippers[i].speed_raw),
   command_min_(static_cast<int>(std::ceil(c.grippers[i].command_raw_min))),
   command_max_(static_cast<int>(std::floor(c.grippers[i].command_raw_max))),
   commands_enabled_(c.gripper_commands_enabled()),
   needs_speed_setup_(c.transport=="sdk" && commands_enabled_),
   speed_setup_(needs_speed_setup_ ? SpeedSetup::needed : SpeedSetup::ready) {}
bool GripperProtocol::target(double raw) {
  if (!commands_enabled_ || stopped_ || feedback_.command_result || !std::isfinite(raw) ||
      raw < command_min_ || raw > command_max_) return false;
  target_raw_=static_cast<int>(std::lround(raw));
  diagnostics_.last_target_raw=target_raw_;
  return true;
}
SerialFrame GripperProtocol::next(int64_t now, bool allow_motion) {
  if (now<next_send_) return {};
  if (pending_ && now-pending_since_<response_ms_*1000000LL) return {};
  if (pending_==0x41) ++diagnostics_.query_timeouts;
  else if (pending_) {
    if (pending_==0x54) ++diagnostics_.opening_timeouts;
    feedback_.command_result=-1; diagnostics_.failed_command=pending_;
    if (pending_==0x51 || pending_==0xD2) speed_setup_=SpeedSetup::failed;
  }
  pending_=0;
  SerialFrame f;
  // A poll-only phase may defer a pending opening, but must not replace its
  // reserved turn with another poll when the preceding reply took >= poll_ms_.
  if (now>=next_poll_ && (poll_next_ || target_raw_==sent_raw_ || target_raw_<0 || stopped_ ||
      (!allow_motion && speed_setup_!=SpeedSetup::ready))) {
    f=gripper_request(device_,0x41); next_poll_=now+poll_ms_*1000000LL; poll_next_=false;
  } else if (allow_motion && !stopped_ && !feedback_.command_result &&
             target_raw_>=0 && target_raw_!=sent_raw_) {
    if (speed_setup_==SpeedSetup::needed) {
      // EG2 manual 2.4.13: compatible 0x51 sets six speed words without
      // issuing a catch/release/position action. No force or flash write.
      uint8_t p[12]{};
      for (std::size_t i=0;i<6;++i) { p[2*i]=speed_raw_; p[2*i+1]=speed_raw_>>8; }
      f=gripper_request(device_,0x51,p,sizeof(p));
      speed_setup_=SpeedSetup::awaiting_ack;
      diagnostics_.configured_speed_raw=speed_raw_; ++diagnostics_.speed_writes;
    } else if (speed_setup_==SpeedSetup::readback_needed) {
      f=gripper_request(device_,0xD2); speed_setup_=SpeedSetup::awaiting_readback;
      ++diagnostics_.speed_reads;
    } else if (speed_setup_==SpeedSetup::ready && now>=next_position_) {
      uint8_t p[2]={static_cast<uint8_t>(target_raw_),static_cast<uint8_t>(target_raw_>>8)};
      f=gripper_request(device_,0x54,p,2); sent_raw_=target_raw_; poll_next_=true;
      next_position_=now+position_ms_*1000000LL;
      if (diagnostics_.first_opening_raw<0) {
        diagnostics_.first_opening_raw=target_raw_;
        diagnostics_.min_opening_raw=diagnostics_.max_opening_raw=target_raw_;
      }
      diagnostics_.last_opening_raw=target_raw_;
      diagnostics_.min_opening_raw=std::min(diagnostics_.min_opening_raw,target_raw_);
      diagnostics_.max_opening_raw=std::max(diagnostics_.max_opening_raw,target_raw_);
    }
  }
  if (f.size) { pending_=f.data[4]; pending_since_=now; next_send_=now+command_ms_*1000000LL; }
  return f;
}
void GripperProtocol::received(const uint8_t *p, std::size_t n, int64_t stamp) {
  uint8_t cmd=0; SerialFrame data;
  if (!gripper_response(p,n,device_,cmd,data) || (cmd==0x41 && data.size!=7) ||
      ((cmd==0x51 || cmd==0x54) && data.size!=1) ||
      (cmd==0xD2 && data.size!=2 && data.size!=12)) {
    ++diagnostics_.rejected_replies;
    return;
  }
  if (pending_==cmd && stamp>=pending_since_) {
    const auto elapsed=stamp-pending_since_;
    if (cmd==0x41) diagnostics_.max_query_reply_ns=std::max(diagnostics_.max_query_reply_ns,elapsed);
    else if (cmd==0x54) diagnostics_.max_opening_reply_ns=std::max(diagnostics_.max_opening_reply_ns,elapsed);
    pending_=0;
    if (cmd==0x51 || cmd==0x54) {
      if (data.data[0]!=1) {
        feedback_.command_result=-2; diagnostics_.failed_command=cmd;
        if (cmd==0x51) speed_setup_=SpeedSetup::failed;
        else ++diagnostics_.opening_rejections;
      } else if (cmd==0x51 && speed_setup_==SpeedSetup::awaiting_ack) {
        speed_setup_=SpeedSetup::readback_needed;
      }
    } else if (cmd==0xD2 && speed_setup_==SpeedSetup::awaiting_readback) {
      const int speed=data.data[0] | (unsigned(data.data[1])<<8);
      diagnostics_.readback_speed_raw=speed;
      if (speed==speed_raw_) speed_setup_=SpeedSetup::ready;
      else {
        speed_setup_=SpeedSetup::failed; feedback_.command_result=-3;
        diagnostics_.failed_command=cmd;
      }
    }
  }
  if (cmd==0x54) {
    ++diagnostics_.opening_replies;
    diagnostics_.last_opening_reply=data.size ? data.data[0] : -1;
  }
  if (cmd!=0x41 || data.size!=7 || stamp<=feedback_.received_ns) return;
  feedback_.status=data.data[0]; feedback_.fault=data.data[1];
  feedback_.raw=static_cast<unsigned>(data.data[3])|(static_cast<unsigned>(data.data[4])<<8);
  feedback_.received_ns=stamp; feedback_.valid=true;
  ++diagnostics_.feedback_replies;
}
SerialFrame GripperProtocol::stop() {
  if (!commands_enabled_) return {};
  stopped_=true; target_raw_=-1; pending_=0;
  if (speed_setup_!=SpeedSetup::ready) speed_setup_=SpeedSetup::needed;
  return gripper_request(device_,0x16);
}
void GripperProtocol::resume() {
  stopped_=false;
  target_raw_=sent_raw_=feedback_.valid ?
    static_cast<int>(std::lround(std::clamp(feedback_.raw,double(command_min_),double(command_max_)))) : -1;
  // Keep polling cadence and any pending query across lifecycle transitions.
  // A previous stop already discarded any outstanding motion command.
}
}
