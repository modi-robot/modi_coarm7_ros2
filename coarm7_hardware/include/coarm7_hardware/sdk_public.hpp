#pragma once
#include "coarm7_hardware/device.hpp"
#include "modi_joint_canevo.h"
#include <cmath>
#include <cstdio>
#include <string>

namespace coarm7_hardware {
// These value-only SDK getters return zero on failed reads. Required zero
// values are therefore unverified, not a successful fault-free/limit reading.
constexpr int kPublicValueUnverified = -1000;
inline int public_version(const std::string &text, uint16_t &out) {
  unsigned major=0,minor=0,patch=0; char suffix=0;
  if (std::sscanf(text.c_str(),"%u.%u.%u%c",&major,&minor,&patch,&suffix)!=3 ||
      major>15 || minor>31 || patch>127) return kPublicValueUnverified;
  const auto value=uint16_t((major<<12)|(minor<<7)|patch);
  if (!value) return kPublicValueUnverified;
  out=value; return 0;
}
template<class Joint>
class PublicParameterReader final : public ParameterReader {
public:
  explicit PublicParameterReader(Joint &joint) : joint_(joint) {}
  bool physical_units() const override { return true; }
  bool checked_results() const override { return false; }
  int u16(uint8_t index,uint8_t sub,uint16_t &out) override {
    if (index==0 && sub==1) return public_version(joint_.NrtGetProtocolVersion(),out);
    if (index==0 && sub==0x0D) return public_version(joint_.NrtGetFwVersion(),out);
    if (index==0 && sub==0x0E) return public_version(joint_.NrtGetHwVersion(),out);
    uint16_t value=0;
    if (index==0 && sub==0x0F) value=joint_.NrtGetSyncPeriod();
    else if (index==0 && sub==0x10) value=joint_.NrtGetCommTimeout();
    else if (index==0 && sub==0x08) value=uint16_t(joint_.NrtGetFaultCode());
    else if (index==1 && sub==0x0D) {
      value=joint_.NrtGetBrakeStatus();
      // 2 denotes unknown/unsupported. Zero cannot prove a closed brake with
      // this API; it must not be reported as physical stop acknowledgement.
      out=value ? value : 2;
      return 0;
    } else return int(CanEvoError::kNotSupported);
    if (!value) return kPublicValueUnverified;
    out=value; return 0;
  }
  int f32(uint8_t index,uint8_t sub,float &out) override {
    float value=0;
    if (index==1) {
      switch (sub) {
        case 2:value=joint_.NrtGetMaxSpeed();break;
        case 3:value=joint_.NrtGetMaxAccel();break;
        case 4:value=joint_.NrtGetMaxDecel();break;
        case 5:value=joint_.NrtGetEstopDecel();break;
        case 6:value=joint_.NrtGetMaxPos();break;
        case 7:value=joint_.NrtGetMinPos();break;
        default:return int(CanEvoError::kNotSupported);
      }
    } else if (index==0x1F && sub==3) value=joint_.NrtGetTorqueCoefficient();
    else return int(CanEvoError::kNotSupported);
    if (!std::isfinite(value) || value==0) return kPublicValueUnverified;
    out=value; return 0;
  }
private:
  Joint &joint_;
};
}  // namespace coarm7_hardware
