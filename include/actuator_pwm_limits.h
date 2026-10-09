#pragma once

#include "flight_config.h"
#include <cstdint>
#include <cstring>

// 用于最终输出，非有限数回退到本机锁定位置。
inline int limitActuatorPwm(float value, int invalidValue) {
  // Release 固件启用 -ffast-math；按 float32 位型检测 NaN/Inf 才可靠。
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(value), "Expected float32 PWM value");
  std::memcpy(&bits, &value, sizeof(bits));
  if ((bits & 0x7f800000u) == 0x7f800000u) return invalidValue;
  if (value < PWM_SERVO_MIN_US) return PWM_SERVO_MIN_US;
  if (value > PWM_SERVO_MAX_US) return PWM_SERVO_MAX_US;
  return static_cast<int>(value);
}

inline int lockedActuatorPwm(float trim) {
  return limitActuatorPwm(PWM_CENTER_US + trim, PWM_CENTER_US);
}
