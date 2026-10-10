#pragma once

#include "flight_config.h"
#include <cstdint>
#include <cstring>

inline bool isFiniteActuatorPwm(float value) {
  // Release 固件启用 -ffast-math；按 float32 位型检测 NaN/Inf 才可靠。
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(value), "Expected float32 PWM value");
  std::memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7f800000u) != 0x7f800000u;
}

// 仅用于解锁后的控制输出；非有限数回退到本机锁定位置。
inline int limitActuatorPwm(float value, int minimum, int maximum, int invalidValue) {
  if (!isFiniteActuatorPwm(value)) return invalidValue;
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return static_cast<int>(value);
}
