#include "MT6701.h"

bool MT6701::readRegister(uint8_t reg, uint8_t &value) {
  bus_.beginTransmission(address_);
  bus_.write(reg);
  if (bus_.endTransmission(false) != 0) return false;
  if (bus_.requestFrom(address_, static_cast<uint8_t>(1)) != 1) return false;
  const int received = bus_.read();
  if (received < 0) return false;
  value = static_cast<uint8_t>(received);
  return true;
}

bool MT6701::readAngle(float &degrees) {
  uint8_t msb, lsb;
  // Read register 0x03 before 0x04, matching the supplied working example.
  if (!readRegister(0x03, msb) || !readRegister(0x04, lsb)) return false;
  const uint16_t raw = (static_cast<uint16_t>(msb) << 6) | (lsb >> 2);
  degrees = raw * (360.0f / 16384.0f);
  return true;
}

float MT6701::relativeAngle(float degrees, float zero, bool reverse) {
  float delta = degrees - zero;
  if (delta > 180.0f) delta -= 360.0f;
  else if (delta < -180.0f) delta += 360.0f;
  return reverse ? -delta : delta;
}
