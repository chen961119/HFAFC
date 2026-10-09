#pragma once

#include <Wire.h>

// Read-only Teensy/Arduino adaptation of the supplied MT6701 Wire example.
class MT6701 {
public:
  explicit MT6701(TwoWire &bus, uint8_t address = 0x06)
      : bus_(bus), address_(address) {}
  // Degrees in [0, 360); leaves the caller's value unchanged on an I2C error.
  bool readAngle(float &degrees);
  static float relativeAngle(float degrees, float zero, bool reverse);

private:
  bool readRegister(uint8_t reg, uint8_t &value);
  TwoWire &bus_;
  uint8_t address_;
};
