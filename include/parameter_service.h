#pragma once
#include <stdint.h>

// Called after SD.begin(), before controller/filter initialization.
void initializeParameterService(bool sdReady);
// Bounded serial RX/TX processing; call once at the end of a control cycle.
// SD writes are synchronous.
void pollParameterService();
// True means durably saved. Immediate parameters may already be applied on false.
bool saveParameterValue(uint16_t id, double value);
// Latched after a committed change to any reboot-effective parameter.
bool parameterRebootRequired();
bool saveImuCalibrationParameters(const float *offsets);
