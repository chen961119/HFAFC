#include "aircraft_config.h"
#include "sensor_processing.h"
#include "flight_lock.h"
#include "parameter_service.h"
#include "device_reboot.h"

namespace {
bool flightLocked = true;
}

bool isFlightLocked() { return flightLocked || parameterRebootRequired() || deviceRebootPending(); }
void setFlightLocked(bool locked) {
  flightLocked = locked || parameterRebootRequired() || deviceRebootPending() ||
    !aircraftConfigurationValid() || !imuCalibrationValid();
}
