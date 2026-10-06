// Flight-controller initialization and loop scheduling.

#include "actuator_output.h"
#include "control_allocation.h"
#include "control_modes.h"
#include "control_state.h"
#include "filters.h"
#include "flight_clock.h"
#include "human_interface.h"
#include "interaircraft_comm.h"
#include "math_utils.h"
#include "radioComm.h"
#include "sensor_processing.h"
#include <Arduino.h>

void setup() {
  beginHumanInterfaceLinks();
  beginParentLink();
  beginStrainSensorLink();
  beginChildLinks();
  beginExternalImuLink();
  delay(20);
  initializeHumanInterface();
  initBAROMETER();
  loadRotateSensorOffset();
  loadImuCalibration();
  prepareActuatorPower();
  displayAircraftIdentity();
  attachActuators();
  delay(5);
  radioSetup();
  initializeRadioFailsafeChannels();
#if defined INTIMU
  IMUinit();
#endif
  delay(5);
  commandSafeActuatorPositions();
  delay(5);
  initializeControlAllocation();
  calibrateAirspeedSensor();
  initializeInitialAttitude();
  currentMode = MANUAL_MODE;
  initializeControlFilters();
}

//========================================================================================================================//
//                                                       MAIN LOOP //
//========================================================================================================================//

void loop() {
  updateFlightClock();

  armedStatus();
#if defined INTIMU
  getBMI088data();
  Madgwick(dt);
#endif
  getAngularACC();
#if defined EXTIMU
  getIMUdata_EXT();
#endif
  increase_Clp();
  getairspeed();
  getDesState();
  ProcessButtonState();
  displayAttitude();
  Strain_read_all();
  getairdata();
  receiveAdjacentAircraftStates();

  runSelectedControlMode();
  getpinvBplusmini();
  controlMixer();
  scaleCommands();
  prepareActuatorCommands();
  applyAndTransmitActuatorCommands();

  loggerTEAM();
  getCommands();
  failSafe();
  loopRate(500, current_time);
}
