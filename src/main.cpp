#include "aircraft_config.h"
#include "flight_lock.h"
// 飞控入口：仅编排初始化与每周期任务；各模块的状态和实现留在对应模块中。

#include "actuator_output.h"
#include "control_allocation.h"
#include "control_modes.h"
#include "control_state.h"
#include "filters.h"
#include "firmware_debug.h"
#include "flight_clock.h"
#include "human_interface.h"
#include "interaircraft_comm.h"
#include "logger.h"
#include "parameter_service.h"
#include "radio_comm.h"
#include "sensor_processing.h"
#include "telemetry.h"
#include <Arduino.h>

// 按通信、人机界面、传感器、执行器和控制器的依赖顺序完成上电初始化。
void setup() {
  
  // 建立串口链路，再初始化传感器、执行器和控制器。
  beginHumanInterfaceLinks();
  initializeFirmwareDebug();
  // beginTelemetryLink();
  beginParentLink();
  beginStrainSensorLink();
  beginChildLinks();
  beginExternalImuLink();

  delay(20);

  initializeHumanInterface();
  initializeLogger();
  displayfilenum();
  initializeParameterService(loggerSdReady());
  initBAROMETER();
  loadRotateSensorOffset();
  initializeRotateSensor();
  loadImuCalibration();
  // prepareActuatorPower();
  displayAircraftIdentity();
  attachActuators();

  delay(5);
  radioSetup();
  initializeRadioFailsafeChannels();
  IMUinit();

  delay(5);
  commandSafeActuatorPositions();
  
  delay(5);
  // initializeControlAllocation();
  calibrateAirspeedSensor();
  initializeInitialAttitude();
  initializeInitialControlMode();
  initializeControlFilters();
}

// 执行一次控制周期：采样与估计、生成控制量、输出与记录，最后读取下一周期遥控指令并等待 500 Hz 节拍。
void loop() {

  updateFlightClock();

  getBMI088data();
  Madgwick();
  getAngularACC();
  getIMUdata_EXT();
  increase_Clp();
  getairspeed();
  getRotateSensor1();
  ProcessButtonState();
  displayAttitude();
  Strain_read_all();
  getairdata();
  if (!aircraftIsSingle()) receiveAdjacentAircraftStates();
  // telemetry(); //数传。 传输等效姿态角，相对转角，相对扭转角 10hz

  if (!aircraftConfigurationValid() || parameterRebootRequired()) {
    setFlightLocked(true);
    commandSafeActuatorPositions();
    pollParameterService();
    loopRate(500);
    return;
  }

  getDesState();
  runSelectedControlMode();
  // getpinvBplusmini();
  controlMixer();
  
  convertControlCommandsToPWM();
  prepareActuatorCommands();
  applyAndTransmitActuatorCommands();

  if (aircraftIsSingle()) loggerSINGLE();
  else loggerTEAM();
 
  getCommands(); // 接收值供下一周期控制使用，避免周期中途改变控制输入。
  failSafe();
  pollParameterService();
  
  loopRate(500);// 目标频率 500 Hz；若本周期已超时，loopRate 不会额外等待。
}
