// 飞控入口：仅编排初始化与每周期任务；各模块的状态和实现留在对应模块中。

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

// 按通信、人机界面、传感器、执行器和控制器的依赖顺序完成上电初始化。
void setup() {
  // 先建立串口链路，再初始化传感器、执行器和控制器。
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

// 执行一次控制周期：采样与估计、生成控制量、输出与记录，最后读取下一周期遥控指令并等待 500 Hz 节拍。
void loop() {
  // 本周期的时间戳用于计算 dt，也用于末尾的固定频率等待。
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
  // 接收值供下一周期控制使用，避免周期中途改变控制输入。
  getCommands();
  failSafe();
  // 目标频率 500 Hz；若本周期已超时，loopRate 不会额外等待。
  loopRate(500, current_time);
}
