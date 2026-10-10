#include "aircraft_config.h"
#include "serial_ports.h"
#include "actuator_output.h"
#include "control_modes.h"
#include "debug_print.h"
#include "flight_config.h"
#include "flight_lock.h"
#include "actuator_pwm_limits.h"
#include "control_state.h"
#include "interaircraft_comm.h"
#include <Servo.h>

// A* 为本机最终物理 PWM；B～G 的旧 *_PWM 名称表示待发送控制偏移（μs），不含中位/rev/trim。
float ail1_PWM, ail2_PWM, ele_PWM, thro_PWM, rudd_PWM;
float B_ail1_PWM, B_ail2_PWM, B_thro_PWM, B_rudd_PWM;
float C_ail1_PWM, C_ail2_PWM, C_thro_PWM, C_rudd_PWM;
float D_ail1_PWM, D_ail2_PWM, D_thro_PWM, D_rudd_PWM;
float E_ail1_PWM, E_ail2_PWM, E_thro_PWM, E_rudd_PWM;
float F_ail1_PWM, F_ail2_PWM, F_thro_PWM, F_rudd_PWM;
float G_ail1_PWM, G_ail2_PWM, G_thro_PWM, G_rudd_PWM;
float B_ele_command_PWM_Manual, B_ele_command_PWM_FF;
float C_ele_command_PWM_Manual, C_ele_command_PWM_FF;
float D_ele_command_PWM_Manual, D_ele_command_PWM_FF;
float E_ele_command_PWM_Manual, E_ele_command_PWM_FF;
float F_ele_command_PWM_Manual, F_ele_command_PWM_FF;
float G_ele_command_PWM_Manual, G_ele_command_PWM_FF;

namespace {
// 缩放后的逻辑控制量（μs 尺度，以 1500 为逻辑零点）。
float A_ail1_control_us, A_ail2_control_us, A_ele_control_us,
    A_thro_control_us, A_rudd_control_us;
float B_ail1_control_us, B_ail2_control_us, B_thro_control_us,
    B_rudd_control_us, B_ele_control_us_manual;
float C_ail1_control_us, C_ail2_control_us, C_thro_control_us,
    C_rudd_control_us, C_ele_control_us_manual;
float D_ail1_control_us, D_ail2_control_us, D_thro_control_us,
    D_rudd_control_us, D_ele_control_us_manual;
float E_ail1_control_us, E_ail2_control_us, E_thro_control_us,
    E_rudd_control_us, E_ele_control_us_manual;
float F_ail1_control_us, F_ail2_control_us, F_thro_control_us,
    F_rudd_control_us, F_ele_control_us_manual;
float G_ail1_control_us, G_ail2_control_us, G_thro_control_us,
    G_rudd_control_us, G_ele_control_us_manual;
// 本地俯仰控制产生的物理 PWM，尚未叠加上级前馈或手动覆盖。
float localElevatorBasePWM;
float localElevatorControlDeviation;
void prepareLocalElevatorCommand();
} // namespace

// 预留的电调输出引脚；实际舵面由下方 Servo 对象输出 PWM。
// const int m1Pin = 37;
// const int m2Pin = 37;
// const int m3Pin = 38;
// const int m4Pin = 39;
// const int m5Pin = 40;
// const int m6Pin = 41;

const int servo1Pin = 2; // 左副翼
const int servo2Pin = 3; // 右副翼
const int servo3Pin = 4; // 升降
const int servo4Pin = 5; // 油门
const int servo5Pin = 6; // 方向
// const int servo6Pin = 9;
// const int servo7Pin = 9;

// 舵机或电调 PWM 输出对象。
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
// Servo servo6;
// Servo servo7;

// 在指定硬件版本上电阶段将 5 号引脚写为低，并延时等待执行器供电稳定。
void prepareActuatorPower() {
  // 指定硬件版本在绑定舵机前，先将 5 号引脚写为低并等待 100 ms。
#if defined expensive
  digitalWrite(5, LOW);
  delay(100);
#endif
}

// 绑定各舵机和电调引脚，配置独立于各通道软件限幅的驱动脉宽范围。
void attachActuators() {
  // 指定各路执行器的有效脉宽范围，单位 μs。
  servo1.attach(servo1Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US);// 左副翼
  servo2.attach(servo2Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US);// 右副翼
  servo3.attach(servo3Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US);// 升降
  servo4.attach(servo4Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US);// 油门
  servo5.attach(servo5Pin, PWM_SERVO_ATTACH_MIN_US, PWM_SERVO_ATTACH_MAX_US);// 方向
  // servo6.attach(servo6Pin, 1000, 2000);
  // servo7.attach(servo7Pin, 1000, 2000);

  commandSafeActuatorPositions();
}

// 上电和锁定时的固定物理位置，不应用 rev 或控制量。
void commandSafeActuatorPositions() {
  servo1.writeMicroseconds(PWM_CENTER_US + pwm_channel1_trim);
  servo2.writeMicroseconds(PWM_CENTER_US + pwm_channel2_trim);
  servo3.writeMicroseconds(PWM_CENTER_US + pwm_channel3_trim);
  servo4.writeMicroseconds(PWM_CENTER_US + pwm_channel4_trim);
  servo5.writeMicroseconds(PWM_CENTER_US + pwm_channel5_trim);
}

// 将归一化控制指令转换为微秒尺度的逻辑 PWM；此阶段尚未应用 rev 或 trim。
void convertControlCommandsToPWM() {
  // *_control_us 以 1500 为逻辑零点，不是最终物理 PWM。
  // 对输出做行程限幅，避免超过执行器允许范围。
  A_ail1_control_us = PWM_CENTER_US + 1000 * (A_ail1_scaled);
  A_ail2_control_us = PWM_CENTER_US + 1000 * (A_ail2_scaled);
  A_ele_control_us = PWM_CENTER_US + 1000 * (A_ele_scaled);
  A_thro_control_us = PWM_CENTER_US + 1000 * (A_thro_scaled);
  A_rudd_control_us = PWM_CENTER_US + 1000 * (A_rudd_scaled);

  B_ail1_control_us = PWM_CENTER_US + 1000 * (B_ail1_scaled);
  B_ail2_control_us = PWM_CENTER_US + 1000 * (B_ail2_scaled);
  B_thro_control_us = PWM_CENTER_US + 1000 * (B_thro_scaled);
  B_rudd_control_us = PWM_CENTER_US + 1000 * (B_rudd_scaled);
  B_ele_control_us_manual = PWM_CENTER_US + 1000 * (A_ele_scaled) +
                             0.5 * 1000 * (B_ail1_scaled + B_ail2_scaled) / 2;

  C_ail1_control_us = PWM_CENTER_US + 1000 * (C_ail1_scaled);
  C_ail2_control_us = PWM_CENTER_US + 1000 * (C_ail2_scaled);
  C_thro_control_us = PWM_CENTER_US + 1000 * (C_thro_scaled);
  C_rudd_control_us = PWM_CENTER_US + 1000 * (C_rudd_scaled);
  C_ele_control_us_manual = PWM_CENTER_US + 1000 * (A_ele_scaled) +
                             0.5 * 1000 * (C_ail1_scaled + C_ail2_scaled) / 2;

  D_ail1_control_us = PWM_CENTER_US + 1000 * (D_ail1_scaled);
  D_ail2_control_us = PWM_CENTER_US + 1000 * (D_ail2_scaled);
  D_thro_control_us = PWM_CENTER_US + 1000 * (D_thro_scaled);
  D_rudd_control_us = PWM_CENTER_US + 1000 * (D_rudd_scaled);
  D_ele_control_us_manual = PWM_CENTER_US + 1000 * (A_ele_scaled) +
                             0.5 * 1000 * (D_ail1_scaled + D_ail2_scaled) / 2;

  E_ail1_control_us = PWM_CENTER_US + 1000 * (E_ail1_scaled);
  E_ail2_control_us = PWM_CENTER_US + 1000 * (E_ail2_scaled);
  E_thro_control_us = PWM_CENTER_US + 1000 * (E_thro_scaled);
  E_rudd_control_us = PWM_CENTER_US + 1000 * (E_rudd_scaled);
  E_ele_control_us_manual = PWM_CENTER_US + 1000 * (A_ele_scaled) +
                             0.5 * 1000 * (E_ail1_scaled + E_ail2_scaled) / 2;

  F_ail1_control_us = PWM_CENTER_US + 1000 * (F_ail1_scaled);
  F_ail2_control_us = PWM_CENTER_US + 1000 * (F_ail2_scaled);
  F_thro_control_us = PWM_CENTER_US + 1000 * (F_thro_scaled);
  F_rudd_control_us = PWM_CENTER_US + 1000 * (F_rudd_scaled);
  F_ele_control_us_manual = PWM_CENTER_US + 1000 * (A_ele_scaled) +
                             0.5 * 1000 * (F_ail1_scaled + F_ail2_scaled) / 2;

  G_ail1_control_us = PWM_CENTER_US + 1000 * (G_ail1_scaled);
  G_ail2_control_us = PWM_CENTER_US + 1000 * (G_ail2_scaled);
  G_thro_control_us = PWM_CENTER_US + 1000 * (G_thro_scaled);
  G_rudd_control_us = PWM_CENTER_US + 1000 * (G_rudd_scaled);
  G_ele_control_us_manual = PWM_CENTER_US + 1000 * (A_ele_scaled) +
                             0.5 * 1000 * (G_ail1_scaled + G_ail2_scaled) / 2;

  // s6_command_PWM = s6_command_scaled * 180;
  // s7_command_PWM = s7_command_scaled * 180;

  // 保留原有逻辑控制量行程限制。
  // A_ail1_control_us = constrain(A_ail1_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  // A_ail2_control_us = constrain(A_ail2_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  // A_ele_control_us = constrain(A_ele_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US); // 贵的飞机限幅
  // A_thro_control_us = constrain(A_thro_control_us, 1000, 2000);
  // A_rudd_control_us = constrain(A_rudd_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  // s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  // s7_command_PWM = constrain(s7_command_PWM, 0, 180);
}

// 准备最终物理 PWM 与子机命令；apply 阶段只应用本函数的结果。
void prepareActuatorCommands() {
  prepareLocalElevatorCommand();
if ((aircraftId() == 1)) {
  // 准备各机控制偏移；本机补偿、rev/trim 在下方共同应用。

  // A机
  int deviation1 = (A_ail1_control_us - PWM_CENTER_US) * 0.4;
  int deviation2 = (A_ail2_control_us - PWM_CENTER_US) * 0.4;
  int deviation4 = A_thro_control_us - PWM_CENTER_US;
  int deviation5 = A_rudd_control_us - PWM_CENTER_US;

  // 刹车量与副翼偏移使用相同的方向约定，在 rev 反向前叠加。
  if (channel_8_pwm > 1600) {
    ailBrake_PWM = 0.0;
  } else {
    ailBrake_PWM = 100.0;
  }

  // A 机无上级接收者，每周期用自身控制结果填充本机控制偏移。
  Local_ail1_control_us = deviation1 + ailBrake_PWM;
  Local_ail2_control_us = deviation2 + ailBrake_PWM;
  Local_thro_control_us = deviation4;
  Local_rudd_control_us = deviation5;
  Local_ele_control_us = A_ele_control_us - PWM_CENTER_US;
  Local_ele_ff_control_us = 0;

  if (!aircraftIsSingle()) {
  // B机
  deviation1 = (B_ail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (B_ail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = B_thro_control_us - PWM_CENTER_US;
  deviation5 = B_rudd_control_us - PWM_CENTER_US;

  B_ail1_PWM = deviation1 + ailBrake_PWM;
  B_ail2_PWM = deviation2 + ailBrake_PWM;
  B_thro_PWM = deviation4;
  B_rudd_PWM = deviation5;

  B_ele_command_PWM_Manual = B_ele_control_us_manual - PWM_CENTER_US;
  B_ele_command_PWM_FF = B_ele_control_ff_us;
  setLeftChildCommand(0, int(B_ail1_PWM),
                      int(B_ail2_PWM), int(B_thro_PWM),
                      int(B_rudd_PWM), B_pitch_sp,
                      int(B_ele_command_PWM_Manual),
                      int(B_ele_command_PWM_FF));

  // C机
  deviation1 = (C_ail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (C_ail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = C_thro_control_us - PWM_CENTER_US;
  deviation5 = C_rudd_control_us - PWM_CENTER_US;

  C_ail1_PWM = deviation1 + ailBrake_PWM;
  C_ail2_PWM = deviation2 + ailBrake_PWM;
  C_thro_PWM = deviation4;
  C_rudd_PWM = deviation5;

  C_ele_command_PWM_Manual = C_ele_control_us_manual - PWM_CENTER_US;
  C_ele_command_PWM_FF = C_ele_control_ff_us;
  setRightChildCommand(0, int(C_ail1_PWM),
                       int(C_ail2_PWM), int(C_thro_PWM),
                       int(C_rudd_PWM), C_pitch_sp,
                       int(C_ele_command_PWM_Manual),
                       int(C_ele_command_PWM_FF));
  // D机
  deviation1 = (D_ail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (D_ail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = D_thro_control_us - PWM_CENTER_US;
  deviation5 = D_rudd_control_us - PWM_CENTER_US;

  D_ail1_PWM = deviation1 + ailBrake_PWM;
  D_ail2_PWM = deviation2 + ailBrake_PWM;
  D_thro_PWM = deviation4;
  D_rudd_PWM = deviation5;

  D_ele_command_PWM_Manual = D_ele_control_us_manual - PWM_CENTER_US;
  D_ele_command_PWM_FF = D_ele_control_ff_us;
  setLeftChildCommand(1, int(D_ail1_PWM),
                      int(D_ail2_PWM), int(D_thro_PWM),
                      int(D_rudd_PWM), D_pitch_sp,
                      int(D_ele_command_PWM_Manual),
                      int(D_ele_command_PWM_FF));

  // E机
  deviation1 = (E_ail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (E_ail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = E_thro_control_us - PWM_CENTER_US;
  deviation5 = E_rudd_control_us - PWM_CENTER_US;

  E_ail1_PWM = deviation1 + ailBrake_PWM;
  E_ail2_PWM = deviation2 + ailBrake_PWM;
  E_thro_PWM = deviation4;
  E_rudd_PWM = deviation5;

  E_ele_command_PWM_Manual = E_ele_control_us_manual - PWM_CENTER_US;
  E_ele_command_PWM_FF = E_ele_control_ff_us;
  setRightChildCommand(1, int(E_ail1_PWM),
                       int(E_ail2_PWM), int(E_thro_PWM),
                       int(E_rudd_PWM), E_pitch_sp,
                       int(E_ele_command_PWM_Manual),
                       int(E_ele_command_PWM_FF));

  // F机
  deviation1 = (F_ail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (F_ail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = F_thro_control_us - PWM_CENTER_US;
  deviation5 = F_rudd_control_us - PWM_CENTER_US;

  F_ail1_PWM = deviation1 + ailBrake_PWM;
  F_ail2_PWM = deviation2 + ailBrake_PWM;
  F_thro_PWM = deviation4;
  F_rudd_PWM = deviation5;

  F_ele_command_PWM_Manual = F_ele_control_us_manual - PWM_CENTER_US;
  F_ele_command_PWM_FF = F_ele_control_ff_us;
  setLeftChildCommand(2, int(F_ail1_PWM),
                      int(F_ail2_PWM), int(F_thro_PWM),
                      int(F_rudd_PWM), F_pitch_sp,
                      int(F_ele_command_PWM_Manual),
                      int(F_ele_command_PWM_FF));

  // G机
  deviation1 = (G_ail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (G_ail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = G_thro_control_us - PWM_CENTER_US;
  deviation5 = G_rudd_control_us - PWM_CENTER_US;

  G_ail1_PWM = deviation1 + ailBrake_PWM;
  G_ail2_PWM = deviation2 + ailBrake_PWM;
  G_thro_PWM = deviation4;
  G_rudd_PWM = deviation5;

  G_ele_command_PWM_Manual = G_ele_control_us_manual - PWM_CENTER_US;
  G_ele_command_PWM_FF = G_ele_control_ff_us;
  setRightChildCommand(2, int(G_ail1_PWM),
                       int(G_ail2_PWM), int(G_thro_PWM),
                       int(G_rudd_PWM), G_pitch_sp,
                       int(G_ele_command_PWM_Manual),
                       int(G_ele_command_PWM_FF));
  }
}
  // 所有飞机共用：控制偏移叠加本地补偿，再应用本机 rev/trim。
  const float elevatorAileronCompensation = localElevatorControlDeviation / 15.0f;
  ail1_PWM = PWM_CENTER_US + pwm_channel1_trim + pwm_channel1_rev *
      (Local_ail1_control_us + elevatorAileronCompensation + Ail_Clp);
  ail2_PWM = PWM_CENTER_US + pwm_channel2_trim + pwm_channel2_rev *
      (Local_ail2_control_us + elevatorAileronCompensation + Ail_Clp);
  ele_PWM = currentMode == MANUAL_MODE ?
      PWM_CENTER_US + pwm_channel3_trim + pwm_channel3_rev * Local_ele_control_us :
      localElevatorBasePWM + pwm_channel3_rev * Local_ele_ff_control_us;
  thro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * Local_thro_control_us;
  rudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * Local_rudd_control_us;
}

namespace {
// 主从机统一准备本地俯仰控制产生的升降舵物理 PWM。
void prepareLocalElevatorCommand() {
#if defined TESTINDI
  if (currentMode == STABILIZE_MODE) {
    // INDI 已返回物理 PWM，包含中位与安装 trim，不再重复反向或加 trim。
    localElevatorBasePWM = PITCH_INDI_control();
    // 仅把本机 INDI 的实测物理 PWM 换回控制方向，供副翼补偿使用。
    localElevatorControlDeviation =
        (localElevatorBasePWM - PWM_CENTER_US - pwm_channel3_trim) / pwm_channel3_rev;
    return;
  }
#endif
  // 其他模式沿用 PID/手动混控的逻辑 PWM 到实际 PWM 的转换。
  const int deviation = A_ele_control_us - PWM_CENTER_US;
  localElevatorControlDeviation = deviation;
  localElevatorBasePWM = PWM_CENTER_US + pwm_channel3_trim + pwm_channel3_rev * deviation;
}

} // namespace

// 直接应用已准备好的物理 PWM，再发送或转发机间数据。
void applyAndTransmitActuatorCommands() {
  // 锁定时直接回到 trim 位置；仅解锁后的控制输出应用软件限幅。
  if (isFlightLocked()) {
    ail1_PWM = PWM_CENTER_US + pwm_channel1_trim;
    ail2_PWM = PWM_CENTER_US + pwm_channel2_trim;
    ele_PWM  = PWM_CENTER_US + pwm_channel3_trim;
    thro_PWM = PWM_CENTER_US + pwm_channel4_trim;
    rudd_PWM = PWM_CENTER_US + pwm_channel5_trim;
  } else {
    ail1_PWM = limitActuatorPwm(ail1_PWM, PWM_SERVO1_MIN_US, PWM_SERVO1_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel1_trim));
    ail2_PWM = limitActuatorPwm(ail2_PWM, PWM_SERVO2_MIN_US, PWM_SERVO2_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel2_trim));
    ele_PWM = limitActuatorPwm(ele_PWM, PWM_SERVO3_MIN_US, PWM_SERVO3_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel3_trim));
    thro_PWM = limitActuatorPwm(thro_PWM, PWM_SERVO4_MIN_US, PWM_SERVO4_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel4_trim));
    rudd_PWM = limitActuatorPwm(rudd_PWM, PWM_SERVO5_MIN_US, PWM_SERVO5_MAX_US, static_cast<int>(PWM_CENTER_US + pwm_channel5_trim));
  }
  servo1.writeMicroseconds(ail1_PWM);
  servo2.writeMicroseconds(ail2_PWM);
  servo3.writeMicroseconds(ele_PWM);
  servo4.writeMicroseconds(thro_PWM);
  servo5.writeMicroseconds(rudd_PWM);
  if (aircraftIsSingle()) return;
if ((aircraftId() == 1)) {
  sendPreparedChildCommands(int_is_valid);
} else {
  sendGYROxANGLE();
  forwardReceivedChildCommands(int_is_valid);
}
}
