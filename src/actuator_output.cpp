#include "serial_ports.h"
#include "actuator_output.h"
#include "control_modes.h"
#include "debug_print.h"
#include "flight_config.h"
#include "control_state.h"
#include "interaircraft_comm.h"
#include <Servo.h>

// 最终物理输出（μs）；只由 prepare 阶段更新。
float Aail1_PWM, Aail2_PWM, Aele_PWM, Athro_PWM, Arudd_PWM;
float Bail1_PWM, Bail2_PWM, Bthro_PWM, Brudd_PWM;
float Cail1_PWM, Cail2_PWM, Cthro_PWM, Crudd_PWM;
float Dail1_PWM, Dail2_PWM, Dthro_PWM, Drudd_PWM;
float Eail1_PWM, Eail2_PWM, Ethro_PWM, Erudd_PWM;
float Fail1_PWM, Fail2_PWM, Fthro_PWM, Frudd_PWM;
float Gail1_PWM, Gail2_PWM, Gthro_PWM, Grudd_PWM;
float B_ele_command_PWM_Manual, B_ele_command_PWM_FF;
float C_ele_command_PWM_Manual, C_ele_command_PWM_FF;
float D_ele_command_PWM_Manual, D_ele_command_PWM_FF;
float E_ele_command_PWM_Manual, E_ele_command_PWM_FF;
float F_ele_command_PWM_Manual, F_ele_command_PWM_FF;
float G_ele_command_PWM_Manual, G_ele_command_PWM_FF;

namespace {
// 缩放后的逻辑控制量（μs 尺度，以 1500 为逻辑零点）。
float Aail1_control_us, Aail2_control_us, Aele_control_us,
    Athro_control_us, Arudd_control_us;
float Bail1_control_us, Bail2_control_us, Bthro_control_us,
    Brudd_control_us, B_ele_control_us_manual;
float Cail1_control_us, Cail2_control_us, Cthro_control_us,
    Crudd_control_us, C_ele_control_us_manual;
float Dail1_control_us, Dail2_control_us, Dthro_control_us,
    Drudd_control_us, D_ele_control_us_manual;
float Eail1_control_us, Eail2_control_us, Ethro_control_us,
    Erudd_control_us, E_ele_control_us_manual;
float Fail1_control_us, Fail2_control_us, Fthro_control_us,
    Frudd_control_us, F_ele_control_us_manual;
float Gail1_control_us, Gail2_control_us, Gthro_control_us,
    Grudd_control_us, G_ele_control_us_manual;
// 本地俯仰控制产生的物理 PWM，尚未叠加上级前馈或手动覆盖。
float localElevatorBasePWM;
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
const int servo6Pin = 9;
const int servo7Pin = 9;

// 舵机或电调 PWM 输出对象。
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

// 在指定硬件版本上电阶段将 5 号引脚写为低，并延时等待执行器供电稳定。
void prepareActuatorPower() {
  // 指定硬件版本在绑定舵机前，先将 5 号引脚写为低并等待 100 ms。
#if defined expensive
  digitalWrite(5, LOW);
  delay(100);
#endif
}

// 绑定各舵机和电调引脚，配置 900～2100 μs 的输出脉宽范围。
void attachActuators() {
  // 指定各路执行器的有效脉宽范围，单位 μs。
  servo1.attach(servo1Pin, 900, 2100);// 左副翼
  servo2.attach(servo2Pin, 900, 2100);// 右副翼
  servo3.attach(servo3Pin, 900, 2100);// 升降
  servo4.attach(servo4Pin, 900, 2100);// 油门
  servo5.attach(servo5Pin, 900, 2100);// 方向
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);
}

// 上电后先将舵面置中位、油门及预留通道置零。
void commandSafeActuatorPositions() {
  // 上电初始化：舵面回中，油门与预留通道置低。
  servo1.write(90);// 左副翼
  servo2.write(90);// 右副翼
  servo3.write(90);// 升降
  servo4.write(0);// 油门
  servo5.write(90);// 方向
  servo6.write(0);
  servo7.write(0);
}

// 将归一化控制量换算为微秒尺度的逻辑控制量；此阶段尚未应用 rev 或 trim。
void scaleCommands() {
  // *_control_us 以 1500 为逻辑零点，不是最终物理 PWM。
  // 对输出做行程限幅，避免超过执行器允许范围。
  Aail1_control_us = PWM_CENTER_US + 1000 * (Aail1_scaled);
  Aail2_control_us = PWM_CENTER_US + 1000 * (Aail2_scaled);
  Aele_control_us = PWM_CENTER_US + 1000 * (Aele_scaled);
  Athro_control_us = 1100 + 1000 * (Athro_scaled);
  Arudd_control_us = PWM_CENTER_US + 1000 * (Arudd_scaled);

  Bail1_control_us = PWM_CENTER_US + 1000 * (Bail1_scaled);
  Bail2_control_us = PWM_CENTER_US + 1000 * (Bail2_scaled);
  Bthro_control_us = 1100 + 1000 * (Bthro_scaled);
  Brudd_control_us = PWM_CENTER_US + 1000 * (Brudd_scaled);
  B_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Bail1_scaled + Bail2_scaled) / 2;

  Cail1_control_us = PWM_CENTER_US + 1000 * (Cail1_scaled);
  Cail2_control_us = PWM_CENTER_US + 1000 * (Cail2_scaled);
  Cthro_control_us = 1100 + 1000 * (Cthro_scaled);
  Crudd_control_us = PWM_CENTER_US + 1000 * (Crudd_scaled);
  C_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Cail1_scaled + Cail2_scaled) / 2;

  Dail1_control_us = PWM_CENTER_US + 1000 * (Dail1_scaled);
  Dail2_control_us = PWM_CENTER_US + 1000 * (Dail2_scaled);
  Dthro_control_us = 1100 + 1000 * (Dthro_scaled);
  Drudd_control_us = PWM_CENTER_US + 1000 * (Drudd_scaled);
  D_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Dail1_scaled + Dail2_scaled) / 2;

  Eail1_control_us = PWM_CENTER_US + 1000 * (Eail1_scaled);
  Eail2_control_us = PWM_CENTER_US + 1000 * (Eail2_scaled);
  Ethro_control_us = 1100 + 1000 * (Ethro_scaled);
  Erudd_control_us = PWM_CENTER_US + 1000 * (Erudd_scaled);
  E_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Eail1_scaled + Eail2_scaled) / 2;

  Fail1_control_us = PWM_CENTER_US + 1000 * (Fail1_scaled);
  Fail2_control_us = PWM_CENTER_US + 1000 * (Fail2_scaled);
  Fthro_control_us = 1100 + 1000 * (Fthro_scaled);
  Frudd_control_us = PWM_CENTER_US + 1000 * (Frudd_scaled);
  F_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Fail1_scaled + Fail2_scaled) / 2;

  Gail1_control_us = PWM_CENTER_US + 1000 * (Gail1_scaled);
  Gail2_control_us = PWM_CENTER_US + 1000 * (Gail2_scaled);
  Gthro_control_us = 1100 + 1000 * (Gthro_scaled);
  Grudd_control_us = PWM_CENTER_US + 1000 * (Grudd_scaled);
  G_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Gail1_scaled + Gail2_scaled) / 2;

  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;
  // 保留原有逻辑控制量行程限制。
  Aail1_control_us = constrain(Aail1_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  Aail2_control_us = constrain(Aail2_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  Aele_control_us = constrain(Aele_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US); // 贵的飞机限幅
  Athro_control_us = constrain(Athro_control_us, 1100, 1920);
  Arudd_control_us = constrain(Arudd_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);
}

// 准备最终物理 PWM 与子机命令；apply 阶段只应用本函数的结果。
void prepareActuatorCommands() {
  prepareLocalElevatorCommand();
#if defined APLANE
  // 围绕 1500 μs 中位叠加升降补偿、刹车和各机微调。
  // 本函数计算待发送指令；实际 PWM 输出由 applyAndTransmitActuatorCommands() 完成。

  // A机
  int deviation1 = (Aail1_control_us - PWM_CENTER_US) * 0.4;
  int deviation2 = (Aail2_control_us - PWM_CENTER_US) * 0.4;
  int deviation3 = (Aele_control_us - PWM_CENTER_US);
  int deviation4 = Athro_control_us - PWM_CENTER_US;
  int deviation5 = Arudd_control_us - PWM_CENTER_US;
  deviation1 =
      (Aail1_control_us + deviation3 / 6.0 - PWM_CENTER_US) * 0.4; // 升降舵负升力襟翼补偿
  deviation2 = (Aail2_control_us + deviation3 / 6.0 - PWM_CENTER_US) * 0.4;

  // 刹车量与副翼偏移使用相同的方向约定，在 rev 反向前叠加。
  if (channel_8_pwm > 1600) {
    ailBrake_PWM = 0.0;
  } else {
    ailBrake_PWM = 100.0;
  }

  Aail1_PWM =
      PWM_CENTER_US + pwm_channel1_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM + Ail_Clp);
  Aail2_PWM =
      PWM_CENTER_US + pwm_channel2_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM + Ail_Clp);
  Athro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Arudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  // B机
  deviation1 = (Bail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Bail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Bthro_control_us - PWM_CENTER_US;
  deviation5 = Brudd_control_us - PWM_CENTER_US;

  Bail1_PWM = PWM_CENTER_US + pwm_channel1B_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM);
  Bail2_PWM = PWM_CENTER_US + pwm_channel2B_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM);
  Bthro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Brudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  B_ele_command_PWM_Manual =
      PWM_CENTER_US + pwm_channel3B_trim +
      pwm_channel3_rev * (B_ele_control_us_manual - PWM_CENTER_US);
  B_ele_command_PWM_FF = pwm_channel3_rev * B_ele_control_ff_us;
  setLeftChildCommand(0, int(Bail1_PWM),
                      int(Bail2_PWM), int(Bthro_PWM),
                      int(Brudd_PWM), B_pitch_sp,
                      int(B_ele_command_PWM_Manual),
                      int(B_ele_command_PWM_FF));

  // C机
  deviation1 = (Cail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Cail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Cthro_control_us - PWM_CENTER_US;
  deviation5 = Crudd_control_us - PWM_CENTER_US;

  Cail1_PWM = PWM_CENTER_US + pwm_channel1C_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM);
  Cail2_PWM = PWM_CENTER_US + pwm_channel2C_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM);
  Cthro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Crudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  C_ele_command_PWM_Manual =
      PWM_CENTER_US + pwm_channel3C_trim +
      pwm_channel3_rev * (C_ele_control_us_manual - PWM_CENTER_US);
  C_ele_command_PWM_FF = pwm_channel3_rev * C_ele_control_ff_us;
  setRightChildCommand(0, int(Cail1_PWM),
                       int(Cail2_PWM), int(Cthro_PWM),
                       int(Crudd_PWM), C_pitch_sp,
                       int(C_ele_command_PWM_Manual),
                       int(C_ele_command_PWM_FF));
  // D机
  deviation1 = (Dail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Dail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Dthro_control_us - PWM_CENTER_US;
  deviation5 = Drudd_control_us - PWM_CENTER_US;

  Dail1_PWM = PWM_CENTER_US + pwm_channel1D_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM);
  Dail2_PWM = PWM_CENTER_US + pwm_channel2D_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM);
  Dthro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Drudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  D_ele_command_PWM_Manual =
      PWM_CENTER_US + pwm_channel3D_trim +
      pwm_channel3_rev * (D_ele_control_us_manual - PWM_CENTER_US);
  D_ele_command_PWM_FF = pwm_channel3_rev * D_ele_control_ff_us;
  setLeftChildCommand(1, int(Dail1_PWM),
                      int(Dail2_PWM), int(Dthro_PWM),
                      int(Drudd_PWM), D_pitch_sp,
                      int(D_ele_command_PWM_Manual),
                      int(D_ele_command_PWM_FF));

  // E机
  deviation1 = (Eail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Eail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Ethro_control_us - PWM_CENTER_US;
  deviation5 = Erudd_control_us - PWM_CENTER_US;

  Eail1_PWM = PWM_CENTER_US + pwm_channel1E_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM);
  Eail2_PWM = PWM_CENTER_US + pwm_channel2E_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM);
  Ethro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Erudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  E_ele_command_PWM_Manual =
      PWM_CENTER_US + pwm_channel3E_trim +
      pwm_channel3_rev * (E_ele_control_us_manual - PWM_CENTER_US);
  E_ele_command_PWM_FF = pwm_channel3_rev * E_ele_control_ff_us;
  setRightChildCommand(1, int(Eail1_PWM),
                       int(Eail2_PWM), int(Ethro_PWM),
                       int(Erudd_PWM), E_pitch_sp,
                       int(E_ele_command_PWM_Manual),
                       int(E_ele_command_PWM_FF));

  // F机
  deviation1 = (Fail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Fail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Fthro_control_us - PWM_CENTER_US;
  deviation5 = Frudd_control_us - PWM_CENTER_US;

  Fail1_PWM = PWM_CENTER_US + pwm_channel1F_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM);
  Fail2_PWM = PWM_CENTER_US + pwm_channel2F_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM);
  Fthro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Frudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  F_ele_command_PWM_Manual =
      PWM_CENTER_US + pwm_channel3F_trim +
      pwm_channel3_rev * (F_ele_control_us_manual - PWM_CENTER_US);
  F_ele_command_PWM_FF = pwm_channel3_rev * F_ele_control_ff_us;
  setLeftChildCommand(2, int(Fail1_PWM),
                      int(Fail2_PWM), int(Fthro_PWM),
                      int(Frudd_PWM), F_pitch_sp,
                      int(F_ele_command_PWM_Manual),
                      int(F_ele_command_PWM_FF));

  // G机
  deviation1 = (Gail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Gail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Gthro_control_us - PWM_CENTER_US;
  deviation5 = Grudd_control_us - PWM_CENTER_US;

  Gail1_PWM = PWM_CENTER_US + pwm_channel1G_trim + pwm_channel1_rev * (deviation1 + ailBrake_PWM);
  Gail2_PWM = PWM_CENTER_US + pwm_channel2G_trim + pwm_channel2_rev * (deviation2 + ailBrake_PWM);
  Gthro_PWM = PWM_CENTER_US + pwm_channel4_trim + pwm_channel4_rev * deviation4;
  Grudd_PWM = PWM_CENTER_US + pwm_channel5_trim + pwm_channel5_rev * deviation5;

  G_ele_command_PWM_Manual =
      PWM_CENTER_US + pwm_channel3G_trim +
      pwm_channel3_rev * (G_ele_control_us_manual - PWM_CENTER_US);
  G_ele_command_PWM_FF = pwm_channel3_rev * G_ele_control_ff_us;
  setRightChildCommand(2, int(Gail1_PWM),
                       int(Gail2_PWM), int(Gthro_PWM),
                       int(Grudd_PWM), G_pitch_sp,
                       int(G_ele_command_PWM_Manual),
                       int(G_ele_command_PWM_FF));
  Aele_PWM = localElevatorBasePWM;
#else
  // 上级下发的 Local_*_PWM 已应用安装 trim 与反向，不再次转换。
  // 将本地升降舵物理偏移还原为控制方向；零控制量时补偿为零。
  const float elevatorControlDeviation =
      (localElevatorBasePWM - PWM_CENTER_US - pwm_channel3_trim) / pwm_channel3_rev;
  const float elevatorAileronCompensation = elevatorControlDeviation / 15.0f;
  Aail1_PWM = Local_ail1_PWM +
      pwm_channel1_rev * (elevatorAileronCompensation + Ail_Clp);
  Aail2_PWM = Local_ail2_PWM +
      pwm_channel2_rev * (elevatorAileronCompensation + Ail_Clp);
  Aele_PWM = currentMode == MANUAL_MODE ? Local_ele_PWM :
      localElevatorBasePWM + Local_ele_ff_PWM;
  Athro_PWM = Local_thro_PWM;
  Arudd_PWM = Local_rudd_PWM;
#endif
}

namespace {
// 主从机统一准备本地俯仰控制产生的升降舵物理 PWM。
void prepareLocalElevatorCommand() {
#if defined TESTINDI
  if (currentMode == STABILIZE_MODE) {
    // INDI 已返回物理 PWM，包含中位与安装 trim，不再重复反向或加 trim。
    localElevatorBasePWM = PITCH_INDI_control();
    return;
  }
#endif
  // 其他模式沿用 PID/手动混控的逻辑 PWM 到实际 PWM 的转换。
  const int deviation = Aele_control_us - PWM_CENTER_US;
  localElevatorBasePWM = PWM_CENTER_US + pwm_channel3_trim + pwm_channel3_rev * deviation;
}

} // namespace

// 直接应用已准备好的物理 PWM，再发送或转发机间数据。
void applyAndTransmitActuatorCommands() {
  servo1.writeMicroseconds(Aail1_PWM);
  servo2.writeMicroseconds(Aail2_PWM);
  servo3.writeMicroseconds(Aele_PWM);
  servo4.writeMicroseconds(Athro_PWM);
  servo5.writeMicroseconds(Arudd_PWM);
#if defined APLANE
  sendPreparedChildCommands(int_is_valid);
#else
  sendGYROxANGLE();
  forwardReceivedChildCommands(int_is_valid);
#endif
}
