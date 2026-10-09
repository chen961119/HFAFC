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

  commandSafeActuatorPositions();
}

// 上电和锁定时的固定物理位置，不应用 rev 或控制量。
void commandSafeActuatorPositions() {
  servo1.writeMicroseconds(lockedActuatorPwm(pwm_channel1_trim));
  servo2.writeMicroseconds(lockedActuatorPwm(pwm_channel2_trim));
  servo3.writeMicroseconds(lockedActuatorPwm(pwm_channel3_trim));
  servo4.writeMicroseconds(lockedActuatorPwm(pwm_channel4_trim));
  servo5.writeMicroseconds(lockedActuatorPwm(pwm_channel5_trim));
  servo6.writeMicroseconds(PWM_SERVO_MIN_US);
  servo7.writeMicroseconds(PWM_SERVO_MIN_US);
}

// 将归一化控制指令转换为微秒尺度的逻辑 PWM；此阶段尚未应用 rev 或 trim。
void convertControlCommandsToPWM() {
  // *_control_us 以 1500 为逻辑零点，不是最终物理 PWM。
  // 对输出做行程限幅，避免超过执行器允许范围。
  Aail1_control_us = PWM_CENTER_US + 1000 * (Aail1_scaled);
  Aail2_control_us = PWM_CENTER_US + 1000 * (Aail2_scaled);
  Aele_control_us = PWM_CENTER_US + 1000 * (Aele_scaled);
  Athro_control_us = PWM_CENTER_US + 1000 * (Athro_scaled);
  Arudd_control_us = PWM_CENTER_US + 1000 * (Arudd_scaled);

  Bail1_control_us = PWM_CENTER_US + 1000 * (Bail1_scaled);
  Bail2_control_us = PWM_CENTER_US + 1000 * (Bail2_scaled);
  Bthro_control_us = PWM_CENTER_US + 1000 * (Bthro_scaled);
  Brudd_control_us = PWM_CENTER_US + 1000 * (Brudd_scaled);
  B_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Bail1_scaled + Bail2_scaled) / 2;

  Cail1_control_us = PWM_CENTER_US + 1000 * (Cail1_scaled);
  Cail2_control_us = PWM_CENTER_US + 1000 * (Cail2_scaled);
  Cthro_control_us = PWM_CENTER_US + 1000 * (Cthro_scaled);
  Crudd_control_us = PWM_CENTER_US + 1000 * (Crudd_scaled);
  C_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Cail1_scaled + Cail2_scaled) / 2;

  Dail1_control_us = PWM_CENTER_US + 1000 * (Dail1_scaled);
  Dail2_control_us = PWM_CENTER_US + 1000 * (Dail2_scaled);
  Dthro_control_us = PWM_CENTER_US + 1000 * (Dthro_scaled);
  Drudd_control_us = PWM_CENTER_US + 1000 * (Drudd_scaled);
  D_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Dail1_scaled + Dail2_scaled) / 2;

  Eail1_control_us = PWM_CENTER_US + 1000 * (Eail1_scaled);
  Eail2_control_us = PWM_CENTER_US + 1000 * (Eail2_scaled);
  Ethro_control_us = PWM_CENTER_US + 1000 * (Ethro_scaled);
  Erudd_control_us = PWM_CENTER_US + 1000 * (Erudd_scaled);
  E_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Eail1_scaled + Eail2_scaled) / 2;

  Fail1_control_us = PWM_CENTER_US + 1000 * (Fail1_scaled);
  Fail2_control_us = PWM_CENTER_US + 1000 * (Fail2_scaled);
  Fthro_control_us = PWM_CENTER_US + 1000 * (Fthro_scaled);
  Frudd_control_us = PWM_CENTER_US + 1000 * (Frudd_scaled);
  F_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Fail1_scaled + Fail2_scaled) / 2;

  Gail1_control_us = PWM_CENTER_US + 1000 * (Gail1_scaled);
  Gail2_control_us = PWM_CENTER_US + 1000 * (Gail2_scaled);
  Gthro_control_us = PWM_CENTER_US + 1000 * (Gthro_scaled);
  Grudd_control_us = PWM_CENTER_US + 1000 * (Grudd_scaled);
  G_ele_control_us_manual = PWM_CENTER_US + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Gail1_scaled + Gail2_scaled) / 2;

  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;
  // 保留原有逻辑控制量行程限制。
  // Aail1_control_us = constrain(Aail1_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  // Aail2_control_us = constrain(Aail2_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  // Aele_control_us = constrain(Aele_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US); // 贵的飞机限幅
  // Athro_control_us = constrain(Athro_control_us, 1000, 2000);
  // Arudd_control_us = constrain(Arudd_control_us, PWM_SURFACE_MIN_US, PWM_SURFACE_MAX_US);
  // s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  // s7_command_PWM = constrain(s7_command_PWM, 0, 180);
}

// 准备最终物理 PWM 与子机命令；apply 阶段只应用本函数的结果。
void prepareActuatorCommands() {
  prepareLocalElevatorCommand();
#if defined APLANE
  // 准备各机控制偏移；本机补偿、rev/trim 在下方共同应用。

  // A机
  int deviation1 = (Aail1_control_us - PWM_CENTER_US) * 0.4;
  int deviation2 = (Aail2_control_us - PWM_CENTER_US) * 0.4;
  int deviation4 = Athro_control_us - PWM_CENTER_US;
  int deviation5 = Arudd_control_us - PWM_CENTER_US;

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
  Local_ele_control_us = Aele_control_us - PWM_CENTER_US;
  Local_ele_ff_control_us = 0;

  // B机
  deviation1 = (Bail1_control_us - PWM_CENTER_US) * 0.4;
  deviation2 = (Bail2_control_us - PWM_CENTER_US) * 0.4;
  deviation4 = Bthro_control_us - PWM_CENTER_US;
  deviation5 = Brudd_control_us - PWM_CENTER_US;

  Bail1_PWM = deviation1 + ailBrake_PWM;
  Bail2_PWM = deviation2 + ailBrake_PWM;
  Bthro_PWM = deviation4;
  Brudd_PWM = deviation5;

  B_ele_command_PWM_Manual = B_ele_control_us_manual - PWM_CENTER_US;
  B_ele_command_PWM_FF = B_ele_control_ff_us;
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

  Cail1_PWM = deviation1 + ailBrake_PWM;
  Cail2_PWM = deviation2 + ailBrake_PWM;
  Cthro_PWM = deviation4;
  Crudd_PWM = deviation5;

  C_ele_command_PWM_Manual = C_ele_control_us_manual - PWM_CENTER_US;
  C_ele_command_PWM_FF = C_ele_control_ff_us;
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

  Dail1_PWM = deviation1 + ailBrake_PWM;
  Dail2_PWM = deviation2 + ailBrake_PWM;
  Dthro_PWM = deviation4;
  Drudd_PWM = deviation5;

  D_ele_command_PWM_Manual = D_ele_control_us_manual - PWM_CENTER_US;
  D_ele_command_PWM_FF = D_ele_control_ff_us;
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

  Eail1_PWM = deviation1 + ailBrake_PWM;
  Eail2_PWM = deviation2 + ailBrake_PWM;
  Ethro_PWM = deviation4;
  Erudd_PWM = deviation5;

  E_ele_command_PWM_Manual = E_ele_control_us_manual - PWM_CENTER_US;
  E_ele_command_PWM_FF = E_ele_control_ff_us;
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

  Fail1_PWM = deviation1 + ailBrake_PWM;
  Fail2_PWM = deviation2 + ailBrake_PWM;
  Fthro_PWM = deviation4;
  Frudd_PWM = deviation5;

  F_ele_command_PWM_Manual = F_ele_control_us_manual - PWM_CENTER_US;
  F_ele_command_PWM_FF = F_ele_control_ff_us;
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

  Gail1_PWM = deviation1 + ailBrake_PWM;
  Gail2_PWM = deviation2 + ailBrake_PWM;
  Gthro_PWM = deviation4;
  Grudd_PWM = deviation5;

  G_ele_command_PWM_Manual = G_ele_control_us_manual - PWM_CENTER_US;
  G_ele_command_PWM_FF = G_ele_control_ff_us;
  setRightChildCommand(2, int(Gail1_PWM),
                       int(Gail2_PWM), int(Gthro_PWM),
                       int(Grudd_PWM), G_pitch_sp,
                       int(G_ele_command_PWM_Manual),
                       int(G_ele_command_PWM_FF));
#endif
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
  const int deviation = Aele_control_us - PWM_CENTER_US;
  localElevatorControlDeviation = deviation;
  localElevatorBasePWM = PWM_CENTER_US + pwm_channel3_trim + pwm_channel3_rev * deviation;
}

} // namespace

// 直接应用已准备好的物理 PWM，再发送或转发机间数据。
void applyAndTransmitActuatorCommands() {
  if (isFlightLocked()) {
    ail1_PWM = lockedActuatorPwm(pwm_channel1_trim);
    ail2_PWM = lockedActuatorPwm(pwm_channel2_trim);
    ele_PWM = lockedActuatorPwm(pwm_channel3_trim);
    thro_PWM = lockedActuatorPwm(pwm_channel4_trim);
    rudd_PWM = lockedActuatorPwm(pwm_channel5_trim);
  } else {
    ail1_PWM = limitActuatorPwm(ail1_PWM, lockedActuatorPwm(pwm_channel1_trim));
    ail2_PWM = limitActuatorPwm(ail2_PWM, lockedActuatorPwm(pwm_channel2_trim));
    ele_PWM = limitActuatorPwm(ele_PWM, lockedActuatorPwm(pwm_channel3_trim));
    thro_PWM = limitActuatorPwm(thro_PWM, lockedActuatorPwm(pwm_channel4_trim));
    rudd_PWM = limitActuatorPwm(rudd_PWM, lockedActuatorPwm(pwm_channel5_trim));
  }
  servo1.writeMicroseconds(ail1_PWM);
  servo2.writeMicroseconds(ail2_PWM);
  servo3.writeMicroseconds(ele_PWM);
  servo4.writeMicroseconds(thro_PWM);
  servo5.writeMicroseconds(rudd_PWM);
#if defined APLANE
  sendPreparedChildCommands(int_is_valid);
#else
  sendGYROxANGLE();
  forwardReceivedChildCommands(int_is_valid);
#endif
}
