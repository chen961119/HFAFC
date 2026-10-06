#include "actuator_output.h"
#include "control_modes.h"
#include "debug_print.h"
#include "flight_config.h"
#include "control_state.h"
#include "interaircraft_comm.h"
#include <Servo.h>

// OneShot125 ESC pin outputs:
const int m1Pin = 37;
const int m2Pin = 37;
const int m3Pin = 38;
const int m4Pin = 39;
const int m5Pin = 40;
const int m6Pin = 41;

const int servo1Pin = 2; // 左副翼
const int servo2Pin = 3; // 右副翼
const int servo3Pin = 4; // 升降
const int servo4Pin = 5; // 油门
const int servo5Pin = 6; // 方向
const int servo6Pin = 9;
const int servo7Pin = 9;
Servo servo1; // Create servo objects to control a servo or ESC with PWM
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

void prepareActuatorPower() {
#if defined expensive
  digitalWrite(5, LOW);
  delay(100);
#endif
}

void attachActuators() {
  servo1.attach(servo1Pin, 900, 2100);
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);
}

void commandSafeActuatorPositions() {
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(0);
  servo5.write(90);
  servo6.write(0);
  servo7.write(0);
}

void prepareActuatorCommands() {
  // 贵的飞机做了舵面限幅
  int deviation1 = (Aail1_PWM - 1520) * 0.4;
  int deviation2 = (Aail2_PWM - 1520) * 0.4;
  int deviation3 = (Aele_PWM - 1520);
  int deviation4 = Athro_PWM - 1520;
  int deviation5 = Arudd_PWM - 1520;
  deviation1 =
      (Aail1_PWM + deviation3 / 6.0 - 1520) * 0.4; // 升降舵负升力襟翼补偿
  deviation2 = (Aail2_PWM + deviation3 / 6.0 - 1520) * 0.4;

  // 增加刹车
  if (channel_8_pwm > 1600) {
    ailBrake_PWM = 00.0;
  } else {
    ailBrake_PWM = 100.0;
  }

  Aail1_PWM =
      1520 + pwm_channel1_rev * deviation1 + pwm_channel1_trim - ailBrake_PWM;
  Aail2_PWM =
      1520 + pwm_channel2_rev * deviation2 + pwm_channel2_trim + ailBrake_PWM;
  Aele_PWM = 1520 + pwm_channel3_rev * deviation3 + pwm_channel3_trim;
  Athro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Arudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  // B机

  deviation1 = (Bail1_PWM - 1520) * 0.4;
  deviation2 = (Bail2_PWM - 1520) * 0.4;
  deviation4 = Bthro_PWM - 1520;
  deviation5 = Brudd_PWM - 1520;

  Bail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Bail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Bthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Brudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  B_ele_command_PWM_Manual =
      (B_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  setLeftChildCommand(0, int(Bail1_PWM + pwm_channel1B_trim),
                      int(Bail2_PWM + pwm_channel2B_trim), int(Bthro_PWM),
                      int(Brudd_PWM), B_pitch_sp,
                      int(B_ele_command_PWM_Manual + pwm_channel3B_trim),
                      int(B_ele_command_PWM_FF));

  // C机
  deviation1 = (Cail1_PWM - 1520) * 0.4;
  deviation2 = (Cail2_PWM - 1520) * 0.4;
  deviation4 = Cthro_PWM - 1520;
  deviation5 = Crudd_PWM - 1520;

  Cail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Cail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Cthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Crudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  C_ele_command_PWM_Manual =
      (C_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  setRightChildCommand(0, int(Cail1_PWM + pwm_channel1C_trim),
                       int(Cail2_PWM + pwm_channel2C_trim), int(Cthro_PWM),
                       int(Crudd_PWM), int(C_pitch_sp),
                       int(C_ele_command_PWM_Manual + pwm_channel3C_trim),
                       int(C_ele_command_PWM_FF));
  // D机
  deviation1 = (Dail1_PWM - 1520) * 0.4;
  deviation2 = (Dail2_PWM - 1520) * 0.4;
  deviation4 = Dthro_PWM - 1520;
  deviation5 = Drudd_PWM - 1520;

  Dail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Dail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Dthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Drudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  D_ele_command_PWM_Manual =
      (D_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  setLeftChildCommand(1, int(Dail1_PWM + pwm_channel1D_trim),
                      int(Dail2_PWM + pwm_channel2D_trim), int(Dthro_PWM),
                      int(Drudd_PWM), D_pitch_sp,
                      int(D_ele_command_PWM_Manual + pwm_channel3D_trim),
                      int(D_ele_command_PWM_FF));

  // E机
  deviation1 = (Eail1_PWM - 1520) * 0.4;
  deviation2 = (Eail2_PWM - 1520) * 0.4;
  deviation4 = Ethro_PWM - 1520;
  deviation5 = Erudd_PWM - 1520;

  Eail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Eail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Ethro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Erudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  E_ele_command_PWM_Manual =
      (E_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  setRightChildCommand(1, int(Eail1_PWM + pwm_channel1E_trim),
                       int(Eail2_PWM + pwm_channel2E_trim), int(Ethro_PWM),
                       int(Erudd_PWM), E_pitch_sp,
                       int(E_ele_command_PWM_Manual + pwm_channel3E_trim),
                       int(E_ele_command_PWM_FF));

  // F机
  deviation1 = (Fail1_PWM - 1520) * 0.4;
  deviation2 = (Fail2_PWM - 1520) * 0.4;
  deviation4 = Fthro_PWM - 1520;
  deviation5 = Frudd_PWM - 1520;

  Fail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Fail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Fthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Frudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  F_ele_command_PWM_Manual =
      (F_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  setLeftChildCommand(2, int(Fail1_PWM + pwm_channel1F_trim),
                      int(Fail2_PWM + pwm_channel2F_trim), int(Fthro_PWM),
                      int(Frudd_PWM), F_pitch_sp,
                      int(F_ele_command_PWM_Manual + pwm_channel3F_trim),
                      int(F_ele_command_PWM_FF));

  // G机
  deviation1 = (Gail1_PWM - 1520) * 0.4;
  deviation2 = (Gail2_PWM - 1520) * 0.4;
  deviation4 = Gthro_PWM - 1520;
  deviation5 = Grudd_PWM - 1520;

  Gail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Gail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Gthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Grudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  G_ele_command_PWM_Manual =
      (G_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  setRightChildCommand(2, int(Gail1_PWM + pwm_channel1G_trim),
                       int(Gail2_PWM + pwm_channel2G_trim), int(Gthro_PWM),
                       int(Grudd_PWM), G_pitch_sp,
                       int(G_ele_command_PWM_Manual + pwm_channel3G_trim),
                       int(G_ele_command_PWM_FF));

}

void applyAndTransmitActuatorCommands() {
#if defined APLANE // 是主机
  // servo1.write(90+40*sin(1*micros()/100000.0));
  // Serial.println(90+40*sin(micros()/100000.0));
  // servo1.writeMicroseconds(1520+400*sin(5*micros()/100000.0)); //左副翼

#if defined TESTINDI
  if (currentMode == STABILIZE_MODE) {
    Aele_PWM = PITCH_INDI_control();
  }
#endif

  servo1.writeMicroseconds(Aail1_PWM + Ail_Clp); // 左副翼
  servo2.writeMicroseconds(Aail2_PWM + Ail_Clp); // 右副翼
  servo3.writeMicroseconds(Aele_PWM);            // 升降
  servo4.writeMicroseconds(Athro_PWM);           // 油门
  servo5.writeMicroseconds(Arudd_PWM);           // 方向

  sendPreparedChildCommands(int_is_valid);

#else // 是从机

#if defined TESTINDI
  if (currentMode == STABILIZE_MODE) {
    Aele_PWM = PITCH_INDI_control();
  }
#endif

  servo1.writeMicroseconds(Local_ail1_PWM + (Aele_PWM - 1520) / 15.0 +
                           Ail_Clp); // 左副翼
  servo2.writeMicroseconds(Local_ail2_PWM - (Aele_PWM - 1520) / 15.0 +
                           Ail_Clp); // 右副翼

  if (currentMode == MANUAL_MODE) {
    servo3.writeMicroseconds(
        Local_ele_PWM); // 强制手动时用主机下发的当地升降舵PWM
  } else {
    servo3.writeMicroseconds(Aele_PWM +
                             Local_ele_ff_PWM); // 非强制手动时保持原逻辑
  }
  servo4.writeMicroseconds(Local_thro_PWM); // 油门

  // 检测从机油门。
  printLocalThrottle();

  servo5.writeMicroseconds(Local_rudd_PWM); // 方向
  sendGYROxANGLE();                         // 发送所有的滚转角速度
  // setQuaternion(); //发送自己的四元数姿态

  forwardReceivedChildCommands(int_is_valid);

#endif

}
