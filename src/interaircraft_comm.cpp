#include "aircraft_config.h"
#include "serial_ports.h"
#include "interaircraft_comm.h"
#include "flight_config.h"
#include "control_state.h"
#include "sensor_processing.h"

float rollAB_rad_Qua, pitchAB_rad_Qua, yawAB_rad_Qua;

// 通信层只引用控制和传感器状态，不持有这些状态。
extern float phiac, phibd, phice, phidf, phieg;
extern float roll_IMU, pitch_IMU, GyroX_9250;

struct ReceivedCommandData {
  // 固定长度 50 字节指令帧解析后的字段；控制偏移单位 μs（不含中位/rev/trim），角度单位 °。
  int servo[12];
  float pitch[3];
  int ele_pwm[3];
  int ele_ff_pwm[3];
  bool lights[3];
  bool int_is_valid;
  bool Force_manual;
};

static bool parentCommandReceived = false;
static bool leftStateReceived = false, rightStateReceived = false;
static unsigned long leftStateTime = 0, rightStateTime = 0;
bool wingStateFresh(bool left) {
  return (left ? leftStateReceived : rightStateReceived) &&
         millis() - (left ? leftStateTime : rightStateTime) <= 1000;
}
bool localWingRelativeAngle(float &angle) {
  if (aircraftIsSingle()) { angle=0;return false; }
if ((aircraftId() == 1)) {
  angle = phiB_raw - roll_IMU; return wingStateFresh(true);
} else if ((aircraftId() == 2)) {
  angle = phiD_raw - roll_IMU; return wingStateFresh(true);
} else if ((aircraftId() == 4)) {
  angle = phiF_raw - roll_IMU; return wingStateFresh(true);
} else if ((aircraftId() == 3)) {
  angle = phiE_raw - roll_IMU; return wingStateFresh(false);
} else if ((aircraftId() == 5)) {
  angle = phiG_raw - roll_IMU; return wingStateFresh(false);
} else {
  angle = 0; return false;
}
}
bool hasReceivedParentCommand() { return parentCommandReceived; }

// 向A的左发 F<-D<-B<-A
static int servoCommandsleft[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float pitchAnglesleft[3] = {0, 0, 0};
static int elePwmCommandsLeft[3] = {};
static int eleFFPwmCommandsLeft[3] = {};
static bool lightSignalsleft[3] = {0, 0, 1};
// 向A的右发 A->C->E->G
static int servoCommandsright[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float pitchAnglesright[3] = {0, 0, 0};
static int elePwmCommandsRight[3] = {};
static int eleFFPwmCommandsRight[3] = {};
static bool lightSignalsright[3] = {0, 0, 1};

// 从机往内发的。
static float RelativeAngleAll[3] = {1.2, 2.4, 4}; // 相对转角 F->D->B->A
static float GYROAll[3] = {5, 3, 1};              // 本体滚转角速度。F->D->B->A
static float PHIALL[3] = {5, 3, 1};               // 本体滚转角度。F->D->B->A
static float THETAALL[3] = {5, 3, 1};             // 本体俯仰角度。F->D->B->A
static int ELEPWM[3] = {1500, 1500, 1500};        // 本体升降舵指令
static float AIRSPEED[3] = {10, 10, 10};          // 本体空速
static ReceivedCommandData recvData;
static uint16_t bufIndex = 0;
static bool frameStarted = false;
float Local_pitch_des;
// 未收到上级命令前保持 1000 μs 的逻辑低油门。
int Local_thro_control_us = 1000 - PWM_CENTER_US;
int Local_ail1_control_us, Local_ail2_control_us, Local_rudd_control_us,
    Local_ele_control_us, Local_ele_ff_control_us;
float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw, phiG_raw;
float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw, thetaG_raw;
int B_ele_PWM, C_ele_PWM, D_ele_PWM, E_ele_PWM, F_ele_PWM, G_ele_PWM;
float GYRO_X_B, GYRO_X_C, GYRO_X_D, GYRO_X_E, GYRO_X_F, GYRO_X_G;
static unsigned long lastsendTime = 0;
static unsigned long lastsendTimeQua = 0;
static unsigned long lasttransTime = 0;
static unsigned long lasttransTime3 = 0;
static unsigned int transfreq = 100;
static unsigned int Freqsendback = 200;

static void sendAllDataleft(int servoCommandsleft[12], float pitchAnglesleft[3],
                            int elePwmLeft[3], int eleFFPwmLeft[3],
                            bool lightSignalsleft[3], bool int_is_valid);
static void sendAllDataright(int servoCommandsright[12], float pitchAnglesright[3],
                             int elePwmRight[3], int eleFFPwmRight[3],
                             bool lightSignalsright[3], bool int_is_valid);

// 把左侧指定子机的副翼、油门、方向舵、俯仰角和升降舵指令写入发送缓存。
// index 为同侧由近到远的 0～2；调用方负责检查范围，控制偏移单位为 μs，不含中位/rev/trim。
void setLeftChildCommand(unsigned int index, int aileron1, int aileron2,
                         int throttle, int rudder, float pitch,
                         int elevatorManual, int elevatorFeedForward) {
  const unsigned int offset = index * 4;
  servoCommandsleft[offset] = aileron1;
  servoCommandsleft[offset + 1] = aileron2;
  servoCommandsleft[offset + 2] = throttle;
  servoCommandsleft[offset + 3] = rudder;
  pitchAnglesleft[index] = pitch;
  elePwmCommandsLeft[index] = elevatorManual;
  eleFFPwmCommandsLeft[index] = elevatorFeedForward;
}

// 把右侧指定子机的舵面和俯仰指令写入发送缓存。
// index 为同侧由近到远的 0～2；调用方负责检查范围，控制偏移单位为 μs，不含中位/rev/trim。
void setRightChildCommand(unsigned int index, int aileron1, int aileron2,
                          int throttle, int rudder, float pitch,
                          int elevatorManual, int elevatorFeedForward) {
  const unsigned int offset = index * 4;
  servoCommandsright[offset] = aileron1;
  servoCommandsright[offset + 1] = aileron2;
  servoCommandsright[offset + 2] = throttle;
  servoCommandsright[offset + 3] = rudder;
  pitchAnglesright[index] = pitch;
  elePwmCommandsRight[index] = elevatorManual;
  eleFFPwmCommandsRight[index] = elevatorFeedForward;
}

// 将左右两侧发送缓存分别打包下发，并携带积分控制有效标志。
void sendPreparedChildCommands(bool intIsValid) {
  if (aircraftIsSingle()) return;
  sendAllDataright(servoCommandsright, pitchAnglesright, elePwmCommandsRight,
                   eleFFPwmCommandsRight, lightSignalsright, intIsValid);
  sendAllDataleft(servoCommandsleft, pitchAnglesleft, elePwmCommandsLeft,
                  eleFFPwmCommandsLeft, lightSignalsleft, intIsValid);
}

// 从机取出本机指令后，保留更远子机的数据并沿所在侧继续转发。
void forwardReceivedChildCommands(bool intIsValid) {
  // 启动时接收缓存全零；零油门偏移代表 1500 μs，不能作为低油门转发。
  // 首帧到达前让下游保持自身 -500 μs 的默认偏移（逻辑 PWM 1000 μs）。
  if (!parentCommandReceived) return;
if ((aircraftId() == 2) || (aircraftId() == 4) || (aircraftId() == 6)) {
  for (unsigned int i = 0; i < 4; ++i) {
    servoCommandsleft[i] = 0;
  }
  elePwmCommandsLeft[0] = 0;
  eleFFPwmCommandsLeft[0] = 0;
  for (unsigned int i = 4; i < 12; ++i) {
    servoCommandsleft[i] = recvData.servo[i];
  }
  for (unsigned int i = 1; i < 3; ++i) {
    pitchAnglesleft[i] = recvData.pitch[i];
    elePwmCommandsLeft[i] = recvData.ele_pwm[i];
    eleFFPwmCommandsLeft[i] = recvData.ele_ff_pwm[i];
  }
  sendAllDataleft(servoCommandsleft, pitchAnglesleft, elePwmCommandsLeft,
                  eleFFPwmCommandsLeft, lightSignalsleft, intIsValid);
} else if ((aircraftId() == 3) || (aircraftId() == 5) || (aircraftId() == 7)) {
  for (unsigned int i = 0; i < 4; ++i) {
    servoCommandsright[i] = 0;
  }
  elePwmCommandsRight[0] = 0;
  eleFFPwmCommandsRight[0] = 0;
  for (unsigned int i = 4; i < 12; ++i) {
    servoCommandsright[i] = recvData.servo[i];
  }
  for (unsigned int i = 1; i < 3; ++i) {
    pitchAnglesright[i] = recvData.pitch[i];
    elePwmCommandsRight[i] = recvData.ele_pwm[i];
    eleFFPwmCommandsRight[i] = recvData.ele_ff_pwm[i];
  }
  sendAllDataright(servoCommandsright, pitchAnglesright, elePwmCommandsRight,
                   eleFFPwmCommandsRight, lightSignalsright, intIsValid);
}
}

// 以 921600 波特率启动面向上一级机体的 ParentSerial。
void beginParentLink() { ParentSerial.begin(921600); }
// 以 921600 波特率启动左右子机所用的 LeftChildSerial 与 RightChildSerial。
void beginChildLinks() {
  RightChildSerial.begin(921600);
  LeftChildSerial.begin(921600);
}

void printReceivedData();

// 从机将本机及更远机体的转角、角速度、姿态和升降舵 PWM 组成 33 字节状态帧。
// 状态按 0.1 单位量化，以设定频率通过 ParentSerial 发往上一级机体。
void sendGYROxANGLE() // 从机要做的
{

  float GyroX;
#if defined USE_MPU6050_I2C
  GyroX = -GyroX_6050; // 安装位置
#endif
#if defined USE_MPU9250_SPI
  GyroX = -GyroX_9250;
#endif

#if defined EXTIMU
  GyroX = Gyro_X_EXT;
#endif

  // 数据包结构：2 字节帧头 + 15 个 int16_t + 1 字节校验和，共 33 字节。
  uint8_t buffer[33];
  uint8_t pos = 0;
  uint8_t checksum = 0;

if ((aircraftId() == 2)) {
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = phiD_raw - roll_IMU;
  RelativeAngleAll[2] = phiF_raw - phiD_raw;
  GYROAll[0] = GyroX;                      // 自己
  GYROAll[1] = GYRO_X_D;                   // 听来的
  GYROAll[2] = GYRO_X_F;                   // 听来的
  PHIALL[0] = roll_IMU;                    // 自己
  PHIALL[1] = phiD_raw;                    // 听来的
  PHIALL[2] = phiF_raw;                    // 听来的
  THETAALL[0] = pitch_IMU;                 // 自己
  THETAALL[1] = thetaD_raw;                // 听来的
  THETAALL[2] = thetaF_raw;                // 听来的
  ELEPWM[0] = ele_PWM; // 自己
  ELEPWM[1] = D_ele_PWM;                    // 听来的
  ELEPWM[2] = F_ele_PWM;                    // 听来的

} else if ((aircraftId() == 3)) {
  RelativeAngleAll[0] = 0.0; // Parent computes C-A from the two rolls.
  RelativeAngleAll[1] = phiE_raw - roll_IMU;
  RelativeAngleAll[2] = phiG_raw - phiE_raw;
  GYROAll[0] = GyroX;                      // 自己
  GYROAll[1] = GYRO_X_E;                   // 听来的
  GYROAll[2] = GYRO_X_G;                   // 听来的
  PHIALL[0] = roll_IMU;                    // 自己
  PHIALL[1] = phiE_raw;                    // 听来的
  PHIALL[2] = phiG_raw;                    // 听来的
  THETAALL[0] = pitch_IMU;                 // 自己
  THETAALL[1] = thetaE_raw;                // 听来的
  THETAALL[2] = thetaG_raw;                // 听来的
  ELEPWM[0] = ele_PWM; // 自己
  ELEPWM[1] = E_ele_PWM;                    // 听来的
  ELEPWM[2] = G_ele_PWM;                    // 听来的

} else if ((aircraftId() == 4)) {
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = phiF_raw - roll_IMU;
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = GyroX;                      // 自己的
  GYROAll[2] = GYRO_X_F;                   // 听来的
  PHIALL[0] = 0.0;                         // 自己
  PHIALL[1] = roll_IMU;                    // 自己
  PHIALL[2] = phiF_raw;                    // 自己
  THETAALL[0] = 0.0;                       // 自己
  THETAALL[1] = pitch_IMU;                 // 自己
  THETAALL[2] = thetaF_raw;                // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = ele_PWM; // 自己的
  ELEPWM[2] = F_ele_PWM;                    // 听来的
} else if ((aircraftId() == 5)) {
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0; // Parent computes E-C from the two rolls.
  RelativeAngleAll[2] = phiG_raw - roll_IMU;
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = GyroX;                      // 自己的
  GYROAll[2] = GYRO_X_G;                   // 听来的
  PHIALL[0] = 0.0;                         // 自己
  PHIALL[1] = roll_IMU;                    // 自己
  PHIALL[2] = phiG_raw;                    // 听来的
  THETAALL[0] = 0.0;                       // 自己
  THETAALL[1] = pitch_IMU;                 // 自己
  THETAALL[2] = thetaG_raw;                // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = ele_PWM; // 自己的
  ELEPWM[2] = G_ele_PWM;                    // 听来的
} else if ((aircraftId() == 6)) {
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = 0.0; // Leaf: no downstream joint measurement.
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = 0.0;                        // 空
  GYROAll[2] = GyroX;                      // 自己的
  PHIALL[0] = 0.0;                         //
  PHIALL[1] = 0.0;                         //
  PHIALL[2] = roll_IMU;                    // 自己
  THETAALL[0] = 0.0;                       //
  THETAALL[1] = 0.0;                       //
  THETAALL[2] = pitch_IMU;                 // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = 0;                           //
  ELEPWM[2] = ele_PWM; // 听来的
} else if ((aircraftId() == 7)) {
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = 0.0; // Leaf: no downstream joint measurement.
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = 0.0;                        // 空
  GYROAll[2] = GyroX;                      // 自己的
  PHIALL[0] = 0.0;                         // 自己
  PHIALL[1] = 0.0;                         // 自己
  PHIALL[2] = roll_IMU;                    // 自己
  THETAALL[0] = 0.0;                       //
  THETAALL[1] = 0.0;                       //
  THETAALL[2] = pitch_IMU;                 // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = 0;                           //
  ELEPWM[2] = ele_PWM; // 听来的
}

  float invFreq = 1.0 / Freqsendback * 1000000.0;
  unsigned long checker2 = micros();
  if (checker2 - lastsendTime < invFreq)
    return;
  lastsendTime = checker2;

  /* 如果代码没问题就改成这个
    // 应该使用unsigned long类型，避免浮点数比较：
  uint32_t interval = 1000000 / Freqsendback;  // 微秒
  if (micros() - lastsendTime < interval) return;
  lastsendTime = micros();
  */

  //  float anglerand = random(-200000, 200001) / 1000.0;  // 范围 -200.000 ~
  //  +200.000

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x71;
  checksum += 0x71; // 新类型标识

  // 1. 打包相对转角 (3个浮点数，保留1位小数)

  // 2. 打包相对转角 (转换为int16_t，保留1位小数)
  for (int i = 0; i < 3; i++) {
    int16_t val = RelativeAngleAll[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 3. 打包角速度 (同样处理)
  for (int i = 0; i < 3; i++) {
    int16_t val = GYROAll[i] * 10.0f;
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 4.打包滚转欧拉角
  for (int i = 0; i < 3; i++) {
    int16_t val = PHIALL[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 5.打包俯仰欧拉角
  for (int i = 0; i < 3; i++) {
    int16_t val = THETAALL[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 5.打包升降舵pwm
  for (int i = 0; i < 3; i++) {
    int16_t val = ELEPWM[i];
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8) & 0xFF;
    checksum += buffer[pos - 1];
  }

  // 6. 校验和
  buffer[pos] = checksum;
  // uint8_t checksum1 = 0x55 + 0x71;
  // for (int i = 2; i < 20; i++) checksum1 += buffer[i];

  // 5. 发送
  ParentSerial.write(buffer, sizeof(buffer));
}


// 将左侧三架子机的控制缓存编码为 50 字节指令帧，经 LeftChildSerial 下发。
// int_is_valid 表示是否允许积分控制；发送频率由 transfreq 限制。
static void sendAllDataleft(int servoCommandsleft[12], float pitchAnglesleft[3],
                     int elePwmLeft[3], int eleFFPwmLeft[3],
                     bool lightSignalsleft[3], bool int_is_valid) {
  float invFreq2 = 1.0 / transfreq * 1000000.0;
  unsigned long checker2 = micros();
if ((aircraftId() == 1)) {
  force_manual = (currentMode == MANUAL_MODE);
} else {
  force_manual = recvData.Force_manual;
}

  if (checker2 - lasttransTime < invFreq2)
    return;
  lasttransTime = checker2;
  // 固定大小缓冲区：2头字节 + 1积分控制有效bool + 1强制手动bool + 12舵机×2 +
  // 3角度×2 + 3手动升降舵偏移×2 + 3升降舵前馈偏移×2 + 3灯光×1 + 1校验和 = 50字节
  uint8_t buffer[50];
  uint8_t checksum = 0;
  int pos = 0;

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x60;
  checksum += 0x60;

  // 1.5 积分控制有效
  buffer[pos++] = int_is_valid ? 0x01 : 0x00; // 积分控制有效标志。
  checksum += buffer[pos - 1];

  // 1.6 强制手动
  buffer[pos++] = force_manual ? 0x01 : 0x00;
  checksum += buffer[pos - 1];

  // 2. 舵机指令(12个) - 直接写入高低字节
  for (int i = 0; i < 12; i++) {
    buffer[pos++] = servoCommandsleft[i] & 0xFF;        // 低字节
    buffer[pos++] = (servoCommandsleft[i] >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 3. 俯仰角(3个) - 直接写入高低字节
  for (int i = 0; i < 3; i++) {
    int16_t pitchAnglesleft_int = pitchAnglesleft[i] * 10.0f;
    buffer[pos++] = pitchAnglesleft_int & 0xFF;        // 低字节
    buffer[pos++] = (pitchAnglesleft_int >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 4. 手动升降舵偏移(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = elePwmLeft[i] & 0xFF;
    buffer[pos++] = (elePwmLeft[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 5. 升降舵前馈偏移(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = eleFFPwmLeft[i] & 0xFF;
    buffer[pos++] = (eleFFPwmLeft[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 6. 灯光信号(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = lightSignalsleft[i] ? 0x01 : 0x00;
    checksum += buffer[pos - 1];
  }

  // 7. 校验和
  buffer[pos] = checksum;
  // 8. 一次性发送
  LeftChildSerial.write(buffer, sizeof(buffer));
}

// 将右侧三架子机的控制缓存编码为 50 字节指令帧，经 RightChildSerial 下发。
// int_is_valid 表示是否允许积分控制；发送频率由 transfreq 限制。
static void sendAllDataright(int servoCommandsright[12], float pitchAnglesright[3],
                      int elePwmRight[3], int eleFFPwmRight[3],
                      bool lightSignalsright[3], bool int_is_valid) {
  float invFreq3 = 1.0 / transfreq * 1000000.0;
  unsigned long checker3 = micros();
if ((aircraftId() == 1)) {
  force_manual = (currentMode == MANUAL_MODE);
} else {
  force_manual = recvData.Force_manual;
}

  if (checker3 - lasttransTime3 < invFreq3)
    return;
  lasttransTime3 = checker3;

  // 固定大小缓冲区：2头字节 + 1积分控制有效bool + 1强制手动bool + 12舵机×2 +
  // 3角度×2 + 3手动升降舵偏移×2 + 3升降舵前馈偏移×2 + 3灯光×1 + 1校验和 = 50字节
  uint8_t buffer[50];
  uint8_t checksum = 0;
  int pos = 0;

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x60;
  checksum += 0x60;

  // 1.5 积分控制有效
  buffer[pos++] = int_is_valid ? 0x01 : 0x00; // 积分控制有效标志。
  checksum += buffer[pos - 1];

  // 1.6 强制手动
  buffer[pos++] = force_manual ? 0x01 : 0x00;
  checksum += buffer[pos - 1];

  // 2. 舵机指令(12个) - 直接写入高低字节
  for (int i = 0; i < 12; i++) {
    buffer[pos++] = servoCommandsright[i] & 0xFF;        // 低字节
    buffer[pos++] = (servoCommandsright[i] >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 3. 俯仰角(3个) - 直接写入高低字节
  for (int i = 0; i < 3; i++) {
    int16_t pitchAnglesright_int = pitchAnglesright[i] * 10.0f;
    buffer[pos++] = pitchAnglesright_int & 0xFF;        // 低字节
    buffer[pos++] = (pitchAnglesright_int >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 4. 手动升降舵偏移(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = elePwmRight[i] & 0xFF;
    buffer[pos++] = (elePwmRight[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 5. 升降舵前馈偏移(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = eleFFPwmRight[i] & 0xFF;
    buffer[pos++] = (eleFFPwmRight[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 6. 灯光信号(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = lightSignalsright[i] ? 0x01 : 0x00;
    checksum += buffer[pos - 1];
  }

  // 7. 校验和
  buffer[pos] = checksum;
  // 8. 一次性发送
  RightChildSerial.write(buffer, sizeof(buffer));
}

// 接受指令数据
// 从 ParentSerial 接收上一级机体的 50 字节指令帧；校验通过后提取本机舵面和模式指令。
void receiveCommandData() {

  static uint8_t buffer[50];
  static uint8_t pos = 0;

  while (ParentSerial.available()) {
    uint8_t byte = ParentSerial.read();
    // USBSerial.println("ss");
    //  检查数据头
    if (pos == 0 && byte != 0x55)
      continue;
    if (pos == 1 && byte != 0x60) {
      pos = 0;
      continue;
    }

    buffer[pos++] = byte;

    // 完整数据包接收
    if (pos == 50) {
      pos = 0;
      // USBSerial.println("ok");
      //  计算校验和 (0x55 + 0x60 + 数据字节)
      uint8_t checksum = 0x55 + 0x60;
      for (int i = 2; i < 49; i++) {
        checksum += buffer[i];
      }
      // USBSerial.print(checksum);
      // USBSerial.print(" ");
      // USBSerial.println(buffer[49]);
      //  验证校验和
      if (checksum == buffer[49]) {
        parentCommandReceived = true;
        int bufPos = 2; // 数据起始位置

        // 依序读取积分控制有效标志与强制手动标志。
        recvData.int_is_valid = (buffer[bufPos] == 0x01);
        bufPos += 1;
        recvData.Force_manual = (buffer[bufPos] == 0x01);
        bufPos += 1;

        // 1. 提取舵机指令 (12个int16_t)
        for (int i = 0; i < 12; i++) {
          recvData.servo[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 2. 提取俯仰角 (3个int16_t) 前面的(int16_t)不能少，另外合适的缩放系数
        // 最大的数字不能超过37268
        for (int i = 0; i < 3; i++) {
          recvData.pitch[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }

        // 3. 提取手动升降舵偏移 (3个int16_t)
        for (int i = 0; i < 3; i++) {
          recvData.ele_pwm[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 4. 提取升降舵前馈偏移 (3个int16_t)
        for (int i = 0; i < 3; i++) {
          recvData.ele_ff_pwm[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 5. 提取灯光信号 (3个bool)
        for (int i = 0; i < 3; i++) {
          recvData.lights[i] = (buffer[bufPos] == 0x01);
          bufPos += 1;
        }

// 拿出自己的
if ((aircraftId() == 2) || (aircraftId() == 3)) {
        Local_ail1_control_us = recvData.servo[0];
        Local_ail2_control_us = recvData.servo[1];
        Local_thro_control_us = recvData.servo[2];
        Local_rudd_control_us = recvData.servo[3];
        Local_pitch_des = recvData.pitch[0];
        Local_ele_control_us = recvData.ele_pwm[0];
        Local_ele_ff_control_us = recvData.ele_ff_pwm[0];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
} else if ((aircraftId() == 4) || (aircraftId() == 5)) {
        Local_ail1_control_us = recvData.servo[4];
        Local_ail2_control_us = recvData.servo[5];
        Local_thro_control_us = recvData.servo[6];
        Local_rudd_control_us = recvData.servo[7];
        Local_pitch_des = recvData.pitch[1];
        Local_ele_control_us = recvData.ele_pwm[1];
        Local_ele_ff_control_us = recvData.ele_ff_pwm[1];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
} else if ((aircraftId() == 6) || (aircraftId() == 7)) {
        Local_ail1_control_us = recvData.servo[8];
        Local_ail2_control_us = recvData.servo[9];
        Local_thro_control_us = recvData.servo[10];
        Local_rudd_control_us = recvData.servo[11];
        Local_pitch_des = recvData.pitch[2];
        Local_ele_control_us = recvData.ele_pwm[2];
        Local_ele_ff_control_us = recvData.ele_ff_pwm[2];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
}

        printReceivedData();
      }
    }
  }
}

// 将最近收到的指令帧各字段打印到调试串口；每调用一次计数加一。
void printReceivedData() {
  static uint32_t frameCount = 0;
  USBSerial.printf("\n=== 帧#%d ===\n", ++frameCount);
  USBSerial.printf("int_valid:%d force_manual:%d\n", recvData.int_is_valid ? 1 : 0,
                recvData.Force_manual ? 1 : 0);

  // 舵机指令
  USBSerial.print("舵机: ");
  for (int i = 0; i < 12; i++)
    USBSerial.printf("%d ", recvData.servo[i]);

  // 俯仰角
  USBSerial.print("\n俯仰: ");
  for (int i = 0; i < 3; i++)
    USBSerial.printf("%.3f ", recvData.pitch[i]);

  // 手动升降舵偏移
  USBSerial.print("\nELEPWM: ");
  for (int i = 0; i < 3; i++)
    USBSerial.printf("%d ", recvData.ele_pwm[i]);

  USBSerial.print("\nELEFFPWM: ");
  for (int i = 0; i < 3; i++)
    USBSerial.printf("%d ", recvData.ele_ff_pwm[i]);

  // 灯光
  USBSerial.print("\n灯光: ");
  for (int i = 0; i < 3; i++)
    USBSerial.print(recvData.lights[i] ? "1 " : "0 ");

  USBSerial.println("\n=============");
}

// 从 LeftChildSerial 接收左侧 33 字节状态帧，更新 B、D、F 机的相对角与姿态状态。
void getGYROxANGLEleft() {
  static uint8_t buffer[33];
  static uint8_t pos = 0;

  float relativeangleleft[3];
  float pleft[3];
  float phi_raw[3];
  float theta_raw[3];
  float ele_pwm[3];
  while (LeftChildSerial.available()) {
    uint8_t byte = LeftChildSerial.read();

    // 检查数据头
    if (pos == 0 && byte != 0x55)
      continue;
    if (pos == 1 && byte != 0x71) {
      pos = 0;
      continue;
    }

    buffer[pos++] = byte;

    // 完整数据包处理
    if (pos == 33) {
      pos = 0;

      // 计算校验和
      uint8_t checksum = 0x55 + 0x71;
      for (int i = 2; i < 32; i++) {
        checksum += buffer[i];
      }

      if (checksum == buffer[32]) {
        int bufPos = 2;
        // USBSerial.println("ok");
        //  提取角度数据
        for (int i = 0; i < 3; i++) {
          relativeangleleft[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }

        // 提取角速度数据
        for (int i = 0; i < 3; i++) {
          pleft[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }
        // 提取当地欧拉角度数据
        for (int i = 0; i < 3; i++) {
          phi_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }
        for (int i = 0; i < 3; i++) {
          theta_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        // 提取舵机指令 (3个int16_t)
        for (int i = 0; i < 3; i++) {
          ele_pwm[i] = (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        phibd = relativeangleleft[1];
        phidf = relativeangleleft[2];
        GYRO_X_B = pleft[0];
        GYRO_X_D = pleft[1];
        GYRO_X_F = pleft[2];
        phiB_raw = phi_raw[0];
        phiD_raw = phi_raw[1];
        phiF_raw = phi_raw[2];
        leftStateReceived = true;
        leftStateTime = millis();
        thetaB_raw = theta_raw[0];
        thetaD_raw = theta_raw[1];
        thetaF_raw = theta_raw[2];
        B_ele_PWM = ele_pwm[0];
        D_ele_PWM = ele_pwm[1];
        F_ele_PWM = ele_pwm[2];
      }
    }
  }
}

// 从 RightChildSerial 接收右侧 33 字节状态帧，更新 C、E、G 机的相对角与姿态状态。
void getGYROxANGLEright() {
  static uint8_t buffer[33]; // 2+1+6+6+6+6+6;
  static uint8_t pos = 0;

  float relativeangleright[3];
  float pright[3];
  float phi_raw[3];
  float theta_raw[3];
  float ele_pwm[3];
  while (RightChildSerial.available()) {
    uint8_t byte = RightChildSerial.read();

    // 检查数据头
    if (pos == 0 && byte != 0x55)
      continue;
    if (pos == 1 && byte != 0x71) {
      pos = 0;
      continue;
    }

    buffer[pos++] = byte;

    // 完整数据包处理
    if (pos == 33) {
      pos = 0;

      // 计算校验和
      uint8_t checksum = 0x55 + 0x71;
      for (int i = 2; i < 32; i++) {
        checksum += buffer[i];
      }

      if (checksum == buffer[32]) {
        int bufPos = 2;

        // 提取角度数据
        for (int i = 0; i < 3; i++) {
          relativeangleright[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        // 提取角速度数据
        for (int i = 0; i < 3; i++) {
          pright[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }
        // 提取当地欧拉角度数据
        for (int i = 0; i < 3; i++) {
          phi_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        for (int i = 0; i < 3; i++) {
          theta_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        // 提取舵机指令 (3个int16_t)
        for (int i = 0; i < 3; i++) {
          ele_pwm[i] = (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        phiac = relativeangleright[0];
        phice = relativeangleright[1];
        phieg = relativeangleright[2];
        GYRO_X_C = pright[0];
        GYRO_X_E = pright[1];
        GYRO_X_G = pright[2];
        phiC_raw = phi_raw[0];
        phiE_raw = phi_raw[1];
        phiG_raw = phi_raw[2];
        rightStateReceived = true;
        rightStateTime = millis();
        thetaC_raw = theta_raw[0];
        thetaE_raw = theta_raw[1];
        thetaG_raw = theta_raw[2];
        C_ele_PWM = ele_pwm[0];
        E_ele_PWM = ele_pwm[1];
        G_ele_PWM = ele_pwm[2];
      }
    }
  }
}

// 按本机位置读取左侧、右侧或两侧相邻机体状态。
void receiveAdjacentAircraftStates() {
  if (aircraftIsSingle()) return;
// 信号传输
if ((aircraftId() == 1)) {
  // 主机接收两边，但是从机接收一边
  getGYROxANGLEright(); // 得到三个p，3个转角 计算出pac pce peg
  getGYROxANGLEleft();  // 得到三个p，2个转角 计算出pab pbd pdf

} else if ((aircraftId() == 2) || (aircraftId() == 4) || (aircraftId() == 6)) {
  getGYROxANGLEleft(); // 得到三个p，2个转角 计算出pab pbd pdf
} else if ((aircraftId() == 3) || (aircraftId() == 5) || (aircraftId() == 7)) {
  getGYROxANGLEright(); // 得到三个p，2个转角 计算出pac pce peg
}


}
