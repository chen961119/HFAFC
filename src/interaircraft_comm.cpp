#include "interaircraft_comm.h"
#include "flight_config.h"
#include "control_state.h"
#include "sensor_processing.h"

float rollAB_rad_Qua, pitchAB_rad_Qua, yawAB_rad_Qua;

// Flight-control inputs shared with the communication layer.
extern float relativeAngle_ready, phiac, phibd, phice, phidf, phieg;
extern float roll_IMU, pitch_IMU, GyroX_9250;

struct ReceivedCommandData {
  int servo[12];
  float pitch[3];
  int ele_pwm[3];
  int ele_ff_pwm[3];
  bool lights[3];
  bool int_is_valid;
  bool Force_manual;
};

// 向A的左发 F<-D<-B<-A
static int servoCommandsleft[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float pitchAnglesleft[3] = {0, 0, 0};
static int elePwmCommandsLeft[3] = {1500, 1500, 1500};
static int eleFFPwmCommandsLeft[3] = {1500, 1500, 1500};
static bool lightSignalsleft[3] = {0, 0, 1};
// 向A的右发 A->C->E->G
static int servoCommandsright[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static float pitchAnglesright[3] = {0, 0, 0};
static int elePwmCommandsRight[3] = {1500, 1500, 1500};
static int eleFFPwmCommandsRight[3] = {1500, 1500, 1500};
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
int Local_ail1_PWM, Local_ail2_PWM, Local_thro_PWM, Local_rudd_PWM,
    Local_ele_PWM, Local_ele_ff_PWM;
float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw, phiG_raw;
float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw, thetaG_raw;
int Bele_PWM, Cele_PWM, Dele_PWM, Eele_PWM, Fele_PWM, Gele_PWM;
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

void sendPreparedChildCommands(bool intIsValid) {
  sendAllDataright(servoCommandsright, pitchAnglesright, elePwmCommandsRight,
                   eleFFPwmCommandsRight, lightSignalsright, intIsValid);
  sendAllDataleft(servoCommandsleft, pitchAnglesleft, elePwmCommandsLeft,
                  eleFFPwmCommandsLeft, lightSignalsleft, intIsValid);
}

void forwardReceivedChildCommands(bool intIsValid) {
#if defined BPLANE || defined DPLANE || defined FPLANE
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
#elif defined CPLANE || defined EPLANE || defined GPLANE
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
#endif
}

void beginParentLink() { Serial6.begin(921600); }
void beginChildLinks() {
  Serial5.begin(921600);
  Serial3.begin(921600);
}

void printReceivedData();

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

  // 数据包结构：头(0x55) + 类型(0x71) + 15个int16_t(各2字节) + 校验和 = 15字节
  uint8_t buffer[33];
  uint8_t pos = 0;
  uint8_t checksum = 0;

#if defined BPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = relativeAngle_ready;
  RelativeAngleAll[2] = phidf;
  GYROAll[0] = GyroX;                      // 自己
  GYROAll[1] = GYRO_X_D;                   // 听来的
  GYROAll[2] = GYRO_X_F;                   // 听来的
  PHIALL[0] = roll_IMU;                    // 自己
  PHIALL[1] = phiD_raw;                    // 听来的
  PHIALL[2] = phiF_raw;                    // 听来的
  THETAALL[0] = pitch_IMU;                 // 自己
  THETAALL[1] = thetaD_raw;                // 听来的
  THETAALL[2] = thetaF_raw;                // 听来的
  ELEPWM[0] = Aele_PWM + Local_ele_ff_PWM; // 自己
  ELEPWM[1] = Dele_PWM;                    // 听来的
  ELEPWM[2] = Fele_PWM;                    // 听来的

#elif defined CPLANE
  RelativeAngleAll[0] = relativeAngle_ready;
  RelativeAngleAll[1] = phice;
  RelativeAngleAll[2] = phieg;
  GYROAll[0] = GyroX;                      // 自己
  GYROAll[1] = GYRO_X_E;                   // 听来的
  GYROAll[2] = GYRO_X_G;                   // 听来的
  PHIALL[0] = roll_IMU;                    // 自己
  PHIALL[1] = phiE_raw;                    // 听来的
  PHIALL[2] = phiG_raw;                    // 听来的
  THETAALL[0] = pitch_IMU;                 // 自己
  THETAALL[1] = thetaE_raw;                // 听来的
  THETAALL[2] = thetaG_raw;                // 听来的
  ELEPWM[0] = Aele_PWM + Local_ele_ff_PWM; // 自己
  ELEPWM[1] = Eele_PWM;                    // 听来的
  ELEPWM[2] = Gele_PWM;                    // 听来的

#elif defined DPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = relativeAngle_ready;
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
  ELEPWM[1] = Aele_PWM + Local_ele_ff_PWM; // 自己的
  ELEPWM[2] = Fele_PWM;                    // 听来的
#elif defined EPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = relativeAngle_ready;
  RelativeAngleAll[2] = phieg;
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
  ELEPWM[1] = Aele_PWM + Local_ele_ff_PWM; // 自己的
  ELEPWM[2] = Gele_PWM;                    // 听来的
#elif defined FPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = relativeAngle_ready;
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
  ELEPWM[2] = Aele_PWM + Local_ele_ff_PWM; // 听来的
#elif defined GPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = relativeAngle_ready;
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
  ELEPWM[2] = Aele_PWM + Local_ele_ff_PWM; // 听来的
#endif

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
  Serial6.write(buffer, sizeof(buffer));
}


static void sendAllDataleft(int servoCommandsleft[12], float pitchAnglesleft[3],
                     int elePwmLeft[3], int eleFFPwmLeft[3],
                     bool lightSignalsleft[3], bool int_is_valid) {
  float invFreq2 = 1.0 / transfreq * 1000000.0;
  unsigned long checker2 = micros();
#if defined APLANE
  force_manual = (currentMode == MANUAL_MODE);
#else
  force_manual = recvData.Force_manual;
#endif

  if (checker2 - lasttransTime < invFreq2)
    return;
  lasttransTime = checker2;
  // 固定大小缓冲区：2头字节 + 1空速有效bool + 1强制手动bool + 12舵机×2 +
  // 3角度×2 + 3升降舵PWM×2 + 3升降舵前馈PWM×2 + 3灯光×1 + 1校验和 = 50字节
  uint8_t buffer[50];
  uint8_t checksum = 0;
  int pos = 0;

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x60;
  checksum += 0x60;

  // 1.5 空速有效
  buffer[pos++] = int_is_valid ? 0x01 : 0x00; // NEW: 添加这行
  checksum += buffer[pos - 1];                // NEW: 添加这行

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

  // 4. 升降舵PWM(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = elePwmLeft[i] & 0xFF;
    buffer[pos++] = (elePwmLeft[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 5. 升降舵前馈PWM(3个)
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
  Serial3.write(buffer, sizeof(buffer));
}

static void sendAllDataright(int servoCommandsright[12], float pitchAnglesright[3],
                      int elePwmRight[3], int eleFFPwmRight[3],
                      bool lightSignalsright[3], bool int_is_valid) {
  float invFreq3 = 1.0 / transfreq * 1000000.0;
  unsigned long checker3 = micros();
#if defined APLANE
  force_manual = (currentMode == MANUAL_MODE);
#else
  force_manual = recvData.Force_manual;
#endif

  if (checker3 - lasttransTime3 < invFreq3)
    return;
  lasttransTime3 = checker3;

  // 固定大小缓冲区：2头字节 + 1空速有效bool + 1强制手动bool + 12舵机×2 +
  // 3角度×2 + 3升降舵PWM×2 + 3升降舵前馈PWM×2 + 3灯光×1 + 1校验和 = 50字节
  uint8_t buffer[50];
  uint8_t checksum = 0;
  int pos = 0;

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x60;
  checksum += 0x60;

  // 1.5 空速有效
  buffer[pos++] = int_is_valid ? 0x01 : 0x00; // NEW: 添加这行
  checksum += buffer[pos - 1];                // NEW: 添加这行

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

  // 4. 升降舵PWM(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = elePwmRight[i] & 0xFF;
    buffer[pos++] = (elePwmRight[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 5. 升降舵前馈PWM(3个)
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
  Serial5.write(buffer, sizeof(buffer));
}

// 接受指令数据
void receiveCommandData() {

  static uint8_t buffer[50];
  static uint8_t pos = 0;

  while (Serial6.available()) {
    uint8_t byte = Serial6.read();
    // Serial.println("ss");
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
      // Serial.println("ok");
      //  计算校验和 (0x55 + 0x60 + 数据字节)
      uint8_t checksum = 0x55 + 0x60;
      for (int i = 2; i < 49; i++) {
        checksum += buffer[i];
      }
      // Serial.print(checksum);
      // Serial.print(" ");
      // Serial.println(buffer[49]);
      //  验证校验和
      if (checksum == buffer[49]) {
        int bufPos = 2; // 数据起始位置

        // 新增：提取int_is_valid字段  // NEW: 添加这行
        recvData.int_is_valid = (buffer[bufPos] == 0x01); // NEW: 添加这行
        bufPos += 1;                                      // NEW: 添加这行
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

        // 3. 提取升降舵PWM (3个int16_t)
        for (int i = 0; i < 3; i++) {
          recvData.ele_pwm[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 4. 提取升降舵前馈PWM (3个int16_t)
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
#if defined BPLANE || defined CPLANE
        Local_ail1_PWM = recvData.servo[0];
        Local_ail2_PWM = recvData.servo[1];
        Local_thro_PWM = recvData.servo[2];
        Local_rudd_PWM = recvData.servo[3];
        Local_pitch_des = recvData.pitch[0];
        Local_ele_PWM = recvData.ele_pwm[0];
        Local_ele_ff_PWM = recvData.ele_ff_pwm[0];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
#elif defined DPLANE || defined EPLANE
        Local_ail1_PWM = recvData.servo[4];
        Local_ail2_PWM = recvData.servo[5];
        Local_thro_PWM = recvData.servo[6];
        Local_rudd_PWM = recvData.servo[7];
        Local_pitch_des = recvData.pitch[1];
        Local_ele_PWM = recvData.ele_pwm[1];
        Local_ele_ff_PWM = recvData.ele_ff_pwm[1];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
#elif defined FPLANE || defined GPLANE
        Local_ail1_PWM = recvData.servo[8];
        Local_ail2_PWM = recvData.servo[9];
        Local_thro_PWM = recvData.servo[10];
        Local_rudd_PWM = recvData.servo[11];
        Local_pitch_des = recvData.pitch[2];
        Local_ele_PWM = recvData.ele_pwm[2];
        Local_ele_ff_PWM = recvData.ele_ff_pwm[2];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
#endif

        printReceivedData();
      }
    }
  }
}

void printReceivedData() {
  static uint32_t frameCount = 0;
  Serial.printf("\n=== 帧#%d ===\n", ++frameCount);
  Serial.printf("int_valid:%d force_manual:%d\n", recvData.int_is_valid ? 1 : 0,
                recvData.Force_manual ? 1 : 0);

  // 舵机指令
  Serial.print("舵机: ");
  for (int i = 0; i < 12; i++)
    Serial.printf("%d ", recvData.servo[i]);

  // 俯仰角
  Serial.print("\n俯仰: ");
  for (int i = 0; i < 3; i++)
    Serial.printf("%.3f ", recvData.pitch[i]);

  // 升降舵PWM
  Serial.print("\nELEPWM: ");
  for (int i = 0; i < 3; i++)
    Serial.printf("%d ", recvData.ele_pwm[i]);

  Serial.print("\nELEFFPWM: ");
  for (int i = 0; i < 3; i++)
    Serial.printf("%d ", recvData.ele_ff_pwm[i]);

  // 灯光
  Serial.print("\n灯光: ");
  for (int i = 0; i < 3; i++)
    Serial.print(recvData.lights[i] ? "1 " : "0 ");

  Serial.println("\n=============");
}

void getGYROxANGLEleft() {
  static uint8_t buffer[33];
  static uint8_t pos = 0;

  float relativeangleleft[3];
  float pleft[3];
  float phi_raw[3];
  float theta_raw[3];
  float ele_pwm[3];
  while (Serial3.available()) {
    uint8_t byte = Serial3.read();

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
        // Serial.println("ok");
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
        thetaB_raw = theta_raw[0];
        thetaD_raw = theta_raw[1];
        thetaF_raw = theta_raw[2];
        Bele_PWM = ele_pwm[0];
        Dele_PWM = ele_pwm[1];
        Fele_PWM = ele_pwm[2];
      }
    }
  }
}

void getGYROxANGLEright() {
  static uint8_t buffer[33]; // 2+1+6+6+6+6+6;
  static uint8_t pos = 0;

  float relativeangleright[3];
  float pright[3];
  float phi_raw[3];
  float theta_raw[3];
  float ele_pwm[3];
  while (Serial5.available()) {
    uint8_t byte = Serial5.read();

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
        thetaC_raw = theta_raw[0];
        thetaE_raw = theta_raw[1];
        thetaG_raw = theta_raw[2];
        Cele_PWM = ele_pwm[0];
        Eele_PWM = ele_pwm[1];
        Gele_PWM = ele_pwm[2];
      }
    }
  }
}

void receiveAdjacentAircraftStates() {
// 信号传输
#if defined APLANE // 主机
  // 主机接收两边，但是从机接收一边
  getGYROxANGLEright(); // 得到三个p，3个转角 计算出pac pce peg
  getGYROxANGLEleft();  // 得到三个p，2个转角 计算出pab pbd pdf
  // telemetry(); //数传。 传输等效姿态角，相对转角，相对扭转角 10hz

#elif defined BPLANE || defined DPLANE || defined FPLANE // 是左边从机
  getGYROxANGLEleft(); // 得到三个p，2个转角 计算出pab pbd pdf
#elif defined CPLANE || defined EPLANE || defined GPLANE // 是右边从机
  getGYROxANGLEright(); // 得到三个p，2个转角 计算出pac pce peg
#endif


}
