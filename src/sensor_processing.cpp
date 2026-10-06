#include "serial_ports.h"
#include "sensor_processing.h"
#include "flight_config.h"
#include "flight_clock.h"
#include "math_utils.h"
#include "MS4525.h"
#include <EEPROM.h>
#include <Wire.h>
#if defined USE_BAROMETER
#include <Adafruit_BMP280.h>
#define BMP280_I2C Wire2
Adafruit_BMP280 bmp(&BMP280_I2C);
// 滤波参数
#define FILTER_ALPHA 0.2
float filteredPressure = 1013.25; // 初始海平面气压(hPa)
float filteredAltitude = 0;
float temp;
float pressure;
#endif

#if defined USE_MPU6050_I2C && defined USE_MPU9250_SPI
#include <MPU6050.h>
#include <MPU9250.h>
MPU6050 mpu6050;
MPU9250 mpu9250(SPI, 10);
#elif defined USE_MPU9250_SPI
#include <MPU9250.h>
MPU9250 mpu9250(SPI, 10);
#elif defined USE_MPU6050_I2C
#include <MPU6050.h>
MPU6050 mpu6050;
#else
#error No MPU defined...
#endif

//========================================================================================================================//

// 根据飞控配置选择陀螺仪、加速度计的量程与原始值换算系数。

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250_6050 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500_6050 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000_6050 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000_6050 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2_6050 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4_6050 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8_6050 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16_6050 MPU6050_ACCEL_FS_16
#endif
#if defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250_9250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500_9250 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000_9250 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000_9250 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2_9250 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4_9250 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8_9250 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16_9250 mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_250_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_250_9250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_500_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_500_9250
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_1000_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_1000_9250
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_2000_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_2000_9250
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_2_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_2_9250
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_4_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_4_9250
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_8_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_8_9250
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_16_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_16_9250
#define ACCEL_SCALE_FACTOR 2048.0
#endif




// 以 115200 波特率启动外置 IMU 所用的 ExternalImuSerial。
void beginExternalImuLink() { ExternalImuSerial.begin(115200); }
// 以 115200 波特率启动应变传感器所用的 StrainSensorSerial。
void beginStrainSensorLink() { StrainSensorSerial.begin(115200); }

// 在启动阶段反复读取 BMI088 并迭代姿态滤波器，使姿态估计预热。
// 每轮以 2000 Hz 为目标节拍，循环次数由函数内常量决定。
void calibrateAttitude() {
  // 舵机输出前预热 IMU 和姿态滤波器；每次迭代按 2000 Hz 节拍运行。
  for (int i = 0; i <= 10000; i++) {
    updateFlightClock();
    getBMI088data();
    Madgwick();
    loopRate(2000);
  }
}

// 传感器测量值由本模块更新，控制器与日志模块读取。
SPISettings bmiSettings(10000000, MSBFIRST, SPI_MODE3);
FC_Binary_Packet airdata;
CalibrationAccGyroData calAccGyroData;
int32_t Strain_value1 = 0;
int32_t Strain_value2 = 0;
int32_t Strain_value3 = 0;
int32_t Strain_value4 = 0;
int32_t Strain_value5 = 0;
float AccX_6050, AccY_6050, AccZ_6050;
float GyroX_6050, GyroY_6050, GyroZ_6050;
float Gyro_X_EXT, Gyro_Y_EXT, Gyro_Z_EXT;
float Acc_X_EXT, Acc_Y_EXT, Acc_Z_EXT;
float roll_IMU_EXT, pitch_IMU_EXT, yaw_IMU_EXT;
float airspeed_A;

namespace {
unsigned long last_read_Strain_Time = 0;
unsigned int read_Strain_freq = 10; // 读取应变数据频率
constexpr uint8_t SPI_READ = 0x80;
constexpr uint8_t ICM_ADDR = 0x69;
constexpr uint8_t REG_ICM_BANK_SEL = 0x76;
constexpr uint8_t REG_ICM_PWR_MGMT0 = 0x4E;
constexpr uint8_t REG_ICM_ACC_DATA = 0x1F;
constexpr uint8_t REG_ICM_GYRO_DATA = 0x25;
constexpr uint8_t BMI_ACC_ADDR = 0x19;
constexpr uint8_t BMI_GYR_ADDR = 0x69;
constexpr uint8_t REG_BMI_ACC_DATA = 0x12;
constexpr uint8_t REG_BMI_GYR_DATA = 0x02;
constexpr uint8_t REG_BMI_ACC_PWR_CTRL = 0x7D;
constexpr uint8_t REG_BMI_ACC_PWR_CONF = 0x7C;
const float inv_acc_lsb = 1.0f / 5460.0f;
const float inv_gyr_lsb = 1.0f / 16.384f;
}

// 解析 ExternalImuSerial 的 11 字节外置 IMU 帧；校验通过后更新姿态角、角速度或加速度。
// 输出角度为度、角速度为 °/s、加速度为 m/s²。
void getIMUdata_EXT() {
#if defined EXTIMU
  // 串口帧长 11 字节：0x55 帧头、数据类型、8 字节数据及累加校验字节。
  static uint8_t buf[11], pos = 0;

  while (ExternalImuSerial.available()) {
    uint8_t b = ExternalImuSerial.read();

    // 检查数据头
    if (pos == 0 && b != 0x55)
      continue;
    if (pos == 1 && b != 0x52 && b != 0x59 && b != 0x51 && b != 0x53) {
      pos = 0;
      continue;
    }

    buf[pos++] = b;

    // 完整数据包处理
    if (pos == 11) {
      pos = 0;

      // 计算校验和
      uint8_t sum = buf[0] + buf[1];
      for (int i = 2; i < 10; i++)
        sum += buf[i];
      if (sum == buf[10]) {

        if (buf[1] == 0x59) { // 四元数数据
          float q0 = ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f;
          float q1 = ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f;
          float q2 = ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f;
          float q3 = ((int16_t)(buf[9] << 8) | buf[8]) / 32768.0f;
          // USBSerial.print("Q:"); USBSerial.print(q0,3); USBSerial.print(",");
          // USBSerial.print(q1,3); USBSerial.print(","); USBSerial.print(q2,3);
          // USBSerial.print(","); USBSerial.println(q3,3);
          roll_IMU_EXT = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
                         57.29577951; // 弧度转角度。
          pitch_IMU_EXT = asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999,
                                         0.999999)) *
                          57.29577951; // 弧度转角度。
          yaw_IMU_EXT = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
                        57.29577951; // 弧度转角度。
          // USBSerial.print("R:");
          // USBSerial.println(roll_IMU_EXT);
        } else if (buf[1] == 0x52) { // 角速度数据
          Gyro_X_EXT =
              -1 * (((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 2000.0f);
          Gyro_Y_EXT = ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 2000.0f;
          Gyro_Z_EXT = ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 2000.0f;
          // USBSerial.print("X:");
          // USBSerial.println(Gyro_X_EXT);
          // USBSerial.print(wy,1); USBSerial.print(","); USBSerial.println(wz,1);
        }

        else if (buf[1] == 0x51) { // 加速度数据
          Acc_X_EXT =
              ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 16.0f * 9.8f;
          Acc_Y_EXT =
              ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 16.0f * 9.8f;
          Acc_Z_EXT =
              ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 16.0f * 9.8f;
          // USBSerial.print("A:");
          // USBSerial.println(Acc_X_EXT);
          // USBSerial.print("A:"); USBSerial.print(ax,1); USBSerial.print(",");
          // USBSerial.print(ay,1); USBSerial.print(","); USBSerial.println(az,1);
        }
      }
    }
  }

#endif
}

// 应变传感器的五通道数据按设备协议逐通道转换。
// 从应变响应帧中按设备字节顺序取出指定通道的 32 位有符号测量值。
// index 范围为 0～4；调用方需先保证响应帧长度足够。
int32_t parseChannel(uint8_t *buf, int index) {
  // index 为通道编号，范围 0～4。

  int offset = 3 + index * 4;

  union {
    int32_t val;
    uint8_t b[4];
  } u;

  // 按手册：低位在前，高位在后
  u.b[0] = buf[offset + 1];
  u.b[1] = buf[offset + 0];
  u.b[2] = buf[offset + 3];
  u.b[3] = buf[offset + 2];

  return u.val;
}

// Modbus CRC16，发送时低字节在前。
// 计算 Modbus RTU 的 CRC16；返回值发送时先写低字节。
uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// 发送功能码 0x03 的寄存器读取请求。
// 构造应变传感器的 8 字节 Modbus 读取请求并写入 StrainSensorSerial。
// addr 为设备地址，fcode 为功能码，reg 为起始寄存器，num 为寄存器数量。
void sendStrainCommand(uint8_t addr, uint16_t fcode, uint16_t reg,
                       uint16_t num) {

  uint8_t frame[8];

  frame[0] = addr;
  frame[1] = fcode;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = num >> 8;
  frame[5] = num & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);

  frame[6] = crc & 0xFF; // CRC低位
  frame[7] = crc >> 8;   // CRC高位

  // USBSerial.write(frame, 8);
  StrainSensorSerial.write(frame, 8);
}

// 按设定频率请求 10 个寄存器，并在收到足够长的响应时更新五路应变值。
void Strain_read_all() {
  // 按 read_Strain_freq 限制请求频率；只在收到完整响应时更新五通道值。
  float invFreq = 1.0 / read_Strain_freq * 1000000.0;
  unsigned long checker = micros();

  if (checker - last_read_Strain_Time < invFreq)
    return;
  last_read_Strain_Time = checker;
  // 从地址 0x0000 起读取 10 个寄存器。
  sendStrainCommand(1, 0x03, 0x0000, 10);
  uint8_t buf[32];
  int len = 0;

  while (StrainSensorSerial.available()) {
    // USBSerial.println("available ");
    buf[len++] = StrainSensorSerial.read();
  }

  // 解析
  if (len >= 25 && buf[1] == 0x03) {

    Strain_value1 = parseChannel(buf, 0);
    Strain_value2 = parseChannel(buf, 1);
    Strain_value3 = parseChannel(buf, 2);
    Strain_value4 = parseChannel(buf, 3);
    Strain_value5 = parseChannel(buf, 4);
  }
}

// 经 I2C 连续读取 ICM42688 三轴加速度和角速度，并换算到共享 IMU 变量。
// 当前量程换算后的单位分别为 g 和 °/s；通信失败时保持旧值。
void getICM42688data() {
  // 切换到寄存器 Bank 0 后连续读取三轴加速度和三轴角速度。
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_BANK_SEL);
  Wire.write(0x00);
  Wire.endTransmission();

  // 连续读取 12 字节：前 6 字节为加速度，后 6 字节为陀螺仪。
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_ACC_DATA);
  if (Wire.endTransmission(false) != 0)
    return; // 通讯失败直接退出

  if (Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)12) == 12) {
    // ICM-42688 数据按高字节在前拼接为有符号 16 位整数。
    int16_t raw_ax = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_ay = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_az = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gx = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gy = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gz = (int16_t)(Wire.read() << 8 | Wire.read());

    // 保持控制器沿用的 6050 变量名；按当前 ±16 g、±2000 deg/s 量程换算。
    AccX_6050 = raw_ax / 2048.0;
    AccY_6050 = raw_ay / 2048.0;
    AccZ_6050 = raw_az / 2048.0;

    GyroX_6050 = raw_gx / 16.4;
    GyroY_6050 = raw_gy / 16.4;
    GyroZ_6050 = raw_gz / 16.4;

    // 4. 应用低通滤波 (保留你原有的 B_accel_6050 逻辑)
    /*
    AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 *
    AccX_6050; AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050
    * AccY_6050; AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 +
    B_accel_6050 * AccZ_6050; AccX_prev_6050 = AccX_6050; AccY_prev_6050 =
    AccY_6050; AccZ_prev_6050 = AccZ_6050;

    GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 *
    GyroX_6050; GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050
    * GyroY_6050; GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 +
    B_gyro_6050 * GyroZ_6050; GyroX_prev_6050 = GyroX_6050; GyroY_prev_6050 =
    GyroY_6050; GyroZ_prev_6050 = GyroZ_6050;
    */
  }
}

// 经 SPI 读取 BMI088，加速度与角速度按安装方向旋转 90° 并扣除 EEPROM 零偏。
// 结果写入沿用的 6050 变量，单位分别为 g 和 °/s。
void getBMI088data() {
#if defined INTIMU
  // 加速度计 SPI 读取需要一个哑字节；陀螺仪读取不需要。

  uint8_t buf[6];
  const float inv_acc = 1.0f / 5460.0f; // ±6g
  const float inv_gyr = 1.0f / 16.384f; // ±2000dps
  static unsigned long lastBMI088AccDebug = 0;
  static unsigned long lastBMI088GyrDebug = 0;

  // 读取加速度计原始值。
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(0x12 | SPI_READ);
  SPI.transfer(0x00); // 丢弃协议要求的哑字节。
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_ACC, HIGH);
  SPI.endTransaction();

  int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

  // 读取陀螺仪原始值。
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_GYR, LOW);
  SPI.transfer(0x02 | SPI_READ);
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_GYR, HIGH);
  SPI.endTransaction();

  int16_t rgx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t rgy = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rgz = (int16_t)(buf[4] | (buf[5] << 8));

  // 按安装方向绕 Z 轴旋转 90°：新 X=旧 Y，新 Y=-旧 X，Z 不变。
  AccX_6050 = (ry * inv_acc) - calAccGyroData.AccErrorX_6050;
  AccY_6050 = (-rx * inv_acc) - calAccGyroData.AccErrorY_6050;
  AccZ_6050 = (rz * inv_acc) - calAccGyroData.AccErrorZ_6050;

  GyroX_6050 = (rgy * inv_gyr) - calAccGyroData.GyroErrorX_6050;
  GyroY_6050 = (-rgx * inv_gyr) - calAccGyroData.GyroErrorY_6050;
  GyroZ_6050 = (rgz * inv_gyr) - calAccGyroData.GyroErrorZ_6050;

  // --- C. 应用低通滤波 (保留你原有的 B_accel_6050 参数逻辑) ---
  /*
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050; AccY_prev_6050 = AccY_6050; AccZ_prev_6050 =
  AccZ_6050;

  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050; GyroY_prev_6050 = GyroY_6050; GyroZ_prev_6050 =
  GyroZ_6050;
  */

#endif
}

// 以十六进制打印原始字节和三轴有符号值；用于核对 BMI088 SPI 读数。
void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z) {
  USBSerial.print(tag);
  USBSerial.print(F(" raw:"));
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10)
      USBSerial.print('0');
    USBSerial.print(buf[i], HEX);
    if (i < len - 1)
      USBSerial.print(' ');
  }
  USBSerial.print(F(" val:"));
  USBSerial.print(x);
  USBSerial.print(F(","));
  USBSerial.print(y);
  USBSerial.print(F(","));
  USBSerial.println(z);
}

// 从 TelemetrySerial 查找 0xAA 0x55 帧头、校验二进制数据包并更新 airdata。
// 不合法的真空速值置零；不完整或校验失败的帧不更新共享状态。
void getairdata() {
  while (TelemetrySerial.available() >= sizeof(FC_Binary_Packet)) {

    // 检查第一个帧头
    if (TelemetrySerial.read() == 0xAA) {
      // 检查第二个帧头
      if (TelemetrySerial.peek() == 0x55) {
        FC_Binary_Packet candidate = {};
        uint8_t *ptr = reinterpret_cast<uint8_t *>(&candidate);

        TelemetrySerial.read(); // 跳过第二个帧头
        ptr[0] = 0xAA;
        ptr[1] = 0x55;

        const size_t payloadLen = sizeof(FC_Binary_Packet) - 2;
        if (TelemetrySerial.readBytes(&ptr[2], payloadLen) != payloadLen) {
          continue;
        }

        // 验证校验和
        uint8_t calcSum = 0;
        for (int i = 2; i < (sizeof(FC_Binary_Packet) - 1); i++) {
          calcSum += ptr[i];
        }

        if (calcSum == candidate.checksum) {
          if (!(candidate.tas >= 0.0f && candidate.tas < 200.0f)) {
            candidate.tas = 0.0f;
          }
          airdata = candidate;
          USBSerial.println(airdata.aoa);
        }
      }
    }
  }
}

// 辅助函数：SPI 写寄存器
// 向指定片选引脚对应的 SPI 设备写入一个寄存器值。
void writeRegSPI(int cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(bmiSettings);
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// 通用 I2C 寄存器写入辅助函数
// 通过指定 I2C 总线向设备地址和寄存器写入一个字节。
void writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
}



float relativeAngle_raw, relativeAngle_ready, relativeAngle_offset;
const int eepromAddress1 = 100;


String serialBuffer = "";


// 空速
MS4525read_Task airspeedSensor(&Wire1);


float B_accel_6050 =
    0.6; // MPU6050 加速度一阶低通系数；越小，平滑越强、延迟越大。
float B_gyro_6050 = 0.47;  // MPU6050 陀螺仪低通系数。
float B_accel_9250 = 0.74; // MPU9250 加速度低通系数。
float B_gyro_9250 = 0.67;  // MPU9250 陀螺仪低通系数。
float B_mag_9250 = 1.0;    // MPU9250 磁力计低通系数；1 表示直接采用本次读数。

// MPU9250 磁力计的零偏与各轴比例校准值。
float MagErrorX_9250 = 17.33;
float MagErrorY_9250 = 58.29;
float MagErrorZ_9250 = 29.16;
float MagScaleX_9250 = 0.98;
float MagScaleY_9250 = 1.05;
float MagScaleZ_9250 = 0.97;


// 内置 IMU 的静态校准常量；在线校准结果另存于 calAccGyroData。
// MPU6050

float AccErrorX_6050 = -0.02;
float AccErrorY_6050 = -0.01;
float AccErrorZ_6050 = 0.03;
float GyroErrorX_6050 = -13.83;
float GyroErrorY_6050 = -9.41;
float GyroErrorZ_6050 = 1.48;

// MPU9250
float AccErrorX_9250 = -0.11;
float AccErrorY_9250 = 0.01;
float AccErrorZ_9250 = 0.03;
float GyroErrorX_9250 = 5.41;
float GyroErrorY_9250 = -3.37;
float GyroErrorZ_9250 = -2.51;




// 内置 IMU 当前值与一阶低通的上一采样值。
float AccX_9250, AccY_9250, AccZ_9250;
float AccX_prev_6050, AccY_prev_6050, AccZ_prev_6050;
float AccX_prev_9250, AccY_prev_9250, AccZ_prev_9250;
float GyroX_9250, GyroY_9250, GyroZ_9250;
float GyroX_prev_6050, GyroY_prev_6050, GyroZ_prev_6050;
float GyroX_prev_9250, GyroY_prev_9250, GyroZ_prev_9250;

float MagX_9250, MagY_9250, MagZ_9250;
float MagX_prev_9250, MagY_prev_9250, MagZ_prev_9250;

// 从 EEPROM 地址 100 恢复转角传感器清零偏置。
void loadRotateSensorOffset() { EEPROM.get(eepromAddress1, relativeAngle_offset); }
// 调用空速传感器驱动进行零点标定。
void calibrateAirspeedSensor() { airspeedSensor.calib(); }

// 按编译配置初始化内置 IMU 的总线、电源状态、量程及采样参数。
void IMUinit() {
#if defined INTIMU
// 根据编译配置初始化内置 IMU；BMI088 通过 SPI，MPU9250 通过自身驱动初始化。
#if defined USE_MPU6050_I2C
  Wire.begin();
  // Wire.setClock(1000000);  // 历史配置：1 MHz 超过器件手册常用的 400 kHz。
  // max...

  // mpu6050.initialize();
  //  --- 插入 ICM-42688 初始化 --- [cite: 1833]
  // Wire.setClock(1000000); // 42688p建议 400kHz 比较稳
  // Wire.beginTransmission(ICM_ADDR);
  // Wire.write(REG_ICM_PWR_MGMT0);
  // Wire.write(0x0F); // 开启加速度计和陀螺仪的低噪声模式
  // Wire.endTransmission();
  delay(10);

  // 1. 初始化引脚
  pinMode(CS_ACC, OUTPUT);
  pinMode(CS_GYR, OUTPUT);
  digitalWrite(CS_ACC, HIGH);
  digitalWrite(CS_GYR, HIGH);

  // 2. 启动 SPI
  SPI.begin();
  delay(100);

  // 3. 唤醒加速度计 (必需序列)
  writeRegSPI(CS_ACC, 0x7D, 0x04); // ACC_PWR_CTRL: Active
  delay(10);
  writeRegSPI(CS_ACC, 0x7C, 0x00); // ACC_PWR_CONF: Active
  delay(10);

  // 4. 配置陀螺仪
  writeRegSPI(CS_GYR, 0x0F, 0x00); // ±2000 dps
  writeRegSPI(CS_GYR, 0x10, 0x02); // ODR 1000Hz, BW 116Hz

  USBSerial.println("BMI088 SPI Initialized.");

  /*
    if (mpu6050.testConnection() == false) {
      USBSerial.println("MPU6050 initialization unsuccessful");
      USBSerial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }
    */

  // 配置量程；BMI088 的电源和陀螺仪寄存器已在上方写入。
  mpu6050.setFullScaleGyroRange(GYRO_SCALE_6050);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE_6050);
#endif

#if defined USE_MPU9250_SPI
  int status = mpu9250.begin();

  if (status < 0) {
    USBSerial.println("MPU9250 initialization unsuccessful");
    USBSerial.println("Check MPU9250 wiring or try cycling power");
    USBSerial.print("Status: ");
    USBSerial.println(status);
    while (1) {
    }
  }

  // 配置 MPU9250 的量程、磁力计校准值与采样分频。
  mpu9250.setGyroRange(GYRO_SCALE_9250);
  mpu9250.setAccelRange(ACCEL_SCALE_9250);
  mpu9250.setMagCalX(MagErrorX_9250, MagScaleX_9250);
  mpu9250.setMagCalY(MagErrorY_9250, MagScaleY_9250);
  mpu9250.setMagCalZ(MagErrorZ_9250, MagScaleZ_9250);
  mpu9250.setSrd(
      0); // 陀螺仪和加速度计约 1 kHz，磁力计约 100 Hz。
#endif

#endif
}


// 读取 MPU6050/MPU9250 原始惯性数据，做单位换算、零偏校正和一阶低通。
// 这是旧内置 IMU 路径，是否调用由当前硬件配置决定。
void getIMUdata() {
  // 读取内置 IMU 原始值，换算单位、扣除零偏，再做一阶低通。
  // 加速度单位为 g，角速度为 °/s，磁场强度为 μT。
  int16_t AcX_6050, AcY_6050, AcZ_6050, GyX_6050, GyY_6050, GyZ_6050;
  int16_t AcX_9250, AcY_9250, AcZ_9250, GyX_9250, GyY_9250, GyZ_9250, MgX_9250,
      MgY_9250, MgZ_9250;

#if defined USE_MPU6050_I2C
  mpu6050.getMotion6(&AcX_6050, &AcY_6050, &AcZ_6050, &GyX_6050, &GyY_6050,
                     &GyZ_6050);
#endif
#if defined USE_MPU9250_SPI
  mpu9250.getMotion9(&AcX_9250, &AcY_9250, &AcZ_9250, &GyX_9250, &GyY_9250,
                     &GyZ_9250, &MgX_9250, &MgY_9250, &MgZ_9250);
#endif

  // 加速度：按量程系数换算为 g，随后扣除标定零偏。
  AccX_6050 = AcX_6050 / ACCEL_SCALE_FACTOR;
  AccY_6050 = AcY_6050 / ACCEL_SCALE_FACTOR;
  AccZ_6050 = AcZ_6050 / ACCEL_SCALE_FACTOR;
  // MPU6050 加速度零偏校正。
  AccX_6050 = AccX_6050 - calAccGyroData.AccErrorX_6050;
  AccY_6050 = AccY_6050 - calAccGyroData.AccErrorY_6050;
  AccZ_6050 = AccZ_6050 - calAccGyroData.AccErrorZ_6050;
  // MPU6050 加速度一阶低通。
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050;
  AccY_prev_6050 = AccY_6050;
  AccZ_prev_6050 = AccZ_6050;

  AccX_9250 = AcX_9250 / ACCEL_SCALE_FACTOR;
  AccY_9250 = AcY_9250 / ACCEL_SCALE_FACTOR;
  AccZ_9250 = AcZ_9250 / ACCEL_SCALE_FACTOR;
  // MPU9250 加速度零偏校正。
  AccX_9250 = AccX_9250 - calAccGyroData.AccErrorX_9250;
  AccY_9250 = AccY_9250 - calAccGyroData.AccErrorY_9250;
  AccZ_9250 = AccZ_9250 - calAccGyroData.AccErrorZ_9250;
  // MPU9250 加速度一阶低通。
  AccX_9250 = (1.0 - B_accel_9250) * AccX_prev_9250 + B_accel_9250 * AccX_9250;
  AccY_9250 = (1.0 - B_accel_9250) * AccY_prev_9250 + B_accel_9250 * AccY_9250;
  AccZ_9250 = (1.0 - B_accel_9250) * AccZ_prev_9250 + B_accel_9250 * AccZ_9250;
  AccX_prev_9250 = AccX_9250;
  AccY_prev_9250 = AccY_9250;
  AccZ_prev_9250 = AccZ_9250;

  // 角速度：按量程系数换算为 °/s，随后扣除标定零偏。
  GyroX_6050 = GyX_6050 / GYRO_SCALE_FACTOR;
  GyroY_6050 = GyY_6050 / GYRO_SCALE_FACTOR;
  GyroZ_6050 = GyZ_6050 / GYRO_SCALE_FACTOR;
  // MPU6050 陀螺仪零偏校正。
  GyroX_6050 = GyroX_6050 - calAccGyroData.GyroErrorX_6050;
  GyroY_6050 = GyroY_6050 - calAccGyroData.GyroErrorY_6050;
  GyroZ_6050 = GyroZ_6050 - calAccGyroData.GyroErrorZ_6050;
  // MPU6050 角速度一阶低通。
  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050;
  GyroY_prev_6050 = GyroY_6050;
  GyroZ_prev_6050 = GyroZ_6050;

  GyroX_9250 = GyX_9250 / GYRO_SCALE_FACTOR;
  GyroY_9250 = GyY_9250 / GYRO_SCALE_FACTOR;
  GyroZ_9250 = GyZ_9250 / GYRO_SCALE_FACTOR;
  // MPU9250 陀螺仪零偏校正。
  GyroX_9250 = GyroX_9250 - calAccGyroData.GyroErrorX_9250;
  GyroY_9250 = GyroY_9250 - calAccGyroData.GyroErrorY_9250;
  GyroZ_9250 = GyroZ_9250 - calAccGyroData.GyroErrorZ_9250;
  // MPU9250 角速度一阶低通。
  GyroX_9250 = (1.0 - B_gyro_9250) * GyroX_prev_9250 + B_gyro_9250 * GyroX_9250;
  GyroY_9250 = (1.0 - B_gyro_9250) * GyroY_prev_9250 + B_gyro_9250 * GyroY_9250;
  GyroZ_9250 = (1.0 - B_gyro_9250) * GyroZ_prev_9250 + B_gyro_9250 * GyroZ_9250;
  GyroX_prev_9250 = GyroX_9250;
  GyroY_prev_9250 = GyroY_9250;
  GyroZ_prev_9250 = GyroZ_9250;

  // 磁力计：先换算为 μT，再校正零偏和比例。
  MagX_9250 = MgX_9250 / 6.0;
  MagY_9250 = MgY_9250 / 6.0;
  MagZ_9250 = MgZ_9250 / 6.0;
  // MPU9250 磁力计标定。
  MagX_9250 = (MagX_9250 - MagErrorX_9250) * MagScaleX_9250;
  MagY_9250 = (MagY_9250 - MagErrorY_9250) * MagScaleY_9250;
  MagZ_9250 = (MagZ_9250 - MagErrorZ_9250) * MagScaleZ_9250;
  // MPU9250 磁场一阶低通。
  MagX_9250 = (1.0 - B_mag_9250) * MagX_prev_9250 + B_mag_9250 * MagX_9250;
  MagY_9250 = (1.0 - B_mag_9250) * MagY_prev_9250 + B_mag_9250 * MagY_9250;
  MagZ_9250 = (1.0 - B_mag_9250) * MagZ_prev_9250 + B_mag_9250 * MagZ_9250;
  MagX_prev_9250 = MagX_9250;
  MagY_prev_9250 = MagY_9250;
  MagZ_prev_9250 = MagZ_9250;
}


// 静置采集 BMI088 多组数据，计算旋转后机体系的加速度和角速度零偏。
// 完成后写入 EEPROM；标定期间应保持机体水平且静止。
void calculate_IMU_error() {
  // 采集 12000 组 BMI088 数据，在旋转后的机体系计算静态零偏并写入 EEPROM。
  // EEPROM
  double sAX = 0, sAY = 0, sAZ = 0, sGX = 0, sGY = 0, sGZ = 0;
  int c = 0;
  uint8_t buf[6];

  USBSerial.println(
      "BMI088 SPI Calibrating (Rotated 90)... Keep vehicle flat and still.");

  while (c < 12000) {
    // 1. SPI 读取原始加速度
    SPI.beginTransaction(bmiSettings);
    digitalWrite(CS_ACC, LOW);
    SPI.transfer(0x12 | 0x80); // 读取加速度起始地址
    SPI.transfer(0x00); // 重要：BMI088 加速度计 SPI 读取必需的 Dummy Byte
    for (int i = 0; i < 6; i++)
      buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_ACC, HIGH);
    SPI.endTransaction();

    int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

    // 2. SPI 读取原始陀螺仪
    SPI.beginTransaction(bmiSettings);
    digitalWrite(CS_GYR, LOW);
    SPI.transfer(0x02 | 0x80); // 读取陀螺仪起始地址
    for (int i = 0; i < 6; i++)
      buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_GYR, HIGH);
    SPI.endTransaction();

    int16_t rgx = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t rgy = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t rgz = (int16_t)(buf[4] | (buf[5] << 8));

    // 3. 映射与累加 (保持 90 度旋转逻辑: 新X=旧Y, 新Y=-旧X)
    sAX += ry / 5460.0;
    sAY += -rx / 5460.0;
    sAZ += rz / 5460.0;

    sGX += rgy / 16.384;
    sGY += -rgx / 16.384;
    sGZ += rgz / 16.384;

    if (c % 1000 == 0)
      USBSerial.print(".");
    c++;
    delayMicroseconds(100);
  }

  // 4. 计算均值并补偿 Z 轴重力 (假设平放时 Z 应为 1g)
  calAccGyroData.AccErrorX_6050 = (float)(sAX / c);
  calAccGyroData.AccErrorY_6050 = (float)(sAY / c);
  calAccGyroData.AccErrorZ_6050 = (float)((sAZ / c) - 1.0f);

  calAccGyroData.GyroErrorX_6050 = (float)(sGX / c);
  calAccGyroData.GyroErrorY_6050 = (float)(sGY / c);
  calAccGyroData.GyroErrorZ_6050 = (float)(sGZ / c);

  // 5. 存入 EEPROM
  int addr = 0;
  byte *p = (byte *)&calAccGyroData;
  for (unsigned int i = 0; i < sizeof(CalibrationAccGyroData); i++) {
    EEPROM.write(addr++, p[i]);
  }

  USBSerial.println(
      "\nBMI088 SPI Rotation Calibration Complete & Saved to EEPROM.");
}


// 运行 MPU9250 磁力计交互式标定并打印零偏和比例结果。
// 函数结尾停机，不会返回飞行主循环。
void calibrateMagnetometer() {
#if defined USE_MPU9250_SPI
  float success;
  USBSerial.println("Beginning magnetometer calibration in");
  USBSerial.println("3...");
  delay(1000);
  USBSerial.println("2...");
  delay(1000);
  USBSerial.println("1...");
  delay(1000);
  USBSerial.println("Rotate the IMU about all axes until complete.");
  USBSerial.println(" ");
  success = mpu9250.calibrateMag();
  if (success) {
    USBSerial.println("Calibration Successful!");
    USBSerial.println("Please comment out the calibrateMagnetometer() function "
                   "and copy these values into the code:");
    USBSerial.print("float MagErrorX_9250 = ");
    USBSerial.print(mpu9250.getMagBiasX_uT());
    USBSerial.println(";");
    USBSerial.print("float MagErrorY_9250 = ");
    USBSerial.print(mpu9250.getMagBiasY_uT());
    USBSerial.println(";");
    USBSerial.print("float MagErrorZ_9250 = ");
    USBSerial.print(mpu9250.getMagBiasZ_uT());
    USBSerial.println(";");
    USBSerial.print("float MagScaleX_9250 = ");
    USBSerial.print(mpu9250.getMagScaleFactorX());
    USBSerial.println(";");
    USBSerial.print("float MagScaleY_9250 = ");
    USBSerial.print(mpu9250.getMagScaleFactorY());
    USBSerial.println(";");
    USBSerial.print("float MagScaleZ_9250 = ");
    USBSerial.print(mpu9250.getMagScaleFactorZ());
    USBSerial.println(";");
    USBSerial.println(" ");

    USBSerial.println("If you are having trouble with your attitude estimate at a "
                   "new flying location, repeat this process as needed.");
  } else {
    USBSerial.println(
        "Calibration Unsuccessful. Please reset the board and try again.");
  }

  while (1)
    ; // 标定结束后停在这里，避免直接进入飞行主循环。
#endif
  USBSerial.println("Error: MPU9250 not selected. Cannot calibrate non-existent "
                 "magnetometer.");
  while (1)
    ; // 未启用 MPU9250 时阻止继续运行。
}


// 每隔至少 20 ms 从空速驱动刷新一次 airspeed_A。
void getairspeed() {
  // 以 50 Hz 更新空速；其他控制周期沿用上次有效测量。
  static unsigned long lastAirspeedRead = 0;
  if (micros() - lastAirspeedRead > 20000) {
    lastAirspeedRead = micros();
    airspeed_A = airspeedSensor.getAirspeed();
    // USBSerial.println(airspeed_A);
  }
}



// 启用气压计时读取温度、气压和高度，并打印测量值。
void getbarodata() {
  // 读取气压计温度、气压和按标准海平面气压换算的高度。
#if defined USE_BAROMETER
  // 快速读取原始数据
  temp = bmp.readTemperature();  // °C
  pressure = bmp.readPressure(); // Pa
  // 计算海拔高度(国际标准大气模型)
  filteredAltitude = bmp.readAltitude(1013.25);
  // 输出调试信息
  USBSerial.printf("Temp: %.1fC | Pressure: %.2fPa | Alt: %.2fm\n", temp, pressure,
                filteredAltitude);
#endif
}


// 启用气压计时初始化 BMP280，配置 I2C 时钟、采样倍率和内部滤波。
void initBAROMETER() {
#if defined USE_BAROMETER

  BMP280_I2C.begin();
  BMP280_I2C.setClock(400000); // 提升至400kHz
  if (!bmp.begin(0x76)) {      // 尝试默认地址0x76
    USBSerial.println("BMP280未找到，尝试0x77...");
    if (!bmp.begin(0x77)) {
      USBSerial.println("BMP280初始化失败!");
      // while(1);
    }
  }

  // 配置高性能模式
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // 连续采样
                  Adafruit_BMP280::SAMPLING_X2,   // 温度采样
                  Adafruit_BMP280::SAMPLING_X16,  // 气压采样
                  Adafruit_BMP280::FILTER_X16,    // IIR滤波
                  Adafruit_BMP280::STANDBY_MS_500 // 待机时间
  );
#endif
}


// 从 AngleSensorSerial 解析以换行结束的 Angle 文本，并按机位约定更新相对转角。
void getRotateSensor1() // 每个飞机都要做的
{
  // AngleSensorSerial 接收以换行结束的 "Angle:" 文本帧；不同机位采用不同角度正方向。

  while (AngleSensorSerial.available() > 0) {
    // USBSerial.println("sbb");
    char incomingChar = AngleSensorSerial.read();

    if (incomingChar == '\n') { // 检测到行结束符
      if (serialBuffer.startsWith("Angle:")) {
        String angleStr = serialBuffer.substring(6); // 提取 "X.YYY"
        relativeAngle_raw = angleStr.toFloat();

        // 打印到硬件串口（调试）
        // USBSerial.print("Received Angle: ");
        // USBSerial.print(relativeAngle_raw, 3);
        // USBSerial.println("°");
      }
      serialBuffer = "";               // 清空缓冲区
    } else if (incomingChar != '\r') { // 忽略 \r
      serialBuffer += incomingChar;
    }
  }

#if defined CPLANE || defined EPLANE || defined GPLANE
  relativeAngle_ready = (relativeAngle_raw - relativeAngle_offset);
#elif defined APLANE || defined BPLANE || defined DPLANE
  relativeAngle_ready = -(relativeAngle_raw - relativeAngle_offset);
#endif
}


// 将当前原始转角设为新零点，并将偏置写入 EEPROM 地址 100。
void ResetRotateSensor() {
  relativeAngle_offset = relativeAngle_raw;         // 计算新偏移量
  EEPROM.put(eepromAddress1, relativeAngle_offset); // 存储到 EEPROM
  USBSerial.println("Zero Set! Offset: " + String(relativeAngle_offset));
}






// 从 EEPROM 地址 0 恢复惯性传感器零偏结构体。
void loadImuCalibration() {
  // 从 EEPROM 地址 0 开始恢复 BMI088 零偏结构体。
  int address = 0;
  byte *pData = (byte *)&calAccGyroData;
  for (int i = 0; i < sizeof(CalibrationAccGyroData); i++) {
    pData[i] = EEPROM.read(address++);
  }
}

// 内置 IMU 启用时读取首帧 BMI088，并据此初始化姿态四元数。
void initializeInitialAttitude() {
  // 上电时用首帧加速度估计初始姿态四元数。
#if defined INTIMU
  getBMI088data();
  float phi, theta, psi;
#if defined USE_MPU6050_I2C
  phi = atan2(AccY_6050, AccZ_6050); // 滚转角，绕 x 轴。
  theta = atan2(-AccX_6050, sqrt(AccY_6050 * AccY_6050 +
                                 AccZ_6050 * AccZ_6050)); // 俯仰角，绕 y 轴。
  psi = 0;                                                // 无磁力计时偏航角初始化为 0。
#endif

#if defined USE_MPU9250_SPI
  phi = atan2(AccY_9250, AccZ_9250); // 滚转角，绕 x 轴。
  theta = atan2(-AccX_9250, sqrt(AccY_9250 * AccY_9250 +
                                 AccZ_9250 * AccZ_9250)); // 俯仰角，绕 y 轴。
  psi = atan2(-MagX_9250, MagY_9250);                     // 偏航角由磁力计初始化。
#endif

  const Eigen::Quaternionf initialAttitude = eulerToQuaternion(phi, theta, psi);
  q0 = initialAttitude.w();
  q1 = initialAttitude.x();
  q2 = initialAttitude.y();
  q3 = initialAttitude.z();
#endif
}

namespace {
// 角加速度低通滤波器（三轴）
static PX4LowPassFilter2p gyroDerivFiltX, gyroDerivFiltY,
    gyroDerivFiltZ; // getAngularACC() 求 dp/dq/dr 前的角速度滤波

static PX4LowPassFilter2p angularAccFiltX, angularAccFiltY, angularAccFiltZ;

// 前次滤波后陀螺仪值（用于差分）
float gyroX_filt_prev = 0.0f;
float gyroY_filt_prev = 0.0f;
float gyroZ_filt_prev = 0.0f;
// 前次差分时间（微秒）
unsigned long prev_time_gyro_deriv = 0;


} // namespace

float dp, dq, dr;

// 为角速度差分前后两级滤波器设置 500 Hz 采样参数。
void initializeAngularAccelerationFilters() {
  angularAccFiltX.set_cutoff_frequency(500, 100);
  angularAccFiltY.set_cutoff_frequency(500, 20);
  angularAccFiltZ.set_cutoff_frequency(500, 100);
  gyroDerivFiltX.set_cutoff_frequency(500, 100);
  gyroDerivFiltY.set_cutoff_frequency(500, 40);
  gyroDerivFiltZ.set_cutoff_frequency(500, 100);
}

// 由内置陀螺仪角速度计算 dp、dq、dr，单位为 °/s²；异常采样间隔时清零输出。
void getAngularACC() {
  // 先平滑角速度再差分，最后滤波并限幅，避免差分放大传感器噪声。
  unsigned long now = micros();

  float dt_deriv = (now - prev_time_gyro_deriv) * 1.0e-6f;
  prev_time_gyro_deriv = now;

  if (dt_deriv <= 0.0f || dt_deriv > 0.01f) {
    // 首次调用或采样中断后重建历史值，跳过本次角加速度计算。
    gyroX_filt_prev = gyroDerivFiltX.apply(-GyroX_6050);
    gyroY_filt_prev = gyroDerivFiltY.apply(GyroY_6050);
    gyroZ_filt_prev = gyroDerivFiltZ.apply(GyroZ_6050);
    dp = 0.0f;
    dq = 0.0f;
    dr = 0.0f;
    return;
  }

  float gyroX_now = gyroDerivFiltX.apply(-GyroX_6050);
  float gyroY_now = gyroDerivFiltY.apply(GyroY_6050);
  float gyroZ_now = gyroDerivFiltZ.apply(GyroZ_6050);

  float dp_raw = (gyroX_now - gyroX_filt_prev) / dt_deriv;
  float dq_raw = (gyroY_now - gyroY_filt_prev) / dt_deriv;
  float dr_raw = (gyroZ_now - gyroZ_filt_prev) / dt_deriv;

  dp = angularAccFiltX.apply(dp_raw);
  dq = angularAccFiltY.apply(dq_raw);
  dr = angularAccFiltZ.apply(dr_raw);

  gyroX_filt_prev = gyroX_now;
  gyroY_filt_prev = gyroY_now;
  gyroZ_filt_prev = gyroZ_now;

  const float deriv_limit = 3000.0f;
  dp = constrain(dp, -deriv_limit, deriv_limit);
  dq = constrain(dq, -deriv_limit, deriv_limit);
  dr = constrain(dr, -deriv_limit, deriv_limit);
}


