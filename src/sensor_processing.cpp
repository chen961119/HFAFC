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

// Setup gyro and accel full scale value selection and scale factor

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




void beginExternalImuLink() { Serial1.begin(115200); }
void beginStrainSensorLink() { Serial7.begin(115200); }

void calibrateAttitude() {
  // Warm up the IMU and Madgwick filter before actuator commands are enabled.
  for (int i = 0; i <= 10000; i++) {
    updateFlightClock();
    getBMI088data();
    Madgwick(dt);
    loopRate(2000, current_time);
  }
}

// Sensor state shared with the flight controller and logger.
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

void getIMUdata_EXT() {
  static uint8_t buf[11], pos = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

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
          // Serial.print("Q:"); Serial.print(q0,3); Serial.print(",");
          // Serial.print(q1,3); Serial.print(","); Serial.print(q2,3);
          // Serial.print(","); Serial.println(q3,3);
          roll_IMU_EXT = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
                         57.29577951; // degrees
          pitch_IMU_EXT = asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999,
                                         0.999999)) *
                          57.29577951; // degrees
          yaw_IMU_EXT = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
                        57.29577951; // degrees
          // Serial.print("R:");
          // Serial.println(roll_IMU_EXT);
        } else if (buf[1] == 0x52) { // 角速度数据
          Gyro_X_EXT =
              -1 * (((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 2000.0f);
          Gyro_Y_EXT = ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 2000.0f;
          Gyro_Z_EXT = ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 2000.0f;
          // Serial.print("X:");
          // Serial.println(Gyro_X_EXT);
          // Serial.print(wy,1); Serial.print(","); Serial.println(wz,1);
        }

        else if (buf[1] == 0x51) { // 加速度数据
          Acc_X_EXT =
              ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 16.0f * 9.8f;
          Acc_Y_EXT =
              ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 16.0f * 9.8f;
          Acc_Z_EXT =
              ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 16.0f * 9.8f;
          // Serial.print("A:");
          // Serial.println(Acc_X_EXT);
          // Serial.print("A:"); Serial.print(ax,1); Serial.print(",");
          // Serial.print(ay,1); Serial.print(","); Serial.println(az,1);
        }
      }
    }
  }
}

// 应变传感器
// 进制转换
int32_t parseChannel(uint8_t *buf, int index) {
  // index: 第几个通道（0~4）

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

// ===== CRC16 (Modbus) =====
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

// ===== 发送03读寄存器 =====
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

  // Serial.write(frame, 8);
  Serial7.write(frame, 8);
}

void Strain_read_all() {
  float invFreq = 1.0 / read_Strain_freq * 1000000.0;
  unsigned long checker = micros();

  if (checker - last_read_Strain_Time < invFreq)
    return;
  last_read_Strain_Time = checker;
  // 读取寄存器0x0000，1个寄存器
  sendStrainCommand(1, 0x03, 0x0000, 10);
  uint8_t buf[32];
  int len = 0;

  while (Serial7.available()) {
    // Serial.println("available ");
    buf[len++] = Serial7.read();
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

void getICM42688data() {
  // 1. 强制切换到 Bank 0
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_BANK_SEL);
  Wire.write(0x00);
  Wire.endTransmission();

  // 2. 读取 12 字节数据 (6字节加速 + 6字节陀螺仪) [cite: 1514]
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_ACC_DATA);
  if (Wire.endTransmission(false) != 0)
    return; // 通讯失败直接退出

  if (Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)12) == 12) {
    // ICM-42688 为 Big-Endian (高位在前) [cite: 1810]
    int16_t raw_ax = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_ay = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_az = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gx = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gy = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gz = (int16_t)(Wire.read() << 8 | Wire.read());

    // 3. 映射到你原有的 6050 变量中 (单位转换)
    // 默认量程: Accel ±16g (2048 LSB/g), Gyro ±2000dps (16.4 LSB/dps) [cite:
    // 168, 188]
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

void getBMI088data() {

  uint8_t buf[6];
  const float inv_acc = 1.0f / 5460.0f; // ±6g
  const float inv_gyr = 1.0f / 16.384f; // ±2000dps
  static unsigned long lastBMI088AccDebug = 0;
  static unsigned long lastBMI088GyrDebug = 0;

  // A. 读取加速度计 (注意 Dummy Byte)
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(0x12 | SPI_READ);
  SPI.transfer(0x00); // 重要：哑字节
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_ACC, HIGH);
  SPI.endTransaction();

  int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

  // B. 读取陀螺仪
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

  // C. 坐标轴映射 (保持你之前的 90 度旋转逻辑)
  // 原逻辑: 新X=旧Y, 新Y=-旧X, Z不变
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
}

void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z) {
  Serial.print(tag);
  Serial.print(F(" raw:"));
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10)
      Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i < len - 1)
      Serial.print(' ');
  }
  Serial.print(F(" val:"));
  Serial.print(x);
  Serial.print(F(","));
  Serial.print(y);
  Serial.print(F(","));
  Serial.println(z);
}

void getairdata() {
  while (Serial8.available() >= sizeof(FC_Binary_Packet)) {

    // 检查第一个帧头
    if (Serial8.read() == 0xAA) {
      // 检查第二个帧头
      if (Serial8.peek() == 0x55) {
        FC_Binary_Packet candidate = {};
        uint8_t *ptr = reinterpret_cast<uint8_t *>(&candidate);

        Serial8.read(); // 跳过第二个帧头
        ptr[0] = 0xAA;
        ptr[1] = 0x55;

        const size_t payloadLen = sizeof(FC_Binary_Packet) - 2;
        if (Serial8.readBytes(&ptr[2], payloadLen) != payloadLen) {
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
          Serial.println(airdata.aoa);
        }
      }
    }
  }
}

// 辅助函数：SPI 写寄存器
void writeRegSPI(int cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(bmiSettings);
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// 通用 I2C 寄存器写入辅助函数
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
    0.6; // Accelerometer LP filter paramter, (MPU6050 default: 0.14.
         // 0.26at1000hz   0.6at333hz)。 1-exp((2000/当前hz)）*log(1-0.17))
float B_gyro_6050 = 0.47;  // Gyro LP filter paramter, (MPU6050 default: 0.1.
                           // 0.19at1000hz  0.47at333hz)
float B_accel_9250 = 0.74; // Accelerometer LP filter paramter, ( MPU9250
                           // default: 0.2   0.36at1000hz。 0.74at333hz)
float B_gyro_9250 = 0.67;  // Gyro LP filter paramter, ( MPU9250 default: 0.17。
                           // 0.31at1000hz。0.67at333hz)
float B_mag_9250 = 1.0; // Magnetometer LP filter parameter 1.0 means no fliter

// Magnetometer calibration parameters - if using MPU9250, uncomment
// calibrateMagnetometer() in void setup() to get these values, else just ignore
// these
float MagErrorX_9250 = 17.33;
float MagErrorY_9250 = 58.29;
float MagErrorZ_9250 = 29.16;
float MagScaleX_9250 = 0.98;
float MagScaleY_9250 = 1.05;
float MagScaleZ_9250 = 0.97;


// IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the
// void setup() to get these values, then comment out calculate_IMU_error()
// mpu6050

float AccErrorX_6050 = -0.02;
float AccErrorY_6050 = -0.01;
float AccErrorZ_6050 = 0.03;
float GyroErrorX_6050 = -13.83;
float GyroErrorY_6050 = -9.41;
float GyroErrorZ_6050 = 1.48;

// mpu9250
float AccErrorX_9250 = -0.11;
float AccErrorY_9250 = 0.01;
float AccErrorZ_9250 = 0.03;
float GyroErrorX_9250 = 5.41;
float GyroErrorY_9250 = -3.37;
float GyroErrorZ_9250 = -2.51;




// IMU:
float AccX_9250, AccY_9250, AccZ_9250;
float AccX_prev_6050, AccY_prev_6050, AccZ_prev_6050;
float AccX_prev_9250, AccY_prev_9250, AccZ_prev_9250;
float GyroX_9250, GyroY_9250, GyroZ_9250;
float GyroX_prev_6050, GyroY_prev_6050, GyroZ_prev_6050;
float GyroX_prev_9250, GyroY_prev_9250, GyroZ_prev_9250;

float MagX_9250, MagY_9250, MagZ_9250;
float MagX_prev_9250, MagY_prev_9250, MagZ_prev_9250;

void loadRotateSensorOffset() { EEPROM.get(eepromAddress1, relativeAngle_offset); }
void calibrateAirspeedSensor() { airspeedSensor.calib(); }

void IMUinit() {
// DESCRIPTION: Initialize IMU 有一个用一个，有两个用两个
/*
 * Don't worry about how this works.
 */
#if defined USE_MPU6050_I2C
  Wire.begin();
  // Wire.setClock(1000000);  //Note this is 2.5 times the spec sheet 400 kHz
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

  Serial.println("BMI088 SPI Initialized.");

  /*
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }
    */

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE_6050);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE_6050);
#endif

#if defined USE_MPU9250_SPI
  int status = mpu9250.begin();

  if (status < 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check MPU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {
    }
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu9250.setGyroRange(GYRO_SCALE_9250);
  mpu9250.setAccelRange(ACCEL_SCALE_9250);
  mpu9250.setMagCalX(MagErrorX_9250, MagScaleX_9250);
  mpu9250.setMagCalY(MagErrorY_9250, MagScaleY_9250);
  mpu9250.setMagCalZ(MagErrorZ_9250, MagScaleZ_9250);
  mpu9250.setSrd(
      0); // sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
}


void getIMUdata() {
  // DESCRIPTION: Request full dataset from IMU and LP filter gyro,
  // accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY,
   * AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. These values are scaled
   * according to the IMU datasheet to put them into correct units of g's,
   * deg/sec, and uT. A simple first-order low-pass filter is used to get rid of
   * high frequency noise in these raw signals. Generally you want to cut off
   * everything past 80Hz, but if your loop rate is not fast enough, the low
   * pass filter will cause a lag in the readings. The filter parameters B_gyro
   * and B_accel are set to be good for a 2kHz loop rate. Finally, the constant
   * errors found in calculate_IMU_error() on startup are subtracted from the
   * accelerometer and gyro readings.
   */
  // 读数的单位是G
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

  // Accelerometer
  AccX_6050 = AcX_6050 / ACCEL_SCALE_FACTOR; // G's
  AccY_6050 = AcY_6050 / ACCEL_SCALE_FACTOR;
  AccZ_6050 = AcZ_6050 / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX_6050 = AccX_6050 - calAccGyroData.AccErrorX_6050;
  AccY_6050 = AccY_6050 - calAccGyroData.AccErrorY_6050;
  AccZ_6050 = AccZ_6050 - calAccGyroData.AccErrorZ_6050;
  // LP filter accelerometer data
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050;
  AccY_prev_6050 = AccY_6050;
  AccZ_prev_6050 = AccZ_6050;

  AccX_9250 = AcX_9250 / ACCEL_SCALE_FACTOR; // G's
  AccY_9250 = AcY_9250 / ACCEL_SCALE_FACTOR;
  AccZ_9250 = AcZ_9250 / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX_9250 = AccX_9250 - calAccGyroData.AccErrorX_9250;
  AccY_9250 = AccY_9250 - calAccGyroData.AccErrorY_9250;
  AccZ_9250 = AccZ_9250 - calAccGyroData.AccErrorZ_9250;
  // LP filter accelerometer data
  AccX_9250 = (1.0 - B_accel_9250) * AccX_prev_9250 + B_accel_9250 * AccX_9250;
  AccY_9250 = (1.0 - B_accel_9250) * AccY_prev_9250 + B_accel_9250 * AccY_9250;
  AccZ_9250 = (1.0 - B_accel_9250) * AccZ_prev_9250 + B_accel_9250 * AccZ_9250;
  AccX_prev_9250 = AccX_9250;
  AccY_prev_9250 = AccY_9250;
  AccZ_prev_9250 = AccZ_9250;

  // Gyro
  GyroX_6050 = GyX_6050 / GYRO_SCALE_FACTOR; // deg/sec
  GyroY_6050 = GyY_6050 / GYRO_SCALE_FACTOR;
  GyroZ_6050 = GyZ_6050 / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX_6050 = GyroX_6050 - calAccGyroData.GyroErrorX_6050;
  GyroY_6050 = GyroY_6050 - calAccGyroData.GyroErrorY_6050;
  GyroZ_6050 = GyroZ_6050 - calAccGyroData.GyroErrorZ_6050;
  // LP filter gyro data
  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050;
  GyroY_prev_6050 = GyroY_6050;
  GyroZ_prev_6050 = GyroZ_6050;

  GyroX_9250 = GyX_9250 / GYRO_SCALE_FACTOR; // deg/sec
  GyroY_9250 = GyY_9250 / GYRO_SCALE_FACTOR;
  GyroZ_9250 = GyZ_9250 / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX_9250 = GyroX_9250 - calAccGyroData.GyroErrorX_9250;
  GyroY_9250 = GyroY_9250 - calAccGyroData.GyroErrorY_9250;
  GyroZ_9250 = GyroZ_9250 - calAccGyroData.GyroErrorZ_9250;
  // LP filter gyro data
  GyroX_9250 = (1.0 - B_gyro_9250) * GyroX_prev_9250 + B_gyro_9250 * GyroX_9250;
  GyroY_9250 = (1.0 - B_gyro_9250) * GyroY_prev_9250 + B_gyro_9250 * GyroY_9250;
  GyroZ_9250 = (1.0 - B_gyro_9250) * GyroZ_prev_9250 + B_gyro_9250 * GyroZ_9250;
  GyroX_prev_9250 = GyroX_9250;
  GyroY_prev_9250 = GyroY_9250;
  GyroZ_prev_9250 = GyroZ_9250;

  // Magnetometer
  MagX_9250 = MgX_9250 / 6.0; // uT
  MagY_9250 = MgY_9250 / 6.0;
  MagZ_9250 = MgZ_9250 / 6.0;
  // Correct the outputs with the calculated error values
  MagX_9250 = (MagX_9250 - MagErrorX_9250) * MagScaleX_9250;
  MagY_9250 = (MagY_9250 - MagErrorY_9250) * MagScaleY_9250;
  MagZ_9250 = (MagZ_9250 - MagErrorZ_9250) * MagScaleZ_9250;
  // LP filter magnetometer data
  MagX_9250 = (1.0 - B_mag_9250) * MagX_prev_9250 + B_mag_9250 * MagX_9250;
  MagY_9250 = (1.0 - B_mag_9250) * MagY_prev_9250 + B_mag_9250 * MagY_9250;
  MagZ_9250 = (1.0 - B_mag_9250) * MagZ_prev_9250 + B_mag_9250 * MagZ_9250;
  MagX_prev_9250 = MagX_9250;
  MagY_prev_9250 = MagY_9250;
  MagZ_prev_9250 = MagZ_9250;
}


void calculate_IMU_error() {
  // DESCRIPTION: 采集 12000 样本计算 BMI088 的偏置，并以 90 度旋转后的轴系存入
  // EEPROM
  double sAX = 0, sAY = 0, sAZ = 0, sGX = 0, sGY = 0, sGZ = 0;
  int c = 0;
  uint8_t buf[6];

  Serial.println(
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
      Serial.print(".");
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

  Serial.println(
      "\nBMI088 SPI Rotation Calibration Complete & Saved to EEPROM.");
}


void calibrateMagnetometer() {
#if defined USE_MPU9250_SPI
  float success;
  Serial.println("Beginning magnetometer calibration in");
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Rotate the IMU about all axes until complete.");
  Serial.println(" ");
  success = mpu9250.calibrateMag();
  if (success) {
    Serial.println("Calibration Successful!");
    Serial.println("Please comment out the calibrateMagnetometer() function "
                   "and copy these values into the code:");
    Serial.print("float MagErrorX_9250 = ");
    Serial.print(mpu9250.getMagBiasX_uT());
    Serial.println(";");
    Serial.print("float MagErrorY_9250 = ");
    Serial.print(mpu9250.getMagBiasY_uT());
    Serial.println(";");
    Serial.print("float MagErrorZ_9250 = ");
    Serial.print(mpu9250.getMagBiasZ_uT());
    Serial.println(";");
    Serial.print("float MagScaleX_9250 = ");
    Serial.print(mpu9250.getMagScaleFactorX());
    Serial.println(";");
    Serial.print("float MagScaleY_9250 = ");
    Serial.print(mpu9250.getMagScaleFactorY());
    Serial.println(";");
    Serial.print("float MagScaleZ_9250 = ");
    Serial.print(mpu9250.getMagScaleFactorZ());
    Serial.println(";");
    Serial.println(" ");

    Serial.println("If you are having trouble with your attitude estimate at a "
                   "new flying location, repeat this process as needed.");
  } else {
    Serial.println(
        "Calibration Unsuccessful. Please reset the board and try again.");
  }

  while (1)
    ; // Halt code so it won't enter main loop until this function commented out
#endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent "
                 "magnetometer.");
  while (1)
    ; // Halt code so it won't enter main loop until this function commented out
}


void getairspeed() {
  static unsigned long lastAirspeedRead = 0;
  if (micros() - lastAirspeedRead > 20000) {
    lastAirspeedRead = micros();
    airspeed_A = airspeedSensor.getAirspeed();
    // Serial.println(airspeed_A);
  }
}



void getbarodata() {
#if defined USE_BAROMETER
  // 快速读取原始数据
  temp = bmp.readTemperature();  // °C
  pressure = bmp.readPressure(); // Pa
  // 计算海拔高度(国际标准大气模型)
  filteredAltitude = bmp.readAltitude(1013.25);
  // 输出调试信息
  Serial.printf("Temp: %.1fC | Pressure: %.2fPa | Alt: %.2fm\n", temp, pressure,
                filteredAltitude);
#endif
}


void initBAROMETER() {
#if defined USE_BAROMETER

  BMP280_I2C.begin();
  BMP280_I2C.setClock(400000); // 提升至400kHz
  if (!bmp.begin(0x76)) {      // 尝试默认地址0x76
    Serial.println("BMP280未找到，尝试0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280初始化失败!");
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


void getRotateSensor1() // 每个飞机都要做的
{

  while (Serial7.available() > 0) {
    // Serial.println("sbb");
    char incomingChar = Serial7.read();

    if (incomingChar == '\n') { // 检测到行结束符
      if (serialBuffer.startsWith("Angle:")) {
        String angleStr = serialBuffer.substring(6); // 提取 "X.YYY"
        relativeAngle_raw = angleStr.toFloat();

        // 打印到硬件串口（调试）
        // Serial.print("Received Angle: ");
        // Serial.print(relativeAngle_raw, 3);
        // Serial.println("°");
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


void ResetRotateSensor() {
  relativeAngle_offset = relativeAngle_raw;         // 计算新偏移量
  EEPROM.put(eepromAddress1, relativeAngle_offset); // 存储到 EEPROM
  Serial.println("Zero Set! Offset: " + String(relativeAngle_offset));
}






void loadImuCalibration() {
  int address = 0;
  byte *pData = (byte *)&calAccGyroData;
  for (int i = 0; i < sizeof(CalibrationAccGyroData); i++) {
    pData[i] = EEPROM.read(address++);
  }
}

void initializeInitialAttitude() {
#if defined INTIMU
  getBMI088data();
  eulerToQuaternion();
#endif
}
