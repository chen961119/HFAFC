#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "filters.h"

// 外置空速/迎角传感器的打包数据，字段顺序与串口协议保持一致。
#pragma pack(push, 1)
struct FC_Binary_Packet {
  uint8_t head1;
  uint8_t head2;
  float aoa;
  float aos;
  float tas;
  uint8_t statA;
  uint8_t statS;
  uint8_t checksum;
};
#pragma pack(pop)

typedef struct {
  // 各 IMU 的加速度与陀螺仪零偏，按原有 EEPROM 布局存储。
  float AccErrorX_6050;
  float AccErrorY_6050;
  float AccErrorZ_6050;

  float GyroErrorX_6050;
  float GyroErrorY_6050;
  float GyroErrorZ_6050;

  float AccErrorX_9250;
  float AccErrorY_9250;
  float AccErrorZ_9250;

  float GyroErrorX_9250;
  float GyroErrorY_9250;
  float GyroErrorZ_9250;
} CalibrationAccGyroData;

extern FC_Binary_Packet airdata;
extern CalibrationAccGyroData calAccGyroData;
extern SPISettings bmiSettings;
extern int32_t Strain_value1, Strain_value2, Strain_value3, Strain_value4,
    Strain_value5;
extern float AccX_6050, AccY_6050, AccZ_6050;
extern float GyroX_6050, GyroY_6050, GyroZ_6050;
extern float Gyro_X_EXT, Gyro_Y_EXT, Gyro_Z_EXT;
extern float Acc_X_EXT, Acc_Y_EXT, Acc_Z_EXT;
extern float roll_IMU_EXT, pitch_IMU_EXT, yaw_IMU_EXT;

constexpr int CS_ACC = 10;
constexpr int CS_GYR = 37;

void sendStrainCommand(uint8_t addr, uint16_t fcode, uint16_t reg,
                       uint16_t num);
// 传感器读取函数更新下方共享的测量值，调用方不持有硬件接口。
void Strain_read_all();
void getICM42688data();
void getBMI088data();
void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z);
void getairdata();
void getIMUdata_EXT();
void writeRegSPI(int cs, uint8_t reg, uint8_t val);

extern float AccX_9250, AccY_9250, AccZ_9250;
extern float GyroX_9250, GyroY_9250, GyroZ_9250;
extern float MagX_9250, MagY_9250, MagZ_9250;
extern float relativeAngle_raw, relativeAngle_ready;
extern float airspeed_A;
extern float dp, dq, dr;

void beginExternalImuLink();
void beginStrainSensorLink();
void loadImuCalibration();
bool imuCalibrationValid();
void initializeInitialAttitude();
void IMUinit();
void getIMUdata();
void calculate_IMU_error();
void calibrateMagnetometer();
void getairspeed();
// 角加速度估计器的专用滤波初始化；由 initializeControlFilters() 调用。
void initializeAngularAccelerationFilters();
void getAngularACC();
void getbarodata();
void initBAROMETER();
void initializeRotateSensor();
bool rotateSensorValid();
void getRotateSensor1();
void ResetRotateSensor();
void loadRotateSensorOffset();
void calibrateAirspeedSensor();
void calibrateAttitude();
