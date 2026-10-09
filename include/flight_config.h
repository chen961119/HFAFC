#pragma once

// PWM 逻辑中位统一为 1500 μs；机械中位由各路 trim 修正。
constexpr int PWM_CENTER_US = 1500;
// 保留原有中位上下的逻辑行程，换基准后限幅为 1080～1900 μs。
constexpr int PWM_SURFACE_MIN_US = PWM_CENTER_US - 420;
constexpr int PWM_SURFACE_MAX_US = PWM_CENTER_US + 400;

// 只启用一个机体编号；物理串联顺序为 F-D-B-A-C-E-G。
// #define APLANE
// #define BPLANE
// #define CPLANE
// #define DPLANE
// #define EPLANE
#define FPLANE
// #define GPLANE

#define TESTINDI

// #define THREEPLANE
// #define FOURPLANE
// #define FIVEPLANE
#define SEVENPLANE

// #define userotatesensor
#define expensive

// #define EXTIMU
#define INTIMU

// #define SINGLE
#define TEAM

#define ODD
// #define EVEN

#define USE_SBUS_RX

// IMU 选择宏同时决定机间状态发送使用的数据源。
#define USE_MPU6050_I2C
// #define USE_MPU9250_SPI

enum FlightMode {
  MANUAL_MODE,
  STABLIZE_MODE_NO_I,
  STABILIZE_MODE,
};

// 陀螺仪量程只能启用一项，单位 deg/s。
#define GYRO_250DPS // 当前默认量程
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// 加速度计量程只能启用一项，单位 g。
#define ACCEL_2G // 当前默认量程
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// #define USE_BAROMETER
// 启用气压计前需确认板上焊点已短接，并评估读取耗时对循环周期的影响。
