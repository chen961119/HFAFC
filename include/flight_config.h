#pragma once

// PWM 逻辑中位统一为 1500 μs；机械中位由各路 trim 修正。
constexpr int PWM_CENTER_US = 1500;

constexpr int PWM_SERVO1_MIN_US = 1000; // 左副翼
constexpr int PWM_SERVO1_MAX_US = 2000;
constexpr int PWM_SERVO2_MIN_US = 1000; // 右副翼
constexpr int PWM_SERVO2_MAX_US = 2000;
constexpr int PWM_SERVO3_MIN_US = 1000; // 升降
constexpr int PWM_SERVO3_MAX_US = 2000;
constexpr int PWM_SERVO4_MIN_US = 1000; // 油门
constexpr int PWM_SERVO4_MAX_US = 2000;
constexpr int PWM_SERVO5_MIN_US = 1000; // 方向
constexpr int PWM_SERVO5_MAX_US = 2000;

// 驱动范围独立于软件限幅，覆盖全部合法锁定位置（1500 + trim）。
constexpr int PWM_SERVO_ATTACH_MIN_US = 1000;
constexpr int PWM_SERVO_ATTACH_MAX_US = 2000;
constexpr bool validActuatorPwmLimits(int minimum, int maximum) {
  return PWM_SERVO_ATTACH_MIN_US <= minimum && minimum <= maximum &&
         maximum <= PWM_SERVO_ATTACH_MAX_US;
}
static_assert(validActuatorPwmLimits(PWM_SERVO1_MIN_US, PWM_SERVO1_MAX_US), "Invalid channel 1 PWM limits");
static_assert(validActuatorPwmLimits(PWM_SERVO2_MIN_US, PWM_SERVO2_MAX_US), "Invalid channel 2 PWM limits");
static_assert(validActuatorPwmLimits(PWM_SERVO3_MIN_US, PWM_SERVO3_MAX_US), "Invalid channel 3 PWM limits");
static_assert(validActuatorPwmLimits(PWM_SERVO4_MIN_US, PWM_SERVO4_MAX_US), "Invalid channel 4 PWM limits");
static_assert(validActuatorPwmLimits(PWM_SERVO5_MIN_US, PWM_SERVO5_MAX_US), "Invalid channel 5 PWM limits");

// 只启用一个机体编号；物理串联顺序为 F-D-B-A-C-E-G。
#define APLANE
// #define BPLANE
// #define CPLANE
// #define DPLANE
// #define EPLANE
// #define FPLANE
// #define GPLANE

#define TESTINDI

#define THREEPLANE
// #define FOURPLANE
// #define FIVEPLANE
// #define SEVENPLANE

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
