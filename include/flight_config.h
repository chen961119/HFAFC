#pragma once

// Select one aircraft identity. The chain is F-D-B-A-C-E-G.
// #define APLANE
// #define BPLANE
// #define CPLANE
// #define DPLANE
// #define EPLANE
#define FPLANE
// #define GPLANE

// #define TESTBED
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

// Legacy IMU selection is also used by the inter-aircraft status sender.
#define USE_MPU6050_I2C
// #define USE_MPU9250_SPI

enum FlightMode {
  MANUAL_MODE,
  STABLIZE_MODE_NO_I,
  STABILIZE_MODE,
};

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// #define USE_BAROMETER
// //看你要不要用气压计，已知用了的话就会占用一个串口，并且要用掉3000us
// 要用的话记得板子上相应的要短接
