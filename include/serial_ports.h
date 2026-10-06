#pragma once

#include <Arduino.h>

// 串口硬件映射集中在这里；业务模块按用途使用名称。
#define USBSerial Serial
#define ParentSerial Serial6
#define LeftChildSerial Serial3
#define RightChildSerial Serial5
#define ExternalImuSerial Serial1
#define SbusSerial Serial2
#define DsmSerial Serial3
#define StrainSensorSerial Serial7
#define AngleSensorSerial Serial7
#define TelemetrySerial Serial8

// DSM 与左子机共用 Serial3；应变与转角传感器共用 Serial7。
// 这些映射保持现有接线，相应设备由当前配置决定。
