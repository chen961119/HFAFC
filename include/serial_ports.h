#pragma once

#include <Arduino.h>

// 串口硬件映射集中在这里；业务模块按用途使用名称。
#if defined(HFAFC_TEENSY_DEBUG)
// 调试时普通打印丢弃、参数入口不读取，避免破坏同一串口上的 GDB 数据。
class QuietUsbSerial : public Stream {
public:
  void begin(unsigned long) {}
  int available() override { return 0; }
  int availableForWrite() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t *, size_t size) override { return size; }
  explicit operator bool() const { return false; }
};
inline QuietUsbSerial quietUsbSerial;
// 调试暂停时部分后台刷新不再执行；查询输入前主动提交 GDB 响应。
class GdbUsbSerial : public Stream {
public:
  void begin(unsigned long baud) { Serial.begin(baud); }
  int available() override { Serial.send_now(); return Serial.available(); }
  int availableForWrite() override { return Serial.availableForWrite(); }
  int read() override { return Serial.read(); }
  int peek() override { return Serial.peek(); }
  void flush() override { Serial.send_now(); }
  size_t write(uint8_t value) override { return Serial.write(value); }
  size_t write(const uint8_t *data, size_t size) override { return Serial.write(data, size); }
  explicit operator bool() const { return static_cast<bool>(Serial); }
};
inline GdbUsbSerial gdbUsbSerial;
#define USBSerial quietUsbSerial
#define DebugSerial gdbUsbSerial
#else
#define USBSerial Serial
#endif
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
