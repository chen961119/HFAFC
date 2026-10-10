#pragma once

#include <Arduino.h>

// 串口硬件映射集中在这里；业务模块按用途使用名称。
#if defined(COFLY_TEENSY_DEBUG)
#if !defined(USB_DUAL_SERIAL) || defined(USB_SERIAL)
#error "CoFly debug requires USB_DUAL_SERIAL without USB_SERIAL"
#endif
// 调试暂停时部分后台刷新不再执行；查询输入前主动提交 GDB 响应。
class GdbUsbSerial : public Stream {
public:
  void begin(unsigned long baud) { SerialUSB1.begin(baud); }
  int available() override { SerialUSB1.send_now(); return SerialUSB1.available(); }
  int availableForWrite() override { return SerialUSB1.availableForWrite(); }
  int read() override { return SerialUSB1.read(); }
  int peek() override { return SerialUSB1.peek(); }
  void flush() override { SerialUSB1.send_now(); }
  size_t write(uint8_t value) override { return SerialUSB1.write(value); }
  size_t write(const uint8_t *data, size_t size) override { return SerialUSB1.write(data, size); }
  explicit operator bool() const { return static_cast<bool>(SerialUSB1); }
};
inline GdbUsbSerial gdbUsbSerial;
#define DebugSerial gdbUsbSerial
#endif
// 第一 USB CDC 在普通、调试固件中始终保留给地面站与常规输出。
#define USBSerial Serial
#define ParentSerial Serial6
#define LeftChildSerial Serial3
#define RightChildSerial Serial5
#define ExternalImuSerial Serial1
#define SbusSerial Serial2
#define DsmSerial Serial3
#define StrainSensorSerial Serial7
#define TelemetrySerial Serial8

// DSM 与左子机共用 Serial3；应变传感器使用 Serial7。
// MT6701 转角传感器使用 Wire1，不占用串口。
// 这些映射保持现有接线，相应设备由当前配置决定。
