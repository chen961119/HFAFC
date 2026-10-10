#include "telemetry.h"
#include "serial_ports.h"
#include "control_state.h"
#include "interaircraft_comm.h"
#include "sensor_processing.h"

namespace {
float configuration_tele[6];
float attitude_tele[3];
unsigned long lasttelemetryTime = 0;
unsigned int Freqtelemetry = 5; // 数传频率 Hz。
} // namespace

void beginTelemetryLink() {
  TelemetrySerial.begin(115200);
}

// 以设定频率把六个构型角和两个姿态角打包发送到 TelemetrySerial。
// 数据帧含 0x55 0x71 帧头，角度放大 10 倍后以 int16_t 传输。
void telemetry() // 主机数传
{
  // 数据包结构：2 字节帧头 + 8 个 int16_t + 1 字节校验和，共 19 字节。
  uint8_t buffer[19];
  uint8_t pos = 0;
  uint8_t checksum = 0;

  float invFreq = 1.0 / Freqtelemetry * 1000000.0;
  unsigned long checker3 = micros();

  if (checker3 - lasttelemetryTime < invFreq)
    return;
  lasttelemetryTime = checker3;

  configuration_tele[0] = phiB_raw - roll_IMU; //
  configuration_tele[1] = phiC_raw - roll_IMU; //
  configuration_tele[2] = phiD_raw - phiB_raw;
  configuration_tele[3] = phiE_raw - phiC_raw;
  configuration_tele[4] = phiF_raw - phiD_raw;
  configuration_tele[5] = phiG_raw - phiE_raw;

  attitude_tele[0] = roll_eq;
  attitude_tele[1] = pitch_IMU;

  //  float anglerand = random(-200000, 200001) / 1000.0;  // 范围 -200.000 ~
  //  +200.000

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x71;
  checksum += 0x71; // 新类型标识

  // 2. 打包相对转角6个 (转换为int16_t 一位小数)
  for (int i = 0; i < 6; i++) {
    int16_t val = configuration_tele[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 3.打包欧拉角
  for (int i = 0; i < 2; i++) {
    int16_t val = attitude_tele[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 4. 校验和
  buffer[pos] = checksum;
  // 5. 发送
  TelemetrySerial.write(buffer, sizeof(buffer));
}


