#pragma once

#include <Arduino.h>

// 二阶巴特沃斯低通滤波器，按 PX4 的系数计算方式实现。
class PX4LowPassFilter2p {
public:
  // 采样频率和截止频率均为 Hz；截止频率非正时直通。
  void set_cutoff_frequency(float sample_freq, float cutoff_freq);
  float apply(float sample);
  void reset(float sample);

private:
  float _a1{0.0f}, _a2{0.0f};
  float _b0{1.0f}, _b1{0.0f}, _b2{0.0f};
  float _delay_element_1{0.0f}, _delay_element_2{0.0f};
};

extern PX4LowPassFilter2p gyroFiltX, gyroFiltY, gyroFiltZ;
extern PX4LowPassFilter2p gyroFiltYIndi, gyroFiltXIndi, gyroFiltZIndi;
// 欧拉角为度；四元数无量纲。
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float q0, q1, q2, q3;

void initializeControlFilters();
// 使用飞行时钟中的 dt（秒）更新姿态。
void Madgwick();
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,
                  float invSampleFreq);
