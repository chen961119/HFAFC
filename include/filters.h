#pragma once

#include <Arduino.h>

// Two-pole Butterworth low-pass filter, adapted from PX4.
class PX4LowPassFilter2p {
public:
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
extern float dp, dq, dr;
extern float roll_IMU, pitch_IMU, yaw_IMU;
extern float q0, q1, q2, q3;

void initializeControlFilters();
void getAngularACC();
void Madgwick(float invSampleFreq);
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,
                  float invSampleFreq);
void eulerToQuaternion();
