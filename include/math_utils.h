#pragma once

#include <ArduinoEigenDense.h>

void loopRate(int freq, unsigned long loopStartTime);

float invSqrt(float x);
float keeppositive(float command);
float floatFaderLinear(float param, float param_min, float param_max,
                       float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower,
                        float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq);
Eigen::Quaternionf eulertoqua(float roll, float pitch, float yaw);

struct QuaternionDifferenceEuler {
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float otherYawDeg;
  float otherRollDeg;
  float rollDifferenceDeg;
};

QuaternionDifferenceEuler quatDiffToEuler(const Eigen::Quaternionf &q1,
                                         const Eigen::Quaternionf &q2,
                                         float yawImuDeg, float rollImuDeg);
