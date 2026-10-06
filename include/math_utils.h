#pragma once

#include <ArduinoEigenDense.h>

// 忙等到本轮达到目标周期；若本轮已超时则立即返回，无法补偿超时。
void loopRate(int freq, unsigned long loopStartTime);

float invSqrt(float x);
float keeppositive(float command);
float floatFaderLinear(float param, float param_min, float param_max,
                       float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower,
                        float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq);
Eigen::Quaternionf eulertoqua(float roll, float pitch, float yaw); // 输入角单位为弧度。

struct QuaternionDifferenceEuler {
  // 所有成员均为角度值（度）。
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
