#pragma once

#include <ArduinoEigenDense.h>

float invSqrt(float x);
float keeppositive(float command);
float floatFaderLinear(float param, float param_min, float param_max,
                       float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower,
                        float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq);
// phi/theta/psi 分别为绕 X/Y/Z 轴的滚转/俯仰/偏航角，单位为弧度。
// 返回四元数，不修改共享姿态状态。
Eigen::Quaternionf eulerToQuaternion(float phi, float theta, float psi);

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
