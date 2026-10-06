#include "math_utils.h"
#include <Arduino.h>

using namespace Eigen;

// 按指定时间和调用频率在线性区间内增减参数；state 为 1 时上升，为 0 时下降。
// 返回限幅后的参数值；fadeTime 单位为秒。
float floatFaderLinear(float param, float param_min, float param_max,
                       float fadeTime, int state, int loopFreq) {
  // 按 fadeTime 秒从下限线性过渡到上限（state=1），或反向过渡（state=0）。
  // loopFreq 为调用频率，计算出的步长对应一次调用。
  float diffParam =
      (param_max - param_min) /
      (fadeTime * loopFreq);

  if (state == 1) {
    param = param + diffParam;
  } else if (state == 0) {
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max);

  return param;
}

// 分别用上升、下降时间将参数逼近目标值，并限制在给定上下界内。
// 时间单位为秒，loopFreq 单位为 Hz；返回本次更新值。
float floatFaderLinear2(float param, float param_des, float param_lower,
                        float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq) {
  // 分别按上升/下降时间向目标值逼近，最后限制在指定区间内。
  if (param > param_des) {
    float diffParam = (param_upper - param_des) / (fadeTime_down * loopFreq);
    param = param - diffParam;
  } else if (param < param_des) {
    float diffParam = (param_des - param_lower) / (fadeTime_up * loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper);

  return param;
}

// 返回正数 x 的平方根倒数，供向量和四元数归一化使用。
float invSqrt(float x) {
  // 姿态解算中的归一化运算；当前直接求平方根倒数。
  return 1.0 / sqrtf(x);
}

// 将滚转、俯仰、偏航欧拉角转换为 Eigen 四元数；输入角单位为弧度。
Quaternionf eulertoqua(float roll, float pitch, float yaw) {
  Quaternionf q;

  // 计算半角
  float cy = cosf(yaw * 0.5f);
  float sy = sinf(yaw * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);

  // 计算四元数分量
  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;

  return q;
}

// 将负指令截为零；非负指令保持原值。
float keeppositive(float command) {
  if (command > 0.0)
    return command;
  else
    return 0.0;
}

// 计算 q2 相对 q1 的姿态角，并结合两机 IMU 航向、滚转角修正坐标系。
// IMU 角输入及结果字段的单位均为度。
QuaternionDifferenceEuler quatDiffToEuler(const Eigen::Quaternionf &q1,
                                         const Eigen::Quaternionf &q2,
                                         float yawImuDeg, float rollImuDeg) {
  // 先按 IMU 航向差修正 q2，再把相对四元数投影到 q1 机体系。
  QuaternionDifferenceEuler result{};
  result.otherYawDeg =
      atan2(q2.x() * q2.y() + q2.w() * q2.z(),
            0.5f - q2.y() * q2.y() - q2.z() * q2.z()) *
      57.29577951;
  result.otherRollDeg =
      57.3 * atan2(2.0f * (q2.w() * q2.x() + q2.y() * q2.z()),
                   1.0f - 2.0f * (q2.x() * q2.x() + q2.y() * q2.y()));
  result.rollDifferenceDeg = -result.otherRollDeg - rollImuDeg;

  Eigen::Quaternionf q_rot =
      eulertoqua(0, 0, (yawImuDeg - result.otherYawDeg) / 57.295);
  Eigen::Quaternionf q2_revised = q_rot * q2;
  Eigen::Quaternionf q_diff = q2_revised * q1.conjugate();
  q_diff = q1.conjugate() * q_diff * q1; // 投影到q1上。

  float w = q_diff.w(), x = q_diff.x(), y = q_diff.y(), z = q_diff.z();
  float N = invSqrt(w * w + x * x);
  w *= N;
  x *= N;
  result.rollDeg = -57.3 * atan2(2.0f * (w * x), 1.0f - 2.0f * (x * x));
  result.pitchDeg = 57.3 * asin(2.0f * (w * y - z * x));
  result.yawDeg =
      57.3 * atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
  return result;
}

// 从 loopStartTime 起忙等到目标周期时长；freq 单位为 Hz，时间戳单位为 μs。
// 若处理已超时则立即返回，不会补偿遗漏的周期。
void loopRate(int freq, unsigned long loopStartTime) {
  // micros() 的无符号差值可处理计数器回绕；忙等期间 CPU 不执行其他循环任务。
  const float intervalMicros = 1000000.0f / freq;
  unsigned long checker = micros();
  while (intervalMicros > (checker - loopStartTime)) {
    checker = micros();
  }
}
