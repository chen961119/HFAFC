#include "math_utils.h"
#include <Arduino.h>

using namespace Eigen;

float floatFaderLinear(float param, float param_min, float param_max,
                       float fadeTime, int state, int loopFreq) {
  // DESCRIPTION: Linearly fades a float type variable between min and max
  // bounds based on desired high or low state and time
  /*
   *  Takes in a float variable, desired minimum and maximum bounds, fade time,
   * high or low desired state, and the loop frequency and linearly interpolates
   * that param variable between the maximum and minimum bounds. This function
   * can be called in controlMixer() and high/low states can be determined by
   * monitoring the state of an auxillarly radio channel. For example, if
   * channel_6_pwm is being monitored to switch between two dynamic
   * configurations (hover and forward flight), this function can be called
   * within the logical statements in order to fade controller gains, for
   * example between the two dynamic configurations. The 'state' (1 or 0) can be
   * used to designate the two final options for that control gain based on the
   * dynamic configuration assignment to the auxillary radio channel.
   *
   */
  float diffParam =
      (param_max - param_min) /
      (fadeTime * loopFreq); // Difference to add or subtract from param for
                             // each loop iteration for desired fadeTime

  if (state == 1) { // Maximum param bound desired, increase param by diffParam
                    // for each loop iteration
    param = param + diffParam;
  } else if (state == 0) { // Minimum param bound desired, decrease param by
                           // diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min,
                    param_max); // Constrain param within max bounds

  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower,
                        float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq) {
  // DESCRIPTION: Linearly fades a float type variable from its current value to
  // the desired value, up or down
  /*
   *  Takes in a float variable to be modified, desired new position, upper
   * value, lower value, fade time, and the loop frequency and linearly fades
   * that param variable up or down to the desired value. This function can be
   * called in controlMixer() to fade up or down between flight modes monitored
   * by an auxillary radio channel. For example, if channel_6_pwm is being
   *  monitored to switch between two dynamic configurations (hover and forward
   * flight), this function can be called within the logical statements in order
   * to fade controller gains, for example between the two dynamic
   * configurations.
   *
   */
  if (param > param_des) { // Need to fade down to get to desired
    float diffParam = (param_upper - param_des) / (fadeTime_down * loopFreq);
    param = param - diffParam;
  } else if (param < param_des) { // Need to fade up to get to desired
    float diffParam = (param_des - param_lower) / (fadeTime_up * loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower,
                    param_upper); // Constrain param within max bounds

  return param;
}

float invSqrt(float x) {
  // Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0 / sqrtf(x); // Teensy is fast enough to just take the compute
                         // penalty lol suck it arduino nano
}

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

float keeppositive(float command) {
  if (command > 0.0)
    return command;
  else
    return 0.0;
}

QuaternionDifferenceEuler quatDiffToEuler(const Eigen::Quaternionf &q1,
                                         const Eigen::Quaternionf &q2,
                                         float yawImuDeg, float rollImuDeg) {
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

void loopRate(int freq, unsigned long loopStartTime) {
  const float intervalMicros = 1000000.0f / freq;
  unsigned long checker = micros();
  while (intervalMicros > (checker - loopStartTime)) {
    checker = micros();
  }
}
