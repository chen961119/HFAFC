#include "filters.h"
#include "flight_config.h"
#include "sensor_processing.h"
#include "math_utils.h"
#include "flight_clock.h"

PX4LowPassFilter2p gyroFiltX, gyroFiltY, gyroFiltZ;
PX4LowPassFilter2p gyroFiltYIndi, gyroFiltXIndi, gyroFiltZIndi;

// 按采样频率和截止频率（Hz）计算二阶低通系数；非正参数时改为直接透传。
void PX4LowPassFilter2p::set_cutoff_frequency(float sample_freq, float cutoff_freq) {
  if (sample_freq <= 0.0f || cutoff_freq <= 0.0f) {
    _b0 = 1.0f; _b1 = 0.0f; _b2 = 0.0f;
    _a1 = 0.0f; _a2 = 0.0f;
    return;
  }
  const float fr = sample_freq / cutoff_freq;
  const float ohm = tanf(PI / fr);
  const float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;
  _b0 = ohm * ohm / c;
  _b1 = 2.0f * _b0;
  _b2 = _b0;
  _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
  _a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
}

// 输入一个新样本，更新二阶滤波器内部状态并返回滤波结果。
float PX4LowPassFilter2p::apply(float sample) {
  float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
  float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
  _delay_element_2 = _delay_element_1;
  _delay_element_1 = delay_element_0;
  return output;
}

// 将滤波器延迟状态重置到指定样本对应的稳态值。
void PX4LowPassFilter2p::reset(float sample) {
  _delay_element_1 = _delay_element_2 = sample / (1.0f + _a1 + _a2);
}

// 初始化姿态控制和 INDI 控制所用的独立角速度滤波器。
void initializeControlFilters() {
  // 控制环按 500 Hz 采样；INDI 与常规姿态控制使用独立滤波器状态。
  gyroFiltX.set_cutoff_frequency(500, 40);
  gyroFiltY.set_cutoff_frequency(500, 40);
  gyroFiltZ.set_cutoff_frequency(500, 40);
  initializeAngularAccelerationFilters();
  gyroFiltYIndi.set_cutoff_frequency(500, 40);
  gyroFiltXIndi.set_cutoff_frequency(500, 100);
  gyroFiltZIndi.set_cutoff_frequency(500, 100);
}

float B_madgwick = 0.04; // 九轴姿态校正权重。
float B_madgwick_adaptive = 0.04; // 动态权重
float base_B_madgwick = 0.04;     // 基准权重

float roll_IMU, pitch_IMU, yaw_IMU;

float q0 = 1.0f; // 初始姿态假设机体水平。
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;


// 使用九轴 Madgwick 算法更新共享姿态四元数与欧拉角；磁力计无效时改用六轴算法。
// 直接使用飞行时钟的 dt（秒）。
void Madgwick() {
#if defined INTIMU
  // 九轴姿态融合：陀螺仪积分预测姿态，加速度计与磁力计提供校正。
  // 当前仅启用 MPU6050 时走六轴分支；磁力计全零时也回退至六轴分支。
  // 使用本轮采样间隔 dt；输出欧拉角单位为度。
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
      _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
      q2q2, q2q3, q3q3;

// 仅有 MPU6050 时没有有效磁力计输入。
#if defined USE_MPU6050_I2C && !defined USE_MPU9250_SPI
  Madgwick6DOF(GyroX_6050, GyroY_6050, GyroZ_6050, AccX_6050, AccY_6050,
               AccZ_6050, dt);
  return;
#endif

  // 磁力计三轴全零时无法归一化，改用六轴算法。
  if ((MagY_9250 == 0.0f) && (-MagX_9250 == 0.0f) && (MagZ_9250 == 0.0f)) {
    Madgwick6DOF(GyroX_9250, GyroY_9250, GyroZ_9250, AccX_9250, AccY_9250,
                 AccZ_9250, dt);
    return;
  }

  // 读取传感器坐标轴；下方按姿态算法需要的方向重排磁力计轴。
  gx = GyroX_9250;
  gy = GyroY_9250;
  gz = GyroZ_9250;
  ax = AccX_9250;
  ay = AccY_9250;
  az = AccZ_9250;
  // 磁力计 X/Y 交换并反向，与当前传感器安装方向对应。
  mx = MagY_9250;
  my = -MagX_9250;
  mz = MagZ_9250;

  // 陀螺仪由 deg/s 换算为 rad/s。
  gx *= 0.0174533f; // 1/57.3
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // 根据角速度计算四元数变化率。
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // 加速度计全零时跳过反馈，避免归一化时除以零。
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    float Accnorm = sqrt(ax * ax + ay * ay + az * az);
    // 将加速度和地磁方向归一化，用于姿态误差计算。
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // 缓存重复使用的四元数乘积。
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // 将测得的地磁向量旋转到参考坐标系，估计水平与竖直分量。
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
         _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
         my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
           _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // 梯度下降得到姿态误差校正方向。
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q1 * (2.0f * q0q1 + _2q2q3 - ay) -
         _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q3 + _2bz * q1) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q0 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q2 + _2bz * q0) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q3 - _4bz * q1) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q3 * (2.0f * q0q1 + _2q2q3 - ay) -
         4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) +
         (-_4bx * q2 - _2bz * q0) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (_2bx * q1 + _2bz * q3) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         (_2bx * q0 - _4bz * q2) *
             (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) +
         _2q2 * (2.0f * q0q1 + _2q2q3 - ay) +
         (-_4bx * q3 + _2bz * q1) *
             (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
         (-_2bx * q0 + _2bz * q2) *
             (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
         _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // 归一化校正梯度。
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 按校正权重修正四元数变化率。
    /*
    if (abs(gz)>1)//1rad/s 修正
    {
      B_madgwick=0.04/(abs(gz)-1);
      }
      else
      {
        B_madgwick=0.04;
        }
    */
    float factor = 1.2 / Accnorm;
    qDot1 -= B_madgwick * factor * s0;
    qDot2 -= B_madgwick * factor * s1;
    qDot3 -= B_madgwick * factor * s2;
    qDot4 -= B_madgwick * factor * s3;
  }

  // 用本轮时间间隔积分，并重新归一化四元数。
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // 转回欧拉角；滚转角符号与机体安装坐标系约定一致。
  roll_IMU = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
             57.29577951; // 转为度。
  pitch_IMU =
      asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999, 0.999999)) *
      57.29577951; // 转为度。
  yaw_IMU = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
            57.29577951; // 转为度。

#endif
}

// 用陀螺仪和加速度计进行六轴姿态融合，更新共享四元数及欧拉角。
// gx、gy、gz 单位为 °/s；最后一个参数实际为采样间隔（秒）。
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,
                  float invSampleFreq) {
  // 六轴姿态融合：无磁力计时只用角速度与重力方向修正姿态。
  // 加速度模长偏离 1 g 越多，越降低加速度反馈权重。
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // 陀螺仪由 deg/s 换算为 rad/s。
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // 根据角速度计算四元数变化率。
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // 加速度计全零时不计算反馈，避免除以零。
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    // 1. 计算加速度向量的模
    float acc_norm = sqrt(ax * ax + ay * ay + az * az);

    // 2. 计算与 1.0g 的偏离值
    float acc_error = abs(acc_norm - 1.0f);

    // 3. 动态调整权重：如果偏离超过 0.1g，开始线性减小权重
    // 当偏离达到 0.5g 时，权重降至极低，几乎完全信任陀螺仪积分
    if (acc_error < 0.1f) {
      B_madgwick_adaptive = base_B_madgwick;
    } else {
      // 线性插值：偏离 0.1g~0.5g 之间，权重从 0.04 降到 0.001
      B_madgwick_adaptive =
          base_B_madgwick *
          (1.0f - constrain((acc_error - 0.1f) / 0.4f, 0.0f, 0.98f));
    }
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 缓存重复使用的四元数乘积。
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    // 梯度下降得到重力方向误差。
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // 归一化校正梯度。
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // 用自适应权重修正陀螺仪积分结果。

    qDot1 -= B_madgwick_adaptive * s0;
    qDot2 -= B_madgwick_adaptive * s1;
    qDot3 -= B_madgwick_adaptive * s2;
    qDot4 -= B_madgwick_adaptive * s3;
  }

  // 积分并归一化四元数。
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // 输出角度；滚转角按当前安装方向取反。
  roll_IMU = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
             57.29577951; // 转为度。
  pitch_IMU =
      asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999, 0.999999)) *
      57.29577951; // 转为度。
  yaw_IMU = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
            57.29577951; // 转为度。
}
