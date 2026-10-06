#include "filters.h"
#include "flight_config.h"
#include "sensor_processing.h"
#include "math_utils.h"

PX4LowPassFilter2p gyroFiltX, gyroFiltY, gyroFiltZ;
PX4LowPassFilter2p gyroFiltYIndi, gyroFiltXIndi, gyroFiltZIndi;

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

float PX4LowPassFilter2p::apply(float sample) {
  float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
  float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
  _delay_element_2 = _delay_element_1;
  _delay_element_1 = delay_element_0;
  return output;
}

void PX4LowPassFilter2p::reset(float sample) {
  _delay_element_1 = _delay_element_2 = sample / (1.0f + _a1 + _a2);
}

// 角加速度低通滤波器（三轴）
static PX4LowPassFilter2p gyroDerivFiltX, gyroDerivFiltY,
    gyroDerivFiltZ; // getAngularACC() 求 dp/dq/dr 前的角速度滤波

static PX4LowPassFilter2p angularAccFiltX, angularAccFiltY, angularAccFiltZ;

void initializeAngularAccelerationFilters() {
  angularAccFiltX.set_cutoff_frequency(500, 100);
  angularAccFiltY.set_cutoff_frequency(500, 20);
  angularAccFiltZ.set_cutoff_frequency(500, 100);
  gyroDerivFiltX.set_cutoff_frequency(500, 100);
  gyroDerivFiltY.set_cutoff_frequency(500, 40);
  gyroDerivFiltZ.set_cutoff_frequency(500, 100);
}

// 前次滤波后陀螺仪值（用于差分）
float gyroX_filt_prev = 0.0f;
float gyroY_filt_prev = 0.0f;
float gyroZ_filt_prev = 0.0f;
// 前次差分时间（微秒）
unsigned long prev_time_gyro_deriv = 0;

float dp, dq, dr;

void initializeControlFilters() {
  gyroFiltX.set_cutoff_frequency(500, 40);
  gyroFiltY.set_cutoff_frequency(500, 40);
  gyroFiltZ.set_cutoff_frequency(500, 40);
  initializeAngularAccelerationFilters();
  gyroFiltYIndi.set_cutoff_frequency(500, 40);
  gyroFiltXIndi.set_cutoff_frequency(500, 100);
  gyroFiltZIndi.set_cutoff_frequency(500, 100);
}

void getAngularACC() {
  unsigned long now = micros();

  float dt_deriv = (now - prev_time_gyro_deriv) * 1.0e-6f;
  prev_time_gyro_deriv = now;

  if (dt_deriv <= 0.0f || dt_deriv > 0.01f) {
    gyroX_filt_prev = gyroDerivFiltX.apply(-GyroX_6050);
    gyroY_filt_prev = gyroDerivFiltY.apply(GyroY_6050);
    gyroZ_filt_prev = gyroDerivFiltZ.apply(GyroZ_6050);
    dp = 0.0f;
    dq = 0.0f;
    dr = 0.0f;
    return;
  }

  float gyroX_now = gyroDerivFiltX.apply(-GyroX_6050);
  float gyroY_now = gyroDerivFiltY.apply(GyroY_6050);
  float gyroZ_now = gyroDerivFiltZ.apply(GyroZ_6050);

  float dp_raw = (gyroX_now - gyroX_filt_prev) / dt_deriv;
  float dq_raw = (gyroY_now - gyroY_filt_prev) / dt_deriv;
  float dr_raw = (gyroZ_now - gyroZ_filt_prev) / dt_deriv;

  dp = angularAccFiltX.apply(dp_raw);
  dq = angularAccFiltY.apply(dq_raw);
  dr = angularAccFiltZ.apply(dr_raw);

  gyroX_filt_prev = gyroX_now;
  gyroY_filt_prev = gyroY_now;
  gyroZ_filt_prev = gyroZ_now;

  const float deriv_limit = 3000.0f;
  dp = constrain(dp, -deriv_limit, deriv_limit);
  dq = constrain(dq, -deriv_limit, deriv_limit);
  dr = constrain(dr, -deriv_limit, deriv_limit);
}


float B_madgwick = 0.04; // Madgwick filter parameter 后面定义过了default 0.04
float B_madgwick_adaptive = 0.04; // 动态权重
float base_B_madgwick = 0.04;     // 基准权重

float roll_IMU, pitch_IMU, yaw_IMU;

float q0 = 1.0f; // Initialize quaternion for madgwick filter 假设直立
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;


// Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
void Madgwick(float invSampleFreq) {
  // DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX,
   * AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude
   * estimation. Don't worry about the math. There is a tunable parameter
   * B_madgwick in the user specified variable section which basically adjusts
   * the weight of gyro data in the state estimate. Higher beta leads to noisier
   * estimate, lower beta leads to slower to respond estimate. It is currently
   * tuned for 2kHz loop rate. This function updates the roll_IMU, pitch_IMU,
   * and yaw_IMU variables which are in degrees. If magnetometer data is not
   * available, this function calls Madgwick6DOF() instead.
   */
  float gx, gy, gz, ax, ay, az, mx, my, mz;
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
      _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3,
      q2q2, q2q3, q3q3;

// use 6DOF algorithm if only MPU6050 is being used
#if defined USE_MPU6050_I2C && !defined USE_MPU9250_SPI
  Madgwick6DOF(GyroX_6050, GyroY_6050, GyroZ_6050, AccX_6050, AccY_6050,
               AccZ_6050, invSampleFreq);
  return;
#endif

  // Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in
  // magnetometer normalisation) 用了9250但是磁力计坏了
  if ((MagY_9250 == 0.0f) && (-MagX_9250 == 0.0f) && (MagZ_9250 == 0.0f)) {
    Madgwick6DOF(GyroX_9250, GyroY_9250, GyroZ_9250, AccX_9250, AccY_9250,
                 AccZ_9250, invSampleFreq);
    return;
  }

  // Madgwick6DOF(GyroX_9250, GyroY_9250, GyroZ_9250, AccX_9250, AccY_9250,
  // AccZ_9250, invSampleFreq);
  //   return;

  // Convert gyroscope degrees/sec to radians/sec
  gx = GyroX_9250;
  gy = GyroY_9250;
  gz = GyroZ_9250;
  ax = AccX_9250;
  ay = AccY_9250;
  az = AccZ_9250;
  // IMU的磁力计输出有问题。确实应该这样转一下
  mx = MagY_9250;
  my = -MagX_9250;
  mz = MagZ_9250;

  // 转换一下以满足飞行力学上的坐标定义

  gx *= 0.0174533f; // 1/57.3
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Serial.println(gx -gy -gz);
  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    float Accnorm = sqrt(ax * ax + ay * ay + az * az);
    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
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

    // Reference direction of Earth's magnetic field
    // 从传感器测量值，乘上姿态四元数，得到解算的地磁方向 地磁方向已知吗？
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 +
         _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 +
         my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 +
           _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
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
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
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

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  // Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // compute angles - NWU
  roll_IMU = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
             57.29577951; // degrees
  pitch_IMU =
      asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999, 0.999999)) *
      57.29577951; // degrees
  yaw_IMU = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
            57.29577951; // degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,
                  float invSampleFreq) {
  // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF
   * implimentation for when magnetometer data is not available (for example
   * when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  // Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope  //和飞行动力学课本一样
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in
  // accelerometer normalisation)
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
    // Serial.println(B_madgwick_adaptive);
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
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

    // Gradient decent algorithm corrective step 拿加速度计修正陀螺仪。
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 +
                        s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    // qDot1 -= B_madgwick * s0;
    // qDot2 -= B_madgwick * s1;
    // qDot3 -= B_madgwick * s2;
    // qDot4 -= B_madgwick * s3;

    qDot1 -= B_madgwick_adaptive * s0;
    qDot2 -= B_madgwick_adaptive * s1;
    qDot3 -= B_madgwick_adaptive * s2;
    qDot4 -= B_madgwick_adaptive * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  // Compute angles
  roll_IMU = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
             57.29577951; // degrees 额外加了个负号
  pitch_IMU =
      asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999, 0.999999)) *
      57.29577951; // degrees
  yaw_IMU = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
            57.29577951; // degrees
}


void eulerToQuaternion() {
  // 将角度从度数转换为弧度

  float phi, theta, psi;
#if defined USE_MPU6050_I2C
  phi = atan2(AccY_6050, AccZ_6050); // Roll (绕 x 轴)
  theta = atan2(-AccX_6050, sqrt(AccY_6050 * AccY_6050 +
                                 AccZ_6050 * AccZ_6050)); // Pitch (绕 y 轴)
  psi = 0;                                                // Yaw (绕 z 轴)
#endif

#if defined USE_MPU9250_SPI
  phi = atan2(AccY_9250, AccZ_9250); // Roll (绕 x 轴)
  theta = atan2(-AccX_9250, sqrt(AccY_9250 * AccY_9250 +
                                 AccZ_9250 * AccZ_9250)); // Pitch (绕 y 轴)
  psi = atan2(-MagX_9250, MagY_9250);                     // Yaw (绕 z 轴)
#endif

  // 计算中间变量
  float cosPhi_2 = cos(phi / 2.0);
  float sinPhi_2 = sin(phi / 2.0);
  float cosTheta_2 = cos(theta / 2.0);
  float sinTheta_2 = sin(theta / 2.0);
  float cosPsi_2 = cos(psi / 2.0);
  float sinPsi_2 = sin(psi / 2.0);

  // 计算四元数的各分量

  q0 = cosPhi_2 * cosTheta_2 * cosPsi_2 + sinPhi_2 * sinTheta_2 * sinPsi_2;
  q1 = sinPhi_2 * cosTheta_2 * cosPsi_2 - cosPhi_2 * sinTheta_2 * sinPsi_2;
  q2 = cosPhi_2 * sinTheta_2 * cosPsi_2 + sinPhi_2 * cosTheta_2 * sinPsi_2;
  q3 = cosPhi_2 * cosTheta_2 * sinPsi_2 - sinPhi_2 * sinTheta_2 * cosPsi_2;
}
