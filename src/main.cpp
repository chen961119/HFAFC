// Arduino/Teensy Flight Controller - dRehmFlight
// Author: Nicholas Rehm
// Project Start: 1/6/2020
// Last Updated: 7/29/2022
// Modified by Zhu 2025年4月5日
// Version: Beta 1.3s

//========================================================================================================================//

// CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

现在是双imu读取。

*/

/*
//开发注意事项
1.加了功能要注意运算频率能不能保证，用printlooprate看
*/

/*
//已经改了的地方：
1.sbus接收机
2.mpu9250
3.通道顺序
4.接受中立点改为1520
5.舵机指令中立也是1520 并加入trim变量
6.查找所有13变量和闪烁功能，全部干掉 放出10 11 12
7.增加了可靠的sd卡记录功能，100hz频率，断电保存，最后两秒没有。偶尔卡会读取不成功。
8.现在是双imu读取。
9.现在是双imu读取。
10.controlRATE()那里可能有坑，因为注释掉了gyro_PREV
11.删掉了6050的自检。要自己注意了
12.接收机rx2口
13.转角offset会存在eeprom里。
14.x，z轴方向反了一下。因为imu安装位置的问题。
在姿态估计（Roll——IMU上加了个负号），内环姿态pqr控制上要下功夫
15.加入协调转弯功能
16.epprom存储传感器校准值
17.短按是转角清零，长按是关闭显示。
18.rate模式下，期望值*3了
19.改数量要改混控模型，要改等效姿态,改rollrate
20.5机控制分配，实时。
21.phiab=relativeAngle_ready;
22.新增AB机之间的四元数传输，用A机的Serial3(getQuaternion)和B机的Serial3(setQuaternion).
23.改成外置IMU，通过串口1，所有的控制面改成23456口
24.增加等效clp。
25.31口增加一个按键，用来启动校准。
26.飞行模式改为 手动-增稳（无积分）-增稳（有积分）
其中飞行速度高了之后才用有积分 初始化都是I不可用
27.传输回来的增加了当地欧拉角，俯仰角
28.对贵的飞机，需要增加舵面限幅，副翼在1128行附近，升降在2821行，在指令发布的地方,在等效clp的地方都有。
29.增加数传功能。
30.贵飞机的电调需要上电的时候给一个低电平。不插sd卡过不了。
31.每个飞机的副翼微调不一样。在传输的时候加上，但是在log里减掉。
32.应变传感器功能加入
33.如果考虑拉起的话，可以分段，前半段控角速度，后半段再控角度
34.5/7机放弃多混控策略。6通道改成构型指令 水平/下反/Z字 （上，中，下）
35.增加空速传感器，插在I2c1，跟屏幕共用。其中cuav的和px4的模块动静压是反的
36.增加巴特沃斯滤波器，有采样频率参数，记得跟随实际情况设置。
37.现在是SPI读取BMI088
38.增加应急手动模式。升降舵也是单独给出的。elePwmCommandsRight/left
39.对ROLL——pid低通滤波roll_PID_lpf，用来送给扭转。
40.增加刹车
41.channel7大于1600关闭构型控制
42.俯仰INDI可选。channel6最下面就是INDI
43.B_ele_command_PWM_FF=100*roll_PID;增加了扭转前馈,在铰链工况下，扭转前馈是1度扭转1度舵，约等于10个pwm。
44.TESTBED表示单机。

还希望实现的功能：
1.两个传感器的时候，挂掉任何一个都ok  挂掉一个切另一个
有一个误差过大切另一个，并且禁用角加速度计算，那么问题来了，如何判断挂掉了，如何判断读数不可靠？

飞控板上的implements：
1.按下某个开关才让显示屏显示，因为只有在地上才有用，在天上显示还耗费资源。
2.板载气压计，但是用焊点隔开，可以让用户选择要不要连上。
3.自带rs232转换
4.按下按键开始姿态校准
*/

/*
 硬件说明
  按键是36口
  口01234是pwm输出。
  serial2的rx是接收机
  Serial1是外置传感器
  Serial8是数传。

  读取角度传感器的是serial7。 读取应变传感器也给7
  主机左边：
  发送接受都是serial3
  主机右边：
  发送接受都是serial5 //serial4不行的 给别人了
  从机：
  接受发送给上一级是Serial6
  接受发送给下一级是Serial3/5
  左边是3右边是5
  FDBACEG

  F6 - 3D6 - 3B6 - 3A5 - 6C5 - 6E5 - 6G

  实验说明
  不要加端板了
 */

// 遥控说明
// 5通道是控制模式
// 6通道是滚转模式
// 7通道是构型控制模式
// 8通道是刹车

// 关注可能的死机情况

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES //
//========================================================================================================================//

#include "BMI088.h"
#include "MS4525.h"
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SPI.h>

using namespace Eigen;

// 1. 定义 SPI 片选引脚
const int CS_ACC = 10;
const int CS_GYR = 37;

// 2. SPI 通讯设置 (10MHz, 模式3)
SPISettings bmiSettings(10000000, MSBFIRST, SPI_MODE3);

// 3. BMI088 寄存器读掩码
const uint8_t SPI_READ = 0x80;

#pragma pack(push, 1)
struct FC_Binary_Packet {
  uint8_t head1;
  uint8_t head2;
  float aoa;
  float aos;
  float tas;
  uint8_t statA;
  uint8_t statS;
  uint8_t checksum;
};
#pragma pack(pop)

FC_Binary_Packet airdata;

// ========================================================================================================================//
//                                            FUNCTION PROTOTYPES (前向声明) //
// ========================================================================================================================//
// 核心控制与姿态解算
void getairspeed();
void getIMUdata();
void Madgwick(float invSampleFreq);
void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az,
                  float invSampleFreq);
void getDesState();
void controlANGLE();
void controlANGLE2();
void controlRATE();
int PITCH_INDI_control();
void controlMixer();
void scaleCommands();
void loopRate(int freq);
void getCommands();
void failSafe();
void armedStatus();
void IMUinit();
void calculate_IMU_error();
void calibrateAttitude();
void radioSetup();
void eulerToQuaternion();
void getICM42688data();
void getBMI088data();
void writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val);
void writeRegSPI(int cs, uint8_t reg, uint8_t val);
void getairdata();
// 传感器与辅助计算
void getRotateSensor1();
void ResetRotateSensor();
void ProcessButtonState();
void controlFlapMotion();
void increase_Clp();
void getAngularACC();
float invSqrt(float x);
float keeppositive(float command);
float floatFaderLinear(float param, float param_min, float param_max,
                       float fadeTime, int state, int loopFreq);
float floatFaderLinear2(float param, float param_des, float param_lower,
                        float param_upper, float fadeTime_up,
                        float fadeTime_down, int loopFreq);

// 组队通讯与数传
void telemetry();
void sendGYROxANGLE();
void receiveCommandData();
void getGYROxANGLEleft();
void getGYROxANGLEright();
void sendAllDataleft(int servoCommandsleft[12], float pitchAnglesleft[3],
                     int elePwmLeft[3], int eleFFPwmLeft[3],
                     bool lightSignalsleft[3], bool int_is_valid);
void sendAllDataright(int servoCommandsright[12], float pitchAnglesright[3],
                      int elePwmRight[3], int eleFFPwmRight[3],
                      bool lightSignalsright[3], bool int_is_valid);

// 硬件外设与显示 (OLED & SD & Baro)
void displaythumbsup();
void displayID(char *ss);
void displaySD(char *ss);
void displayfilenum();
void displayAttitude();
void findMaxFileNumber();
void loggerSINGLE();
void loggerTEAM();
void initBAROMETER();
void getbarodata();

// 姿态差计算相关 (涉及 Eigen)
void quatDiffToEuler(const Eigen::Quaternionf &q1, const Eigen::Quaternionf &q2,
                     float &roll, float &pitch, float &yaw);
Eigen::Quaternionf eulertoqua(float roll, float pitch, float yaw);
void getIMUdata_EXT();

// 矩阵运算
void initializeInertiaMatrices(Matrix3f &Ia, Matrix3f &Ib, Matrix3f &Ic,
                               Matrix3f &Id, Matrix3f &Ie);
void initializeRotationMatrices(Matrix3f &Eab, Matrix3f &Eac, Matrix3f &Ebd,
                                Matrix3f &Ece);
void initializeH();
void getpinvBplusmini();

// 调试打印
void printRadioData();
void printDesiredState();
void printGyroData();
void printAccelData();
void printMagData();
void printRollPitchYaw();
void printPIDoutput();
void printMotorCommands();
void printServoCommands();
void printLoopRate();
void printQuaternion();
void printConfigurationData();
void printReceivedData();
void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z);

// 占位/待实现函数 (代码末尾出现的空函数)
void commandMotors();
void armMotors();
void calibrateESCs();
void switchRollYaw(int reverseRoll, int reverseYaw);
void throttleCut();
void calibrateMagnetometer();

// 应变传感器
void Strain_read_all();
void sendStrainCommand();
uint16_t modbusCRC();
int32_t parseChannel();

// 巴特沃斯滤波器
// 参考 PX4 LowPassFilter2p.hpp 移植
class PX4LowPassFilter2p {
public:
  PX4LowPassFilter2p() {}

  // 初始化/更改截止频率 (采样率, 截止频率)
  void set_cutoff_frequency(float sample_freq, float cutoff_freq) {
    if (sample_freq <= 0.0f || cutoff_freq <= 0.0f) {
      _b0 = 1.0f;
      _b1 = 0.0f;
      _b2 = 0.0f;
      _a1 = 0.0f;
      _a2 = 0.0f;
      return;
    }

    // 计算 Butterworth 系数 (基于 PX4 实现)
    const float fr = sample_freq / cutoff_freq;
    const float ohm = tanf(PI / fr);
    const float c = 1.0f + 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm;

    _b0 = ohm * ohm / c;
    _b1 = 2.0f * _b0;
    _b2 = _b0;
    _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    _a2 = (1.0f - 2.0f * cosf(PI / 4.0f) * ohm + ohm * ohm) / c;
  }

  // 应用过滤器 (直接 II 型实现)
  float apply(float sample) {
    float delay_element_0 =
        sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
    float output =
        delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
  }

  // 重置状态
  void reset(float sample) {
    _delay_element_1 = _delay_element_2 = sample / (1.0f + _a1 + _a2);
  }

private:
  float _a1{0.0f}, _a2{0.0f};
  float _b0{1.0f}, _b1{0.0f}, _b2{0.0f};
  float _delay_element_1{0.0f}, _delay_element_2{0.0f};
};

// 为控制内环实例化 3 轴滤波器
PX4LowPassFilter2p gyroFiltX, gyroFiltY, gyroFiltZ;
// 为单独的 INDI 俯仰控制保留独立滤波器，避免影响现有控制链的滤波状态
PX4LowPassFilter2p gyroFiltYIndi, gyroFiltXIndi, gyroFiltZIndi;
// 角加速度低通滤波器（三轴）
PX4LowPassFilter2p gyroDerivFiltX, gyroDerivFiltY,
    gyroDerivFiltZ; // getAngularACC() 求 dp/dq/dr 前的角速度滤波

PX4LowPassFilter2p angularAccFiltX, angularAccFiltY, angularAccFiltZ;

// ========================================================================================================================//
MatrixXf M_pinv(18, 18);
MatrixXf H(35, 20);   // 使用float单精度
MatrixXf Mp(10, 18);  // 共提取10行（3+3+1+1+1+1）
MatrixXf Mpi(10, 10); // 共提取10行（3+3+1+1+1+1）
MatrixXf M(18, 18);
MatrixXf Bplusmini(4, 20);
MatrixXf Bplusminismall(4, 10);
MatrixXf Bplusfull(6, 20);
MatrixXf Bplusfullsmall(6, 10);
MatrixXf Bplusminismall_pinv(10, 4);
MatrixXf Bplusfullsmall_pinv(10, 6);
Matrix3f Eab, Eac, Ebd, Ece;
Matrix3f Ia, Ib, Ic, Id, Ie;
VectorXf dw_config(4);  // 四个变形通道
VectorXf dw_att(6);     // 6个整体运动通道
VectorXf de_att(10);    // 只考虑所有的副翼
VectorXf de_config(10); // 只考虑所有的副翼
VectorXf Qx(10);        // 广义力
VectorXf dw_rev(10);    // 广义加速度

// 常量定义（与MATLAB完全一致）
float Mass = 0.8;
float Mbss = 0.8;
float Mcss = 0.8;
float Mdss = 0.8;
float Mess = 0.8;

// 惯性张量（严格对应MATLAB）
float iaxx = 0.05, iaxy = 0, iaxz = 0.0, iayy = 0.08, iayz = 0, iazz = 0.13;
float ibxx = 0.05, ibxy = 0, ibxz = 0.0, ibyy = 0.08, ibyz = 0, ibzz = 0.13;
float icxx = 0.05, icxy = 0, icxz = 0.0, icyy = 0.08, icyz = 0, iczz = 0.13;
float idxx = 0.05, idxy = 0, idxz = 0, idyy = 0.08, idyz = 0, idzz = 0.13;
float iexx = 0.05, iexy = 0, iexz = 0, ieyy = 0.08, ieyz = 0, iezz = 0.13;

// 几何参数
float span = 1.5; // 需要设置实际值
float y = 0.5 * span;
float c = 0.21;
float S = 0.32;

// 位置向量（严格对应MATLAB）
Vector3f Rab(0, -y, 0);
Vector3f Rbm(0, -y, 0);
Vector3f Rac(0, y, 0);
Vector3f Rcm(0, y, 0);
Vector3f Rbd(0, -y, 0);
Vector3f Rdm(0, -y, 0);
Vector3f Rce(0, y, 0);
Vector3f Rem(0, y, 0);

float yaw_IMU_B, roll_diff;
float roll_IMU_B;

// 选择当前是哪一个飞机
// 其中A机是主机，需要接受遥控器信号，其他飞机都不需要接受遥控器。 GDBACEF
//  #define APLANE
//  #define BPLANE
//  #define CPLANE
//  #define DPLANE
// #define EPLANE
#define FPLANE
// #define GPLANE

// 飞机顺序：FDBACEG

// 是不是测试的载机
// #define TESTBED

// 要不要玩INDI
#define TESTINDI

// 选择连接几个
// #define THREEPLANE
// #define FOURPLANE
// #define FIVEPLANE
#define SEVENPLANE

// 用不用转角传感器
// #define userotatesensor

#define expensive // 贵飞机

// #define EXTIMU //用外置imu
#define INTIMU

// #define SINGLE
#define TEAM

#define ODD
// #define EVEN

float central_pitch = 0.0; // 偶数的时候中间体的俯仰角增量

enum FlightMode {
  MANUAL_MODE,        // 手动模式
  STABLIZE_MODE_NO_I, // 增稳模式无积分
  STABILIZE_MODE,     // 增稳模式有积分

};
FlightMode currentMode;
FlightMode lastMode;
bool int_is_valid = false;
bool force_manual = false;

bool ModeChange = 0;

// 接收数据结构 最多是接受这么多 每个从机的
struct {
  int servo[12];
  float pitch[3];
  int ele_pwm[3];
  int ele_ff_pwm[3];
  bool lights[3];
  bool int_is_valid;
  bool Force_manual;
} recvData;

// 串口接收配置
// char serial6Buffer[1024];  // 大缓冲区应对高速率
uint16_t bufIndex = 0;
bool frameStarted = false;

typedef struct {
  float AccErrorX_6050;
  float AccErrorY_6050;
  float AccErrorZ_6050;

  float GyroErrorX_6050;
  float GyroErrorY_6050;
  float GyroErrorZ_6050;

  float AccErrorX_9250;
  float AccErrorY_9250;
  float AccErrorZ_9250;

  float GyroErrorX_9250;
  float GyroErrorY_9250;
  float GyroErrorZ_9250;
} CalibrationAccGyroData;

CalibrationAccGyroData calAccGyroData;

// 通讯相关
#define RX_BUFFER_SIZE 250
static unsigned char ucRx2Buffer_S1[RX_BUFFER_SIZE]; // 缓冲区
static unsigned char ucRx2Cnt_S1 = 0;                // 缓冲区计数
static unsigned char ucRx2Buffer_S7[RX_BUFFER_SIZE]; // 缓冲区
static unsigned char ucRx2Cnt_S7 = 0;
static unsigned char ucDataUpdateFlag_S1 = 0; // 数据更新标志
static unsigned char ucDataUpdateFlag_S7 = 0; // 数据更新标志
float fAngle;
float relativeAngle_raw, relativeAngle_ready, relativeAngle_offset;
const int eepromAddress1 = 100;

// 按键相关
const int DEBOUNCE_DELAY = 10;              // 消抖时间(ms)
const unsigned long SHORT_PRESS_TIME = 50;  // 短按时间(ms)
const unsigned long LONG_PRESS_TIME = 1000; // 长按时间(ms)
bool buttonActive = false;
bool longPressActive = false;
unsigned long buttonPressTime = 0;

bool buttonActive1 = false; // 第二个按键
bool longPressActive1 = false;
unsigned long buttonPressTime1 = 0;

String serialBuffer = "";

// 巡航速度
float V_cruise = 13;

// Uncomment only one receiver type
// #define USE_PWM_RX
// #define USE_PPM_RX
#define USE_SBUS_RX
// #define USE_DSM_RX
static const uint8_t num_DSM_channels =
    6; // If using DSM RX, change this to match the number of transmitter
       // channels you have

// Uncomment only one IMU
#define USE_MPU6050_I2C // Default
// #define USE_MPU9250_SPI

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // Default
// #define GYRO_500DPS
// #define GYRO_1000DPS
// #define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // Default
// #define ACCEL_4G
// #define ACCEL_8G
// #define ACCEL_16G

// #define USE_BAROMETER
// //看你要不要用气压计，已知用了的话就会占用一个串口，并且要用掉3000us
// 要用的话记得板子上相应的要短接

//========================================================================================================================//

// REQUIRED LIBRARIES (included with download in main sketch folder)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h> // Teensy 4 的模拟 EEPROM 库
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <SD.h>
#include <SPI.h> //SPI communication
#include <Servo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <Wire.h> //I2c communication

// 使用第二I2C接口避免冲突（引脚24/25
// scl/sda,但是占了串口，后面如果不想要就注释掉）

#if defined USE_BAROMETER
#include <Adafruit_BMP280.h>
#define BMP280_I2C Wire2
Adafruit_BMP280 bmp(&BMP280_I2C);
// 滤波参数
#define FILTER_ALPHA 0.2
float filteredPressure = 1013.25; // 初始海平面气压(hPa)
float filteredAltitude = 0;
float temp;
float pressure;
#endif

// 42688p
//  --- ICM-42688-P 专用定义 ---
#define ICM_ADDR 0x69 // 你的 Wire 总线扫描地址
#define REG_ICM_BANK_SEL 0x76
#define REG_ICM_PWR_MGMT0 0x4E
#define REG_ICM_ACC_DATA 0x1F  // 加速度起始地址 (X_H) [cite: 1514]
#define REG_ICM_GYRO_DATA 0x25 // 陀螺仪起始地址 (X_H) [cite: 1514]

// --- BMI088 专用定义 (Wire1 总线) ---
#define BMI_ACC_ADDR 0x19     // SDO1 接高电平 [cite: 3328]
#define BMI_GYR_ADDR 0x69     // SDO2 接高电平 [cite: 3331]
#define REG_BMI_ACC_DATA 0x12 // 加速度起始 [cite: 2911]
#define REG_BMI_GYR_DATA 0x02 // 陀螺仪起始 [cite: 3070]
#define REG_BMI_ACC_PWR_CTRL 0x7D
#define REG_BMI_ACC_PWR_CONF 0x7C
static const float inv_acc_lsb = 1.0f / 5460.0f;
static const float inv_gyr_lsb = 1.0f / 16.384f;

// SETUP OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS                                                         \
  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

#define NUMFLAKES 10     // Number of snowflakes in the animation example
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#if defined USE_SBUS_RX
#include <SBUS.h> //sBus interface
#endif

#if defined USE_DSM_RX
#include <DSMRX.h>
#endif

#if defined USE_MPU6050_I2C && defined USE_MPU9250_SPI
#include <MPU6050.h>
#include <MPU9250.h>
MPU6050 mpu6050;
MPU9250 mpu9250(SPI, 10);
#elif defined USE_MPU9250_SPI
#include <MPU9250.h>
MPU9250 mpu9250(SPI, 10);
#elif defined USE_MPU6050_I2C
#include <MPU6050.h>
MPU6050 mpu6050;
#else
#error No MPU defined...
#endif

//========================================================================================================================//

// Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
#define GYRO_FS_SEL_250_6050 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500_6050 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000_6050 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000_6050 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2_6050 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4_6050 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8_6050 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16_6050 MPU6050_ACCEL_FS_16
#endif
#if defined USE_MPU9250_SPI
#define GYRO_FS_SEL_250_9250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500_9250 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000_9250 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000_9250 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2_9250 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4_9250 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8_9250 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16_9250 mpu9250.ACCEL_RANGE_16G
#endif

#if defined GYRO_250DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_250_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_250_9250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_500_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_500_9250
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_1000_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_1000_9250
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE_6050 GYRO_FS_SEL_2000_6050
#define GYRO_SCALE_9250 GYRO_FS_SEL_2000_9250
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_2_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_2_9250
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_4_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_4_9250
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_8_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_8_9250
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE_6050 ACCEL_FS_SEL_16_6050
#define ACCEL_SCALE_9250 ACCEL_FS_SEL_16_9250
#define ACCEL_SCALE_FACTOR 2048.0
#endif

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES //
//========================================================================================================================//
// PWM——OUTPUT

const int pwm_channel1_rev = -1;
const int pwm_channel2_rev = 1;
const int pwm_channel3_rev = -1;
const int pwm_channel4_rev = 1;
const int pwm_channel5_rev = -1;

// A机
const float pwm_channel1_trim = 190 - 30;  // 减少是向上 20是偏置
const float pwm_channel2_trim = -205 + 30; // 减少是向上

// 襟副翼微调
const float pwm_channel1B_trim = 215 - 30;  // 减少是向上
const float pwm_channel2B_trim = -70 + 30;  // 减少是向上
const float pwm_channel1C_trim = 160 - 30;  // 减少是向上
const float pwm_channel2C_trim = -202 + 30; // 减少是向上
const float pwm_channel1D_trim = 132 - 30;  // 减少是向上
const float pwm_channel2D_trim = -213 + 30; // 减少是向上
const float pwm_channel1E_trim = 150 - 30;  // 减少是向上
const float pwm_channel2E_trim = -220 + 30; // 减少是向上
const float pwm_channel1F_trim = 144 - 10;  // 减少是向上
const float pwm_channel2F_trim = -217 + 10; // 减少是向上
const float pwm_channel1G_trim = 180 - 10;  // 减少是向上
const float pwm_channel2G_trim = -200 + 10; // 减少是向上

// 升降舵微调，手动模式用
const float pwm_channel3B_trim = 42;
const float pwm_channel3C_trim = -50; // 对于子机也要修正
const float pwm_channel3D_trim = 150;
const float pwm_channel3E_trim = -50;
const float pwm_channel3F_trim = 9;
const float pwm_channel3G_trim = 180;

// 当前飞机

#if defined APLANE
const float pwm_channel3_trim =
    20; // 减少是向上  对于子机，也要修正,是用来增稳模式的 A是180 载机是10
#elif defined BPLANE
const float pwm_channel3_trim = pwm_channel3B_trim;
#elif defined CPLANE
const float pwm_channel3_trim = pwm_channel3C_trim;
#elif defined DPLANE
const float pwm_channel3_trim = pwm_channel3D_trim;
#elif defined EPLANE
const float pwm_channel3_trim = pwm_channel3E_trim;
#elif defined FPLANE
const float pwm_channel3_trim = pwm_channel3F_trim;
#elif defined GPLANE
const float pwm_channel3_trim = pwm_channel3G_trim;
#endif

const float pwm_channel4_trim = 0;
const float pwm_channel5_trim = 0;

// 载机
#if defined TESTBED
const float pwm_channel1_trim = 170 - 30;  // 减少是向上 20是偏置
const float pwm_channel2_trim = -240 + 30; // 减少是向上
const float pwm_channel3_trim = 10;        // 减少是向上
#endif

int outputpwm1, outputpwm2, outputpwm3, outputpwm4, outputpwm5;

// 特殊图案
const unsigned char logo[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xf0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x7f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0x81, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f,
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x03, 0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// angular_ACC
// 以9250为基准，6050的位置在9250的方位
float distx = 0.041;
float disty = 0.027;
double GyroY_9250_prev;

// 空速
MS4525read_Task airspeedSensor(&Wire1);

// Radio failsafe values for every channel in the event that bad reciever data
// is detected. Recommended defaults:
unsigned long channel_1_fs = 1500; // thro
unsigned long channel_2_fs = 1500; // ail
unsigned long channel_3_fs = 1000; // elev
unsigned long channel_4_fs = 1500; // rudd
unsigned long channel_5_fs = 1500; // gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; // aux1
unsigned long channel_7_fs = 2000; // aux2
unsigned long channel_8_fs = 2000; // aux3

float Aail1_PWM_TRIM = 0.0; // 舵面微调
float Aail2_PWM_TRIM = 0;   // 舵面微调
float Aele_PWM_TRIM = 0.0;  // 舵面微调
float Athro_PWM_TRIM = 0;   // 舵面微调
float Arudd_PWM_TRIM = 0;   // 舵面微调

#if defined expensive
float Trim_pitch_angle = 3.0;
#else
float Trim_pitch_angle = 8.0;
#endif

// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless
// you know what you are doing:

float B_madgwick = 0.04; // Madgwick filter parameter 后面定义过了default 0.04
float B_madgwick_adaptive = 0.04; // 动态权重
float base_B_madgwick = 0.04;     // 基准权重
float B_accel_6050 =
    0.6; // Accelerometer LP filter paramter, (MPU6050 default: 0.14.
         // 0.26at1000hz   0.6at333hz)。 1-exp((2000/当前hz)）*log(1-0.17))
float B_gyro_6050 = 0.47;  // Gyro LP filter paramter, (MPU6050 default: 0.1.
                           // 0.19at1000hz  0.47at333hz)
float B_accel_9250 = 0.74; // Accelerometer LP filter paramter, ( MPU9250
                           // default: 0.2   0.36at1000hz。 0.74at333hz)
float B_gyro_9250 = 0.67;  // Gyro LP filter paramter, ( MPU9250 default: 0.17。
                           // 0.31at1000hz。0.67at333hz)
float B_mag_9250 = 1.0; // Magnetometer LP filter parameter 1.0 means no fliter

// Magnetometer calibration parameters - if using MPU9250, uncomment
// calibrateMagnetometer() in void setup() to get these values, else just ignore
// these
float MagErrorX_9250 = 17.33;
float MagErrorY_9250 = 58.29;
float MagErrorZ_9250 = 29.16;
float MagScaleX_9250 = 0.98;
float MagScaleY_9250 = 1.05;
float MagScaleZ_9250 = 0.97;

// IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the
// void setup() to get these values, then comment out calculate_IMU_error()
// mpu6050

float AccErrorX_6050 = -0.02;
float AccErrorY_6050 = -0.01;
float AccErrorZ_6050 = 0.03;
float GyroErrorX_6050 = -13.83;
float GyroErrorY_6050 = -9.41;
float GyroErrorZ_6050 = 1.48;

// mpu9250
float AccErrorX_9250 = -0.11;
float AccErrorY_9250 = 0.01;
float AccErrorZ_9250 = 0.03;
float GyroErrorX_9250 = 5.41;
float GyroErrorY_9250 = -3.37;
float GyroErrorZ_9250 = -2.51;

// Control Variables
//  刹车
float ailBrake_PWM = 0.0, flap2eleratio = 0.0;
// A组控制变量
float Aail1_PWM, Aail2_PWM, Aele_PWM, Athro_PWM, Arudd_PWM, A_pitch_sp = 0.0;
// B组控制变量
float Bail1_PWM, Bail2_PWM, B_pitch_sp, Bthro_PWM, Brudd_PWM,
    B_ele_command_PWM_Manual, B_ele_command_PWM_FF = 0.0f;
// C组控制变量
float Cail1_PWM, Cail2_PWM, C_pitch_sp, Cthro_PWM, Crudd_PWM,
    C_ele_command_PWM_Manual, C_ele_command_PWM_FF = 0.0f;
// D组控制变量
float Dail1_PWM, Dail2_PWM, D_pitch_sp, Dthro_PWM, Drudd_PWM,
    D_ele_command_PWM_Manual, D_ele_command_PWM_FF = 0.0f;
// E组控制变量
float Eail1_PWM, Eail2_PWM, E_pitch_sp, Ethro_PWM, Erudd_PWM,
    E_ele_command_PWM_Manual, E_ele_command_PWM_FF = 0.0f;
// F组控制变量
float Fail1_PWM, Fail2_PWM, F_pitch_sp, Fthro_PWM, Frudd_PWM,
    F_ele_command_PWM_Manual, F_ele_command_PWM_FF = 0.0f;
// G组控制变量
float Gail1_PWM, Gail2_PWM, G_pitch_sp, Gthro_PWM, Grudd_PWM,
    G_ele_command_PWM_Manual, G_ele_command_PWM_FF = 0.0f;

// 向A的左发 F<-D<-B<-A
int servoCommandsleft[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float pitchAnglesleft[3] = {0, 0, 0};
int elePwmCommandsLeft[3] = {1500, 1500, 1500};
int eleFFPwmCommandsLeft[3] = {1500, 1500, 1500};
bool lightSignalsleft[3] = {0, 0, 1};
// 向A的右发 A->C->E->G
int servoCommandsright[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float pitchAnglesright[3] = {0, 0, 0};
int elePwmCommandsRight[3] = {1500, 1500, 1500};
int eleFFPwmCommandsRight[3] = {1500, 1500, 1500};
bool lightSignalsright[3] = {0, 0, 1};

// 从机往内发的。
float RelativeAngleAll[3] = {1.2, 2.4, 4}; // 相对转角 F->D->B->A
float GYROAll[3] = {5, 3, 1};              // 本体滚转角速度。F->D->B->A
float PHIALL[3] = {5, 3, 1};               // 本体滚转角度。F->D->B->A
float THETAALL[3] = {5, 3, 1};             // 本体俯仰角度。F->D->B->A
int ELEPWM[3] = {1500, 1500, 1500};        // 本体升降舵指令
float AIRSPEED[3] = {10, 10, 10};          // 本体空速
// 数传
float configuration_tele[6];
float attitude_tele[3];

float Local_pitch_des, Local_pitch_des_last = 8; // 期望当地俯仰角
int Local_ail1_PWM, Local_ail2_PWM, Local_thro_PWM, Local_rudd_PWM,
    Local_ele_PWM, Local_ele_ff_PWM;
int Local_rudd_PWM_last = 1520;
int Local_thro_PWM_last = 1111;
int Local_ail2_PWM_last = 1520;
int Local_ail1_PWM_last = 1520;
float relativeAngle_ready_prev;

// Controller parameters (take note of defaults before modifying!):

// Clp increase
int Ail_Clp;
float Clp_PID = 0.0;
float k_Clp = 0.05;

float i_limit =
    45.0; // Integrator saturation level, mostly for safety (default 25.0)
float i_valid = 0.0;   // 积分是否起作用
float maxRoll = 80.0;  // Max roll angle in degrees for angle mode (maximum ~70
                       // degrees), deg/sec for rate mode
float maxPitch = 60.0; // Max pitch angle in degrees for angle mode (maximum ~70
                       // degrees), deg/sec for rate mode
float maxYaw = 160.0;  // Max yaw rate in deg/sec

float kp_rotate = 0.4;
float Kp_roll_angle = 0.25; // Roll P-gain - angle mode
float Ki_roll_angle = 0.0;  // Roll I-gain - angle mode 0.08
float Kd_roll_angle =
    0.0; // Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 1.0; // Roll damping term for controlANGLE2(), lower is more
                         // damping (must be between 0 to 1)
float Kp_pitch_angle = 0.12; // Pitch P-gain - angle mode
float Ki_pitch_angle = 0.0;  // Pitch I-gain - angle mode 0.1
float Kd_pitch_angle =
    0.00; // Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 1.0; // Pitch damping term for controlANGLE2(), lower is
                          // more damping (must be between 0 to 1)
float Kp_Flap = 0.2;      // Flap
float B_loop_FLAP = 0.0;
float dw4, dw6, dw7, dw10, dw13, dw16, dw19, dw22;
float airspeed_A;

// INDI相关
float dp, dq, dr;                // 角加速度
double eleprev;                  // 升降舵位置估计值。
float indi_pitch_q_gain = 25.0f; // q误差转dq_des的比例
float indi_pitch_effectiveness =
    -113.65f; // 升降舵舵效: dq / delta_e, 单位 deg/s^2 per deg
float indi_pitch_pwm_to_deg_k =
    0.090909f; // delta_e = k * pwm + b, 1500PWM中位, 1720PWM下偏20deg
float indi_pitch_pwm_to_deg_b = -136.3636f;
float indi_pitch_servo_delay_s = 0.03f;       // 舵机纯延迟
float indi_pitch_servo_tau_s = 0.015f;        // 舵机一阶时间常数
float indi_pitch_deflection_min_deg = -20.0f; // 最大最小可用偏角 20deg
float indi_pitch_deflection_max_deg = 20.0f;
float indi_pitch_rate_limit_deg_s = 800.0f; // 舵偏角速度限幅
float indi_pitch_cmd_lpf_fc_hz = 6.0f;      // INDI输出舵偏指令低通截止频率 13.0
float indi_pitch_pwm_min = 1100.0f;
float indi_pitch_pwm_max = 1920.0f;
float indi_pitch_q_des_log = 0.0f;           // INDI目标俯仰角速度
float indi_pitch_q_filt_log = 0.0f;          // INDI独立滤波后的俯仰角速度
float indi_pitch_dq_des_log = 0.0f;          // INDI目标俯仰角加速度
float indi_pitch_dq_used_log = 0.0f;         // INDI使用的角加速度反馈
float indi_pitch_delta_e_cmd_deg_log = 0.0f; // INDI目标舵偏角
float indi_pitch_delta_e_est_deg_log = 0.0f; // INDI估计的实际舵偏角
float indi_pitch_pwm_cmd_log = 0.0f;         // INDI输出PWM
// 前次滤波后陀螺仪值（用于差分）
float gyroX_filt_prev = 0.0f;
float gyroY_filt_prev = 0.0f;
float gyroZ_filt_prev = 0.0f;
// 前次差分时间（微秒）
unsigned long prev_time_gyro_deriv = 0;

#if defined SINGLE
float Kp_roll_rate = 0.045; // Roll P-gain - rate mode 0.06单机
float Ki_roll_rate = 0.010; // Roll I-gain - rate mode 0.01单机
float Kd_roll_rate =
    0.0000; // Roll D-gain - rate mode (be careful when increasing too high,
            // motors will begin to overheat!)
float Kp_pitch_rate = 0.09; // Pitch P-gain - rate mode 0.12
float Ki_pitch_rate = 0.11; // Pitch I-gain - rate mode 0.25
float Kd_pitch_rate =
    0.0000; // Pitch D-gain - rate mode (be careful when increasing too high,
            // motors will begin to overheat!)
float Kff_roll_rate = 0.15;  // Roll FF-gain - rate mode 0.15单机
float Kff_pitch_rate = 0.20; // 0.2单机
float Kff_yaw_rate = 0.03;
float Kff_FLAP_RATE = 0.09;
float Kp_FLAP_RATE = 0.07;
float Ki_FLAP_RATE = 0.12;

#elif defined TEAM

float Kp_roll_rate = 0.15; // Roll P-gain - rate mode 0.06单机
float Ki_roll_rate = 0.1;  // Roll I-gain - rate mode 0.02单机
float Kd_roll_rate =
    0.0002; // Roll D-gain - rate mode (be careful when increasing too high,
            // motors will begin to overheat!)
float Kp_pitch_rate = 0.11; // Pitch P-gain - rate mode
float Ki_pitch_rate = 0.10; // Pitch I-gain - rate mode 0.25
float Kd_pitch_rate =
    0.000; // 0.0002 Pitch D-gain - rate mode (be careful when increasing too
           // high, motors will begin to overheat!)
float Kff_roll_rate = 0.12; // Roll FF-gain - rate mode 0.15单机
float Kff_pitch_rate = 0.20;
float Kff_yaw_rate = 0.03;
float Kff_FLAP_RATE = 0.1;
float Kp_FLAP_RATE = 0.2;  // 0.09
float Ki_FLAP_RATE = 0.20; // 0.15

#endif

float roll_eq;
float AccNorm;

float Kp_yaw = 0.2;     // Yaw P-gain
float Ki_yaw = 0.05;    // Yaw I-gain
float Kd_yaw = 0.00000; // Yaw D-gain (be careful when increasing too high,
                        // motors will begin to overheat!)

// SD log
char filename_sd[20];
File dataFile;
unsigned int fileNumber = 0;
unsigned long sd_counter = 0;
unsigned int logfreq = 50; // 记录频率Hz 注意要是2000的因数。
unsigned long lastLogTime = 0;
unsigned long fileCycle = 0;
String dataString = "";
unsigned long lastdispTime = 0;
unsigned int displayfreq = 10; // 显示屏帧率。
bool isdisplay = 1;
unsigned long lastsendTime = 0;
unsigned long lastsendTimeQua = 0;
unsigned long lasttelemetryTime = 0;
unsigned long lasttransTime = 0;  // 左边
unsigned long lasttransTime3 = 0; // 右边

// 应变
unsigned long last_read_Strain_Time = 0;
int32_t Strain_value1 = 0;
int32_t Strain_value2 = 0;
int32_t Strain_value3 = 0;
int32_t Strain_value4 = 0;
int32_t Strain_value5 = 0;

// 频率
unsigned int transfreq = 100; // 主机往外发串口发送频率Hz 注意要是2000的因数。
unsigned int Freqsendback = 200;    // 子机回发频率
unsigned int Freqtelemetry = 5;     // 数传频率
unsigned int read_Strain_freq = 10; // 读取应变数据频率

//========================================================================================================================//
//                                                     DECLARE PINS //
//========================================================================================================================//

// NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the
// MPU6050 IMU for default setup Radio: Note: If using SBUS, connect to pin 21
// (RX5), if using DSM, connect to pin 15 (RX3)
const int ch1Pin = 15; // throttle
const int ch2Pin = 16; // ail
const int ch3Pin = 17; // ele
const int ch4Pin = 20; // rudd
const int ch5Pin = 21; // gear (throttle cut)
const int ch6Pin = 22; // aux1 (free aux channel)
const int PPM_Pin = 23;
// OneShot125 ESC pin outputs:
const int m1Pin = 37;
const int m2Pin = 37;
const int m3Pin = 38;
const int m4Pin = 39;
const int m5Pin = 40;
const int m6Pin = 41;
// PWM servo or ESC outputs:
const int servo1Pin = 2; // 左副翼
const int servo2Pin = 3; // 右副翼
const int servo3Pin = 4; // 升降
const int servo4Pin = 5; // 油门
const int servo5Pin = 6; // 方向
const int servo6Pin = 9;
const int servo7Pin = 9;
Servo servo1; // Create servo objects to control a servo or ESC with PWM
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

//========================================================================================================================//

// DECLARE GLOBAL VARIABLES

// General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

// Radio communication:
int channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm,
    channel_6_pwm, channel_7_pwm, channel_8_pwm;
int channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev,
    channel_4_pwm_prev;

#if defined USE_SBUS_RX
SBUS sbus(Serial2);
uint16_t sbusChannels[16];
bool sbusFailSafe;
bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
DSM1024 DSM;
#endif

// IMU:
float AccX_6050, AccY_6050, AccZ_6050;
float AccX_9250, AccY_9250, AccZ_9250;
float AccX_prev_6050, AccY_prev_6050, AccZ_prev_6050;
float AccX_prev_9250, AccY_prev_9250, AccZ_prev_9250;
float GyroX_6050, GyroY_6050, GyroZ_6050;
float GyroX_9250, GyroY_9250, GyroZ_9250;
float GyroX_prev_6050, GyroY_prev_6050, GyroZ_prev_6050;
float GyroX_prev_9250, GyroY_prev_9250, GyroZ_prev_9250;
float Gyro_X_EXT, Gyro_Y_EXT, Gyro_Z_EXT;
float Acc_X_EXT, Acc_Y_EXT, Acc_Z_EXT;

float MagX_9250, MagY_9250, MagZ_9250;
float MagX_prev_9250, MagY_prev_9250, MagZ_prev_9250;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float roll_IMU_EXT, pitch_IMU_EXT, yaw_IMU_EXT;

// 滚转角控制指令滤波
float roll_PID_lpf = 0.0f;
const float roll_pid_lpf_fc = 7.0f; // 7hz指令滤波
const float TWO_PI_F = 6.28318530718f;

float q0 = 1.0f; // Initialize quaternion for madgwick filter 假设直立
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float q0B = 1.0f; // Initialize quaternion for madgwick filter 假设直立
float q1B = 0.0f;
float q2B = 0.0f;
float q3B = 0.0f;

float rollAB_rad_Qua, pitchAB_rad_Qua, yawAB_rad_Qua;

// Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des, rotate_speed_des, pitch_des_local;
float thro_des_RAW, roll_des_RAW, pitch_des_RAW, yaw_des_RAW;
float roll_passthru, pitch_passthru, yaw_passthru;
float pitch_des_local_last;
float pitch_des_local_rate;
float pitch_des_local_rate_lpf_fc = 5.0f;

// Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll,
    integral_roll_il, integral_roll_ol, integral_roll_prev,
    integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch,
    integral_pitch_il, integral_pitch_ol, integral_pitch_prev,
    integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch,
    pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev,
    derivative_yaw, yaw_PID = 0;
float rotate_error;
// 滚转方向的前馈
bool roll_pid_dot_initialized = false;
float roll_PID_prev = 0.0f;
float roll_PID_dot = 0.0f;
float roll_PID_dot_lpf = 0.0f;     // 如果你后面想给前馈用，建议保留一个滤波后的
float roll_pid_dot_lpf_fc = 10.0f; // 先给个截止频率参数，后面可调

float error_Phiab_RATE, error_Phiab, Phiab_des = 0, Phiab_Mea,
                                     integral_Phiab_ol, integral_Phiab_prev_ol,
                                     Phiab_des_ol, Phiab_des_nf;
float Phiab_des_prev, integral_Phiab_RATE_il, integral_Phiab_RATE_prev_il,
    Phiab_PID;
float error_Phiac, Phiac_des = 0, Phiac_Mea, integral_Phiac_ol,
                   integral_Phiac_prev_ol, Phiac_des_ol, Phiac_des_prev,
                   error_Phiac_RATE, integral_Phiac_RATE_il,
                   integral_Phiac_RATE_prev_il, Phiac_PID, Phiac_des_nf;
float error_Phibd, Phibd_des = 0, Phibd_Mea, integral_Phibd_ol,
                   integral_Phibd_prev_ol, Phibd_des_ol, Phibd_des_prev,
                   error_Phibd_RATE, integral_Phibd_RATE_il,
                   integral_Phibd_RATE_prev_il, Phibd_PID, Phibd_des_nf;
float error_Phice, Phice_des = 0, Phice_Mea, integral_Phice_ol,
                   integral_Phice_prev_ol, Phice_des_ol, Phice_des_prev,
                   error_Phice_RATE, integral_Phice_RATE_il,
                   integral_Phice_RATE_prev_il, Phice_PID, Phice_des_nf;
float error_Phidf, Phidf_des = 0, Phidf_Mea, integral_Phidf_ol,
                   integral_Phidf_prev_ol, Phidf_des_ol, Phidf_des_prev,
                   error_Phidf_RATE, integral_Phidf_RATE_il,
                   integral_Phidf_RATE_prev_il, Phidf_PID, Phidf_des_nf;
float error_Phieg, Phieg_des = 0, Phieg_Mea, integral_Phieg_ol,
                   integral_Phieg_prev_ol, Phieg_des_ol, Phieg_des_prev,
                   error_Phieg_RATE, integral_Phieg_RATE_il,
                   integral_Phieg_RATE_prev_il, Phieg_PID, Phieg_des_nf;

// TEAM
float phiab, phiac, phibd, phice, phidf, phieg; // 直接传入的。FDBACEG
float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw,
    phiG_raw; // 直接传入的单机本体角度
float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw, thetaG_raw;
int Bele_PWM, Cele_PWM, Dele_PWM, Eele_PWM, Fele_PWM,
    Gele_PWM; // 传入的当地的舵面
float Pab, Pac, Pbd, Pdf, Pce, Peg;
float GYRO_X_B, GYRO_X_C, GYRO_X_D, GYRO_X_E, GYRO_X_F,
    GYRO_X_G; // 每个单体的滚转角速度

// Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled,
    m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM,
    m5_command_PWM, m6_command_PWM;
float Aail1_scaled, Aail2_scaled, Aele_scaled, Athro_scaled, Arudd_scaled,
    s6_command_scaled, s7_command_scaled;
float Bail1_scaled, Bail2_scaled, Bele_scaled, Bthro_scaled, Brudd_scaled;
float Cail1_scaled, Cail2_scaled, Cele_scaled, Cthro_scaled, Crudd_scaled;
float Dail1_scaled, Dail2_scaled, Dele_scaled, Dthro_scaled, Drudd_scaled;
float Eail1_scaled, Eail2_scaled, Eele_scaled, Ethro_scaled, Erudd_scaled;
float Fail1_scaled, Fail2_scaled, Fele_scaled, Fthro_scaled, Frudd_scaled;
float Gail1_scaled, Gail2_scaled, Gele_scaled, Gthro_scaled, Grudd_scaled;

int s6_command_PWM, s7_command_PWM;

// Flight status
bool armedFly = false;

//========================================================================================================================//
//                                                      VOID SETUP //
//========================================================================================================================//

void setup() {
  Serial.begin(500000);  // USB seria再试试921600 不行就换回去
  Serial8.begin(115200); // 数传
  Serial6.begin(921600);
  Serial7.begin(115200); // 传感器
  Serial5.begin(921600);
  Serial3.begin(921600);
  Serial1.begin(115200); // 传感器
  delay(20);
  pinMode(32, OUTPUT); // led
  pinMode(33, OUTPUT); // led
  pinMode(36, OUTPUT); // 设置该引脚为输出模式
  pinMode(36, INPUT_PULLUP);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  displaythumbsup();

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Card failed, or not present");
    displaySD("MISS");

  } else {
    Serial.println("card initialized.");
    // displaySD("HERE");
    // open the file.
  }
  // 查找最大文件编号
  findMaxFileNumber();

  // 创建新文件
  sprintf(filename_sd, "datalog%03d.txt", fileNumber);
  dataFile = SD.open(filename_sd, FILE_WRITE);

  if (!dataFile) {
    Serial.println("Error opening datalog.txt");
  }

  dataFile.println(String(
      "TimeStamp(us),ROLL_IMU(deg),ROLL_Eq(deg),PITCH_IMU(deg),YAW_IMU(deg),"
      "ROLL_des(deg),PITCH_des_local(deg),YAW_des(deg),CH1_PWM,CH2_PWM,CH3_PWM,"
      "CH4_PWM,CH5_PWM,CH6_PWM,CH7_PWM,CH8_PWM,Aail1_PWM,Aail2_PWM,Aele_PWM,"
      "Athro_PWM,Arudd_PWM,Bail1_PWM,Bail2_PWM,Bthro_PWM,Brudd_PWM,Cail1_PWM,"
      "Cail2_PWM,Cthro_PWM,Crudd_PWM,Dail1_PWM,Dail2_PWM,Dthro_PWM,Drudd_PWM,"
      "Eail1_PWM,Eail2_PWM,Ethro_PWM,Erudd_PWM,Fail1_PWM,Fail2_PWM,Fthro_PWM,"
      "Frudd_PWM,Gail1_PWM,Gail2_PWM,Gthro_PWM,Grudd_PWM,Pab(deg),Pac(deg),Pbd("
      "deg),Pce(deg),Pdf(deg),Peg(deg),relativeAngle_ready(deg),Phiab_des(deg),"
      "phiac(deg),phibd(deg),phice(deg),phidf(deg),phieg(deg),Apitchsp,"
      "Bpitchsp,Cpitchsp,Dpitchsp,Epitchsp,Fpitchsp,Gpitchsp,Bpitch_raw,Cpitch_"
      "raw,Dpitch_raw,Epitch_raw,Fpitch_raw,Gpitch_raw,Bele_pwm,Cele_pwm,Dele_"
      "pwm,Eele_pwm,Fele_pwm,Gele_pwm,rollAB_rad_Qua,roll_IMU_EXT,pitch_IMU_"
      "EXT,yaw_IMU_EXT,invAccX_6050,AccY_6050,AccZ_6050,Gyro_X_6050,Gyro_Y_"
      "6050,Gyro_Z_6050,Gyro_X_EXT,Gyro_Y_EXT,Gyro_Z_EXT,phiab_PID,phiac_PID,"
      "phibd_PID,phice_PID,phidf_PID,phieg_PID,roll_PID,pitch_PID,airspeed_A,"
      "Strain_value1,Strain_value2,Strain_value3,Strain_value4,Strain_value5,"
      "AOA,AOS,TAS,dp,dq,dr,INDI_q_des,INDI_q_filt,INDI_dq_des,INDI_dq_used,"
      "INDI_delta_e_cmd_deg,INDI_delta_e_est_deg,INDI_pwm_cmd"));

  dataFile.flush();
  delay(10);
  Serial.print("新建日志文件：");
  Serial.println(filename_sd);
  // displaySD(fileNumber);
  displayfilenum();
  initBAROMETER();

  EEPROM.get(eepromAddress1, relativeAngle_offset);

  int address = 0;
  byte *pData = (byte *)&calAccGyroData;

  for (int i = 0; i < sizeof(CalibrationAccGyroData); i++) {
    pData[i] = EEPROM.read(address++);
  }

#if defined expensive
  digitalWrite(5, LOW);
  delay(100);
#endif

// 识别自己是谁
#if defined APLANE
  displayID("A");
#elif defined BPLANE
  displayID("B");
#elif defined CPLANE
  displayID("C");
#elif defined DPLANE
  displayID("D");
#elif defined EPLANE
  displayID("E");
#elif defined FPLANE
  displayID("F");
#elif defined GPLANE
  displayID("G");
#endif

  // Initialize all pins
  // pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify
  servo1.attach(servo1Pin, 900, 2100); // Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);

  delay(5);

  // Initialize radio communication
  radioSetup();

  // Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

// Initialize IMU communication
#if defined INTIMU
  IMUinit();
#endif

  delay(5);

  // Get IMU error to zero accelerometer and gyro readings, assuming vehicle is
  // level when powered up calculate_IMU_error(); //Calibration parameters
  // printed to serial monitor. Paste these in the user specified variables
  // section, then comment this out forever.

  // Arm servo channels
  servo1.write(90); // Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(90); // Set these to 90 for servos if you do not want them to
                    // briefly max out on startup
  servo3.write(90); // Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(90);
  servo6.write(0);
  servo7.write(0);

  delay(5);

  // calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by
  // setting throttle stick to max, powering on, and lowering throttle to zero
  // after the beeps Code will not proceed past here if this function is
  // uncommented!

  // Indicate entering main loop with 3 quick blinks
  // setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  // If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may
  // need to repeat for new locations) calibrateMagnetometer(); //Generates
  // magentometer error and scale factors to be pasted in user-specified
  // variables section

  // 控制分配事先计算
  initializeInertiaMatrices(Ia, Ib, Ic, Id, Ie); // 惯量矩阵
  initializeH();                                 // 舵效矩阵
  airspeedSensor.calib();
// 加快第一次姿态收敛。
#if defined INTIMU
  // getIMUdata();
  // getICM42688data(); // <-- 插入这一行
  getBMI088data();
  eulerToQuaternion(); // 在公式里，AccX是带负号的，但是在这，去掉了，是因为后面的madwick里，输入的AccX是负的。
#endif

  currentMode = MANUAL_MODE;

  // 初始化滤波器：采样率 500Hz，截止频率 40Hz
  gyroFiltX.set_cutoff_frequency(500, 40);
  gyroFiltY.set_cutoff_frequency(500, 40);
  gyroFiltZ.set_cutoff_frequency(500, 40);

  // INDI
  angularAccFiltX.set_cutoff_frequency(500, 100);
  angularAccFiltY.set_cutoff_frequency(500, 20);
  angularAccFiltZ.set_cutoff_frequency(500, 100);
  gyroFiltYIndi.set_cutoff_frequency(500, 40);
  gyroFiltXIndi.set_cutoff_frequency(500, 100);
  gyroFiltZIndi.set_cutoff_frequency(500, 100);
  gyroDerivFiltX.set_cutoff_frequency(500, 100);
  gyroDerivFiltY.set_cutoff_frequency(500, 40);
  gyroDerivFiltZ.set_cutoff_frequency(500, 100);

  // Serial.println("ss");

  // calculate_IMU_error();
}

//========================================================================================================================//
//                                                       MAIN LOOP //
//========================================================================================================================//

void loop() {
  // Keep track of what time it is and how much time has elapsed since the last
  // loop
  prev_time = current_time;                    // 上一个循环开始的时间
  current_time = micros();                     // 当前循环开始的时间
  dt = (current_time - prev_time) / 1000000.0; // 上一个循环耗时多久。

  // loopBlink(); //Indicate we are in main loop with short blink every 1.5
  // seconds

  // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT
  // ONE: printRadioData();     //Prints radio pwm values (expected: 1000 to
  // 2000) printDesiredState();  //Prints desired vehicle state commanded in
  // either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to
  // 1 for throttle) printGyroData();      //Prints filtered gyro data direct
  // from IMU (expected: ~ -250 to 250, 0 at rest) printAccelData(); //Prints
  // filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0
  // when level, z 1 when level) printMagData();       //Prints filtered
  // magnetometer data direct from IMU (expected: ~ -300 to 300)
  // printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from
  // Madgwick filter (expected: degrees, 0 when level)
  //  printPIDoutput();     //Prints computed stabilized PID variables from
  //  controller and desired setpoint (expected: ~ -1 to 1)
  // printMotorCommands(); //Prints the values being written to the motors
  // (expected: 120 to 250) printServoCommands(); //Prints the values being
  // written to the servos (expected: 0 to 180)
  //  printConfigurationData();
  // printLoopRate();      //Prints the time between loops in microseconds
  // (expected: microseconds between loop iterations)
  //  printQuaternion();

  // getbarodata();//气压计，用处不大

  // 在这里要获取所有的单机信息

  // Get arming status
  armedStatus(); // Check if the throttle cut is off and throttle is low.
// #if defined INTIMU
// Get vehicle state
#if defined INTIMU
  // getIMUdata();  //Pulls raw gyro, accelerometer, and magnetometer data from
  // IMU and LP filters to remove noise
  // 默认用9250的数据，后续可以有融合算法加持。 getICM42688data(); // <--
  // 插入这一行
  getBMI088data();
  Madgwick(dt); // Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates
                // (degrees) //为什么这里有负号？看看论文？跟传感器读数方式有关
// 注意本来这里的轴系定义和飞行力学课本不同，但是已经在欧拉角解算和控制pqr中改过
#endif
  getAngularACC();
// Compute desired state
#if defined EXTIMU
  getIMUdata_EXT();
#endif
  increase_Clp();
  getairspeed();
  getDesState(); // Convert raw commands to normalized values based on saturated
                 // control limits and flight modes 。if A
                 // plane，从遥控器读取。if other，从串口读取。
  // getRotateSensor();//读取转角传感器
  // 每个飞机都要做的，根据标号不同决定当前数值是phiac phibd···还是phiab
  // getRotateSensor1();    //读取转角传感器
  // 每个飞机都要做的，根据标号不同决定当前数值是phiac phibd···还是phiab

  ProcessButtonState(); // 读取按键是否按下 决定要不要清零以及要不要驱动显示屏

  // PID Controller - SELECT ONE:
  // controlANGLE(); //Stabilize on angle setpoint
  // controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate
  // controller must be tuned well first! controlRATE(); //Stabilize on rate
  // setpoint
  displayAttitude(); // 这玩意很浪费时间会把频率拖到80多 串口发送也会受影响
  Strain_read_all();
  getairdata();

// 信号传输
#if defined APLANE // 主机
  // 主机接收两边，但是从机接收一边
  getGYROxANGLEright(); // 得到三个p，3个转角 计算出pac pce peg
  getGYROxANGLEleft();  // 得到三个p，2个转角 计算出pab pbd pdf
  // telemetry(); //数传。 传输等效姿态角，相对转角，相对扭转角 10hz
  // getQuaternion();       //得到人家的四元数。

#elif defined BPLANE || defined DPLANE || defined FPLANE // 是左边从机
  getGYROxANGLEleft(); // 得到三个p，2个转角 计算出pab pbd pdf
#elif defined CPLANE || defined EPLANE || defined GPLANE // 是右边从机
  getGYROxANGLEright(); // 得到三个p，2个转角 计算出pac pce peg
#endif

  if (currentMode == STABILIZE_MODE) // 增稳
  {
    i_valid = 1.0;
    int_is_valid = true;
    force_manual = false;
    controlANGLE2(); // Stabilize on angle setpoint using cascaded method. Rate
                     // controller must be tuned well first!
    digitalWrite(33, HIGH); // 灯灭
    digitalWrite(32, HIGH); // 灯灭
    controlFlapMotion();
  } else if (currentMode == STABLIZE_MODE_NO_I) // 增稳
  {
    int_is_valid = false;
    force_manual = false;
    i_valid = 0.0;
    controlANGLE2();        // Stabilize on angle no I
    digitalWrite(33, LOW);  // 灯亮
    digitalWrite(32, HIGH); // 灯灭
    controlFlapMotion();
  } else if (currentMode == MANUAL_MODE) {
    force_manual = true;
    digitalWrite(33, HIGH); // 灯灭
    digitalWrite(32, LOW);  // 灯亮
    Phiab_PID = 0.0;
    Phiac_PID = 0.0;
    Phibd_PID = 0.0;
    Phice_PID = 0.0;
    Phidf_PID = 0.0;
    Phieg_PID = 0.0;
  }

  /*
  //要不要构型控制
  if (channel_7_pwm>1600)
  {
    Phiab_PID=0.0;
    Phiac_PID=0.0;
    digitalWrite(33, LOW);  //灯亮
    digitalWrite(32, LOW);   //灯亮
  }
  */

  // Actuator mixing and scaling to PWM values
  getpinvBplusmini();
  // printFullMatrix(Bplusmini_pinv);
  // Serial.println("sss");
  controlMixer();  // Mixes PID outputs to scaled actuator commands -- custom
                   // mixing assignments done here
  scaleCommands(); // Scales motor commands to 125 to 250 range (oneshot125
                   // protocol) and servo PWM commands to 0 to 180 (for servo
                   // library)
  // 计算并发布指令
  // A机

  // 贵的飞机做了舵面限幅
  int deviation1 = (Aail1_PWM - 1520) * 0.4;
  int deviation2 = (Aail2_PWM - 1520) * 0.4;
  int deviation3 = (Aele_PWM - 1520);
  int deviation4 = Athro_PWM - 1520;
  int deviation5 = Arudd_PWM - 1520;
  deviation1 =
      (Aail1_PWM + deviation3 / 6.0 - 1520) * 0.4; // 升降舵负升力襟翼补偿
  deviation2 = (Aail2_PWM + deviation3 / 6.0 - 1520) * 0.4;

  // 增加刹车
  if (channel_8_pwm > 1600) {
    ailBrake_PWM = 00.0;
  } else {
    ailBrake_PWM = 100.0;
  }

  Aail1_PWM =
      1520 + pwm_channel1_rev * deviation1 + pwm_channel1_trim - ailBrake_PWM;
  Aail2_PWM =
      1520 + pwm_channel2_rev * deviation2 + pwm_channel2_trim + ailBrake_PWM;
  Aele_PWM = 1520 + pwm_channel3_rev * deviation3 + pwm_channel3_trim;
  Athro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Arudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  // B机

  deviation1 = (Bail1_PWM - 1520) * 0.4;
  deviation2 = (Bail2_PWM - 1520) * 0.4;
  deviation4 = Bthro_PWM - 1520;
  deviation5 = Brudd_PWM - 1520;

  Bail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Bail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Bthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Brudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  servoCommandsleft[0] = int(Bail1_PWM + pwm_channel1B_trim);
  servoCommandsleft[1] = int(Bail2_PWM + pwm_channel2B_trim);
  servoCommandsleft[2] = int(Bthro_PWM);
  servoCommandsleft[3] = int(Brudd_PWM);
  pitchAnglesleft[0] = B_pitch_sp;
  B_ele_command_PWM_Manual =
      (B_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  elePwmCommandsLeft[0] = int(B_ele_command_PWM_Manual + pwm_channel3B_trim);
  eleFFPwmCommandsLeft[0] = int(B_ele_command_PWM_FF);

  // C机
  deviation1 = (Cail1_PWM - 1520) * 0.4;
  deviation2 = (Cail2_PWM - 1520) * 0.4;
  deviation4 = Cthro_PWM - 1520;
  deviation5 = Crudd_PWM - 1520;

  Cail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Cail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Cthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Crudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  servoCommandsright[0] = int(Cail1_PWM + pwm_channel1C_trim);
  servoCommandsright[1] = int(Cail2_PWM + pwm_channel2C_trim);
  servoCommandsright[2] = int(Cthro_PWM);
  servoCommandsright[3] = int(Crudd_PWM);
  pitchAnglesright[0] = int(C_pitch_sp);
  C_ele_command_PWM_Manual =
      (C_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  elePwmCommandsRight[0] = int(C_ele_command_PWM_Manual + pwm_channel3C_trim);
  eleFFPwmCommandsRight[0] = int(C_ele_command_PWM_FF);
  // Serial.println(elePwmCommandsRight[0]);
  // D机
  deviation1 = (Dail1_PWM - 1520) * 0.4;
  deviation2 = (Dail2_PWM - 1520) * 0.4;
  deviation4 = Dthro_PWM - 1520;
  deviation5 = Drudd_PWM - 1520;

  Dail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Dail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Dthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Drudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  servoCommandsleft[4] = int(Dail1_PWM + pwm_channel1D_trim);
  servoCommandsleft[5] = int(Dail2_PWM + pwm_channel2D_trim);
  servoCommandsleft[6] = int(Dthro_PWM);
  servoCommandsleft[7] = int(Drudd_PWM);
  pitchAnglesleft[1] = D_pitch_sp;
  D_ele_command_PWM_Manual =
      (D_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  elePwmCommandsLeft[1] = int(D_ele_command_PWM_Manual + pwm_channel3D_trim);
  eleFFPwmCommandsLeft[1] = int(D_ele_command_PWM_FF);

  // E机
  deviation1 = (Eail1_PWM - 1520) * 0.4;
  deviation2 = (Eail2_PWM - 1520) * 0.4;
  deviation4 = Ethro_PWM - 1520;
  deviation5 = Erudd_PWM - 1520;

  Eail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Eail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Ethro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Erudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  servoCommandsright[4] = int(Eail1_PWM + pwm_channel1E_trim);
  servoCommandsright[5] = int(Eail2_PWM + pwm_channel2E_trim);
  servoCommandsright[6] = int(Ethro_PWM);
  servoCommandsright[7] = int(Erudd_PWM);
  pitchAnglesright[1] = E_pitch_sp;
  E_ele_command_PWM_Manual =
      (E_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  elePwmCommandsRight[1] = int(E_ele_command_PWM_Manual + pwm_channel3E_trim);
  eleFFPwmCommandsRight[1] = int(E_ele_command_PWM_FF);

  // F机
  deviation1 = (Fail1_PWM - 1520) * 0.4;
  deviation2 = (Fail2_PWM - 1520) * 0.4;
  deviation4 = Fthro_PWM - 1520;
  deviation5 = Frudd_PWM - 1520;

  Fail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Fail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Fthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Frudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  servoCommandsleft[8] = int(Fail1_PWM + pwm_channel1F_trim);
  servoCommandsleft[9] = int(Fail2_PWM + pwm_channel2F_trim);
  servoCommandsleft[10] = int(Fthro_PWM);
  servoCommandsleft[11] = int(Frudd_PWM);
  pitchAnglesleft[2] = F_pitch_sp;
  F_ele_command_PWM_Manual =
      (F_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  elePwmCommandsLeft[2] = int(F_ele_command_PWM_Manual + pwm_channel3F_trim);
  eleFFPwmCommandsLeft[2] = int(F_ele_command_PWM_FF);

  // G机
  deviation1 = (Gail1_PWM - 1520) * 0.4;
  deviation2 = (Gail2_PWM - 1520) * 0.4;
  deviation4 = Gthro_PWM - 1520;
  deviation5 = Grudd_PWM - 1520;

  Gail1_PWM = 1520 + pwm_channel1_rev * deviation1 - ailBrake_PWM;
  Gail2_PWM = 1520 + pwm_channel2_rev * deviation2 + ailBrake_PWM;
  Gthro_PWM = 1520 + pwm_channel4_rev * deviation4 + pwm_channel4_trim;
  Grudd_PWM = 1520 + pwm_channel5_rev * deviation5 + pwm_channel5_trim;

  servoCommandsright[8] = int(Gail1_PWM + pwm_channel1G_trim);
  servoCommandsright[9] = int(Gail2_PWM + pwm_channel2G_trim);
  servoCommandsright[10] = int(Gthro_PWM);
  servoCommandsright[11] = int(Grudd_PWM);
  pitchAnglesright[2] = G_pitch_sp;
  G_ele_command_PWM_Manual =
      (G_ele_command_PWM_Manual - 1520) * pwm_channel3_rev + 1520;
  elePwmCommandsRight[2] = int(G_ele_command_PWM_Manual + pwm_channel3G_trim);
  eleFFPwmCommandsRight[2] = int(G_ele_command_PWM_FF);

#if defined APLANE // 是主机
  // servo1.write(90+40*sin(1*micros()/100000.0));
  // Serial.println(90+40*sin(micros()/100000.0));
  // servo1.writeMicroseconds(1520+400*sin(5*micros()/100000.0)); //左副翼

#if defined TESTINDI
  if (currentMode == STABILIZE_MODE) {
    Aele_PWM = PITCH_INDI_control();
  }
#endif

  servo1.writeMicroseconds(Aail1_PWM + Ail_Clp); // 左副翼
  servo2.writeMicroseconds(Aail2_PWM + Ail_Clp); // 右副翼
  servo3.writeMicroseconds(Aele_PWM);            // 升降
  servo4.writeMicroseconds(Athro_PWM);           // 油门
  servo5.writeMicroseconds(Arudd_PWM);           // 方向

  sendAllDataright(servoCommandsright, pitchAnglesright, elePwmCommandsRight,
                   eleFFPwmCommandsRight, lightSignalsright, int_is_valid);
  sendAllDataleft(servoCommandsleft, pitchAnglesleft, elePwmCommandsLeft,
                  eleFFPwmCommandsLeft, lightSignalsleft, int_is_valid);

#else // 是从机

#if defined TESTINDI
  if (currentMode == STABILIZE_MODE) {
    Aele_PWM = PITCH_INDI_control();
  }
#endif

  servo1.writeMicroseconds(Local_ail1_PWM + (Aele_PWM - 1520) / 15.0 +
                           Ail_Clp); // 左副翼
  servo2.writeMicroseconds(Local_ail2_PWM - (Aele_PWM - 1520) / 15.0 +
                           Ail_Clp); // 右副翼

  if (currentMode == MANUAL_MODE) {
    servo3.writeMicroseconds(
        Local_ele_PWM); // 强制手动时用主机下发的当地升降舵PWM
  } else {
    servo3.writeMicroseconds(Aele_PWM +
                             Local_ele_ff_PWM); // 非强制手动时保持原逻辑
  }
  servo4.writeMicroseconds(Local_thro_PWM); // 油门

  // 检测从机油门。
  Serial.println(Local_thro_PWM);

  servo5.writeMicroseconds(Local_rudd_PWM); // 方向
  sendGYROxANGLE();                         // 发送所有的滚转角速度
  // setQuaternion(); //发送自己的四元数姿态

#if defined BPLANE || defined DPLANE || defined FPLANE   // 是左边从机
  servoCommandsleft[0] = 0;
  servoCommandsleft[1] = 0;
  servoCommandsleft[2] = 0;
  servoCommandsleft[3] = 0;
  elePwmCommandsLeft[0] = 0;
  eleFFPwmCommandsLeft[0] = 0;

  servoCommandsleft[4] = recvData.servo[4];
  servoCommandsleft[5] = recvData.servo[5];
  servoCommandsleft[6] = recvData.servo[6];
  servoCommandsleft[7] = recvData.servo[7];
  pitchAnglesleft[1] = recvData.pitch[1];
  elePwmCommandsLeft[1] = recvData.ele_pwm[1];
  eleFFPwmCommandsLeft[1] = recvData.ele_ff_pwm[1];
  servoCommandsleft[8] = recvData.servo[8];
  servoCommandsleft[9] = recvData.servo[9];
  servoCommandsleft[10] = recvData.servo[10];
  servoCommandsleft[11] = recvData.servo[11];
  pitchAnglesleft[2] = recvData.pitch[2];
  elePwmCommandsLeft[2] = recvData.ele_pwm[2];
  eleFFPwmCommandsLeft[2] = recvData.ele_ff_pwm[2];
  // lightSignalsleft={0,0,0};
  sendAllDataleft(servoCommandsleft, pitchAnglesleft, elePwmCommandsLeft,
                  eleFFPwmCommandsLeft, lightSignalsleft, int_is_valid);
#elif defined CPLANE || defined EPLANE || defined GPLANE // 是右边从机
  servoCommandsright[0] = 0;
  servoCommandsright[1] = 0;
  servoCommandsright[2] = 0;
  servoCommandsright[3] = 0;
  elePwmCommandsRight[0] = 0;
  eleFFPwmCommandsRight[0] = 0;
  // printReceivedData();
  servoCommandsright[4] = recvData.servo[4];
  servoCommandsright[5] = recvData.servo[5];
  servoCommandsright[6] = recvData.servo[6];
  servoCommandsright[7] = recvData.servo[7];
  pitchAnglesright[1] = recvData.pitch[1];
  elePwmCommandsRight[1] = recvData.ele_pwm[1];
  eleFFPwmCommandsRight[1] = recvData.ele_ff_pwm[1];
  servoCommandsright[8] = recvData.servo[8];
  servoCommandsright[9] = recvData.servo[9];
  servoCommandsright[10] = recvData.servo[10];
  servoCommandsright[11] = recvData.servo[11];
  pitchAnglesright[2] = recvData.pitch[2];
  elePwmCommandsRight[2] = recvData.ele_pwm[2];
  eleFFPwmCommandsRight[2] = recvData.ele_ff_pwm[2];
  // lightSignalsright={0,0,0};
  sendAllDataright(servoCommandsright, pitchAnglesright, elePwmCommandsRight,
                   eleFFPwmCommandsRight, lightSignalsright, int_is_valid);

#endif

#endif

  loggerTEAM(); // 这玩意不很浪费时间
  // loggerSINGLE(); //这玩意不很浪费时间
  // Get vehicle commands for next loop iteration
  getCommands(); // Pulls current available radio commands

  failSafe(); // Prevent failures in event of bad receiver connection, defaults
              // to failsafe values assigned in setup

  // Regulate loop rate
  loopRate(500); // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz
                 // by default
}

//========================================================================================================================//
//                                                      FUNCTIONS //
//========================================================================================================================//

void controlMixer() {
  // DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs
  // based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and
   * appropriately mixes them for the desired vehicle configuration. For example
   * on a quadcopter, the left two motors should have +roll_PID while the right
   * two motors should have -roll_PID. Front two should have -pitch_PID and the
   * back two should have +pitch_PID etc... every motor has normalized (0 to 1)
   * thro_des command for throttle control. Can also apply direct unstabilized
   * commands from the transmitter with roll_passthru, pitch_passthru, and
   * yaw_passthu. mX_command_scaled and sX_command scaled variables are used in
   * scaleCommands() in preparation to be sent to the motor ESCs and servos.
   *
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command
   * passthrough channel_6_pwm - free auxillary channel, can be used to toggle
   * things with an 'if' statement
   */

  // 0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for
  // conventional PWM, 1.0 is max throttle

  if (currentMode == MANUAL_MODE) {
    roll_PID = roll_des_RAW / 2.0;
    pitch_PID = pitch_des_RAW / 2.0;
    yaw_PID = yaw_des_RAW / 2.0;
  }

  // 混控在这里

#if defined SINGLE
  // 自己飞
  Aail1_scaled = roll_PID;
  Aail2_scaled = -roll_PID;
  Aele_scaled = pitch_PID;
  Athro_scaled = thro_des;
  Arudd_scaled = yaw_PID;

#elif defined TEAM

/*
//2机一起飞 BA
//策略1

float coeroll=1.0;
if (channel_6_pwm>1600)
{
Bail1_scaled = 0.81*roll_PID+0.5*Phiab_PID;
Bail2_scaled = 0.41*roll_PID-0.5*Phiab_PID;
Bthro_scaled = thro_des-0.8*yaw_PID;
Brudd_scaled = yaw_PID;
B_pitch_sp=pitch_des_local+coeroll*0.07*roll_des;//2机、3机


Aail1_scaled = -0.41*roll_PID-0.5*Phiab_PID;
Aail2_scaled = -0.81*roll_PID+0.5*Phiab_PID;
Aele_scaled = pitch_PID;
Athro_scaled = thro_des+0.8*yaw_PID;
Arudd_scaled = yaw_PID;
central_pitch=-0.07*coeroll*roll_des;
}
else if(channel_6_pwm<1600&&channel_6_pwm>1400)
{

//策略2
Bail1_scaled = 0.5*0.81*roll_PID+0.5*Phiab_PID;
Bail2_scaled = 0.5*0.41*roll_PID-0.5*Phiab_PID;
Bthro_scaled = thro_des-0.8*yaw_PID;
Brudd_scaled = yaw_PID;
B_pitch_sp=pitch_des_local+12.0*roll_PID;//2机、3机


Aail1_scaled = 0.5*-0.41*roll_PID-0.5*Phiab_PID;
Aail2_scaled = 0.5*-0.81*roll_PID+0.5*Phiab_PID;
Aele_scaled = pitch_PID;
Athro_scaled = thro_des+0.8*yaw_PID;
Arudd_scaled = yaw_PID;
central_pitch=-12.0*coeroll*roll_PID;

}
else
{
//策略3
Bail1_scaled = 0.1*0.81*roll_PID+0.5*Phiab_PID;
Bail2_scaled = 0.1*0.41*roll_PID-0.5*Phiab_PID;
Bthro_scaled = thro_des+keeppositive(-1.2*yaw_PID)+keeppositive(0.4*roll_PID);
Brudd_scaled = yaw_PID;
B_pitch_sp=pitch_des_local+45.0*coeroll*roll_PID;//2机、3机


Aail1_scaled = 0.1*-0.41*roll_PID-0.5*Phiab_PID;
Aail2_scaled = 0.1*-0.81*roll_PID+0.5*Phiab_PID;
Aele_scaled = pitch_PID;
Athro_scaled = thro_des+keeppositive(1.2*yaw_PID)+keeppositive(-0.4*roll_PID);
Arudd_scaled = yaw_PID;
central_pitch=-45.0*coeroll*roll_PID;


}

*/
// Serial.println(Aail1_scaled);

// 3机一起飞 BAC
#if defined THREEPLANE

  float coeab, coeac, coeroll;
  if (abs(error_Phiab) > 20) {
    coeab = abs(error_Phiab) / 20.0;
  } else {
    coeab = 0.8;
  }
  if (abs(error_Phiac) > 20) {
    coeac = abs(error_Phiac) / 20.0;
  } else {
    coeac = 0.8;
  }

  coeac = 1.0;

  // 策略1
  if (channel_6_pwm > 1600) {
    Bail1_scaled =
        1.016 * roll_PID + coeab * 1 * Phiab_PID - coeac * 0.68 * Phiac_PID;
    Bail2_scaled = 0.593 * roll_PID - coeab * 0.3392 * Phiab_PID -
                   coeac * 0.12 * Phiac_PID;
    Bthro_scaled = thro_des - 0.8 * yaw_PID;
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local; // 2机、3机

    Aail1_scaled =
        0.1 * roll_PID - coeab * 1.186 * Phiab_PID + coeac * 0.71 * Phiac_PID;
    Aail2_scaled =
        -0.1 * roll_PID - coeab * 0.71 * Phiab_PID + coeac * 1.16 * Phiac_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des;
    Arudd_scaled = yaw_PID;

    Cail1_scaled =
        -0.593 * roll_PID + coeab * 0.12 * Phiab_PID + coeac * 0.34 * Phiac_PID;
    Cail2_scaled =
        -1.016 * roll_PID + coeab * 0.68 * Phiab_PID - coeac * 1 * Phiac_PID;
    Cthro_scaled = thro_des + 0.8 * yaw_PID;
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local; // 2机、3机
  }
  // 策略2

  else if (channel_6_pwm < 1600 && channel_6_pwm > 1400) {
    coeroll = 0.2;
    Bail1_scaled = 1.016 * coeroll * roll_PID + coeab * 0.864 * Phiab_PID -
                   coeac * 0.62 * Phiac_PID;
    Bail2_scaled = 0.593 * coeroll * roll_PID - coeab * 0.27 * Phiab_PID -
                   coeac * 0.08 * Phiac_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.2 * yaw_PID) + keeppositive(0.4 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 20.0 * roll_PID_lpf; // 2机、3机
    B_ele_command_PWM_FF = 0.0;

    Aail1_scaled =
        0.1 * roll_PID - coeab * 1.0 * Phiab_PID + coeac * 0.65 * Phiac_PID;
    Aail2_scaled =
        -0.1 * roll_PID - coeab * 0.65 * Phiab_PID + coeac * 1.0 * Phiac_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des;
    Arudd_scaled = yaw_PID;

    Cail1_scaled = -0.593 * coeroll * roll_PID + coeab * 0.08 * Phiab_PID +
                   coeac * 0.27 * Phiac_PID;
    Cail2_scaled = -1.016 * coeroll * roll_PID + coeab * 0.62 * Phiab_PID -
                   coeac * 0.864 * Phiac_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.2 * yaw_PID) + keeppositive(-0.4 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 20.0 * roll_PID_lpf; // 2机、3机
    C_ele_command_PWM_FF = 0.0;

  } else {
    coeroll = 0.2;

    Bail1_scaled = 1.016 * coeroll * roll_PID + coeab * 0.864 * Phiab_PID -
                   coeac * 0.62 * Phiac_PID;
    Bail2_scaled = 0.593 * coeroll * roll_PID - coeab * 0.27 * Phiab_PID -
                   coeac * 0.08 * Phiac_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.2 * yaw_PID) + keeppositive(0.4 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 30.0 * roll_PID_lpf +
                 6.0 * (Bail1_scaled + Bail2_scaled); // 2机、3机
    B_ele_command_PWM_FF =
        -10.0 * 20.0 * roll_PID_lpf - 10.0 * roll_PID_dot_lpf;

    Aail1_scaled =
        0.1 * roll_PID - coeab * 1.0 * Phiab_PID + coeac * 0.65 * Phiac_PID;
    Aail2_scaled =
        -0.1 * roll_PID - coeab * 0.65 * Phiab_PID + coeac * 1.0 * Phiac_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des;
    Arudd_scaled = yaw_PID;

    Cail1_scaled = -0.593 * coeroll * roll_PID + coeab * 0.08 * Phiab_PID +
                   coeac * 0.27 * Phiac_PID;
    Cail2_scaled = -1.016 * coeroll * roll_PID + coeab * 0.62 * Phiab_PID -
                   coeac * 0.864 * Phiac_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.2 * yaw_PID) + keeppositive(-0.4 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp =
        pitch_des_local - 30.0 * roll_PID_lpf +
        6.0 * (Cail1_scaled + Cail2_scaled); // 2机、3机 增加了相对滚转到升降舵
    C_ele_command_PWM_FF = 10.0 * 20.0 * roll_PID_lpf + 10.0 * roll_PID_dot_lpf;
  }

#elif defined FOURPLANE

  // 4机一起飞 DBAC
  float coeab, coeac, coebd,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍

  coeroll = 1.5;

  // Serial.println(coeroll);
  if (abs(error_Phiab) > 20) {
    coeab = 1.2 * abs(error_Phiab) / 20.0;
  } else {
    coeab = 1.2;
  }

  if (abs(error_Phiac) > 20) {
    coeac = 1.0 * abs(error_Phiac) / 20.0;
  } else {
    coeac = 1.0;
  }

  if (abs(error_Phibd) > 20) {
    coebd = 1.0 * abs(error_Phibd) / 20.0;
  } else {
    coebd = 1.0;
  }

  // 策略1
  if (channel_6_pwm > 1600) {
    Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
    Dail2_scaled = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                   0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
    Dthro_scaled = thro_des - 1.2 * yaw_PID;
    Drudd_scaled = yaw_PID;
    D_pitch_sp = pitch_des_local + 0.13 * coeroll * roll_des;

    Bail1_scaled = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
    Bail2_scaled = 0.13 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
    Bthro_scaled = thro_des - 0.6 * yaw_PID;
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 0.07 * coeroll * roll_des;

    Aail1_scaled = -0.13 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
    Aail2_scaled = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled = thro_des + 0.6 * yaw_PID;
    ;
    Arudd_scaled = yaw_PID;
    central_pitch = -0.07 * coeroll * roll_des;

    Cail1_scaled = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                   0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
    Cail2_scaled = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
    Cthro_scaled = thro_des + 1.2 * yaw_PID;
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 0.13 * coeroll * roll_des;

  }

  // 策略2

  else if (channel_6_pwm < 1600 && channel_6_pwm > 1400) {
    coeroll = 0.3;
    Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
    Dail2_scaled = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                   0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
    Dthro_scaled =
        thro_des + keeppositive(-1.5 * yaw_PID) + keeppositive(0.5 * roll_PID);
    Drudd_scaled = yaw_PID;
    D_pitch_sp = pitch_des_local + 70.0 * roll_PID;

    Bail1_scaled = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
    Bail2_scaled = 0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.0 * yaw_PID) + keeppositive(0.3 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 30.0 * roll_PID;

    Aail1_scaled = -0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
    Aail2_scaled = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled =
        thro_des + keeppositive(1.0 * yaw_PID) + keeppositive(-0.3 * roll_PID);
    Arudd_scaled = yaw_PID;
    central_pitch = -30.0 * roll_PID;

    Cail1_scaled = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                   0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
    Cail2_scaled = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.5 * yaw_PID) + keeppositive(-0.5 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 70.0 * roll_PID;

  } else {

    coeroll = 0.2;
    Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   0.53 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID;
    Dail2_scaled = 0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID -
                   0.26 * coeac * Phiac_PID - 0.1 * coebd * Phibd_PID;
    Dthro_scaled =
        thro_des + keeppositive(-1.5 * yaw_PID) + keeppositive(0.5 * roll_PID);
    Drudd_scaled = yaw_PID;
    D_pitch_sp = pitch_des_local + 70.0 * roll_PID + 35.0 * Phiab_PID -
                 20.0 * Phiac_PID + 20.0 * Phibd_PID;

    Bail1_scaled = 0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   0.19 * coeac * Phiac_PID - 1.0 * coebd * Phibd_PID;
    Bail2_scaled = 0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.39 * coeac * Phiac_PID - 0.736 * coebd * Phibd_PID;
    Bthro_scaled =
        thro_des + keeppositive(-1.0 * yaw_PID) + keeppositive(0.3 * roll_PID);
    Brudd_scaled = yaw_PID;
    B_pitch_sp = pitch_des_local + 30.0 * roll_PID - 40.0 * Phiab_PID +
                 15.0 * Phiac_PID - 45.0 * Phibd_PID;

    Aail1_scaled = -0.1 * coeroll * roll_PID - 1.0 * coeab * Phiab_PID +
                   0.736 * coeac * Phiac_PID - 0.39 * coebd * Phibd_PID;
    Aail2_scaled = -0.28 * coeroll * roll_PID - 0.667 * coeab * Phiab_PID +
                   1.0 * coeac * Phiac_PID - 0.19 * coebd * Phibd_PID;
    Aele_scaled = pitch_PID;
    Athro_scaled =
        thro_des + keeppositive(1.0 * yaw_PID) + keeppositive(-0.3 * roll_PID);
    Arudd_scaled = yaw_PID;
    central_pitch = -30.0 * roll_PID - 40.0 * Phiab_PID + 45.0 * Phiac_PID -
                    15.0 * Phibd_PID;

    Cail1_scaled = -0.74 * coeroll * roll_PID + 0.325 * coeab * Phiab_PID +
                   0.1 * coeac * Phiac_PID + 0.26 * coebd * Phibd_PID;
    Cail2_scaled = -1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                   1.0 * coeac * Phiac_PID + 0.53 * coebd * Phibd_PID;
    Cthro_scaled =
        thro_des + keeppositive(1.5 * yaw_PID) + keeppositive(-0.5 * roll_PID);
    Crudd_scaled = yaw_PID;
    C_pitch_sp = pitch_des_local - 70.0 * roll_PID + 35.0 * Phiab_PID -
                 20.0 * Phiac_PID + 20.0 * Phibd_PID;
  }

#elif defined FIVEPLANE

  // 5机一起飞DBACE
  float coeab, coeac, coebd, coece,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍
  if (abs(error_Phiab) > 20) {
    coeab = abs(error_Phiab) / 20.0;
  } else {
    coeab = 1.0;
  }

  if (abs(error_Phiac) > 20) {
    coeac = abs(error_Phiac) / 20.0;
  } else {
    coeac = 1.0;
  }

  if (abs(error_Phibd) > 20) {
    coebd = abs(error_Phibd) / 20.0;
  } else {
    coebd = 0.9;
  }

  if (abs(error_Phice) > 20) {
    coece = abs(error_Phice) / 20.0;
  } else {
    coece = 0.9;
  }

  // 策略1
  coeroll = 1.2;
  Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                 0.72 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID -
                 0.38 * coece * Phice_PID;
  Dail2_scaled = 0.75 * coeroll * roll_PID + 0.44 * coeab * Phiab_PID -
                 0.42 * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID -
                 0.23 * coece * Phice_PID;
  Dthro_scaled = thro_des - 1.0 * yaw_PID;
  Drudd_scaled = yaw_PID;
  D_pitch_sp = pitch_des_local + 0.3 * coeroll * roll_des;

  Bail1_scaled = 0.417 * coeroll * roll_PID - 0.36 * coeab * Phiab_PID +
                 0.05 * coeac * Phiac_PID - 0.7 * coebd * Phibd_PID -
                 0.01 * coece * Phice_PID;
  Bail2_scaled = 0.33 * coeroll * roll_PID - 0.76 * coeab * Phiab_PID +
                 0.31 * coeac * Phiac_PID - 0.6 * coebd * Phibd_PID +
                 0.08 * coece * Phice_PID;
  Bthro_scaled = thro_des - 0.8 * yaw_PID;
  Brudd_scaled = yaw_PID;
  B_pitch_sp = pitch_des_local + 0.15 * coeroll * roll_des;

  Aail1_scaled = 0.15 * coeroll * roll_PID - 0.88 * coeab * Phiab_PID +
                 0.66 * coeac * Phiac_PID - 0.42 * coebd * Phibd_PID +
                 0.26 * coece * Phice_PID;
  Aail2_scaled = -0.15 * coeroll * roll_PID - 0.66 * coeab * Phiab_PID +
                 0.88 * coeac * Phiac_PID - 0.27 * coebd * Phibd_PID +
                 0.38 * coece * Phice_PID;
  Aele_scaled = pitch_PID;
  Athro_scaled = thro_des;
  Arudd_scaled = yaw_PID;

  Cail1_scaled = -0.33 * coeroll * roll_PID - 0.31 * coeab * Phiab_PID +
                 0.76 * coeac * Phiac_PID - 0.08 * coebd * Phibd_PID +
                 0.6 * coece * Phice_PID;
  Cail2_scaled = -0.417 * coeroll * roll_PID - 0.05 * coeab * Phiab_PID +
                 0.36 * coeac * Phiac_PID - 0.01 * coebd * Phibd_PID +
                 0.7 * coece * Phice_PID;
  Cthro_scaled = thro_des + 0.8 * yaw_PID;
  Crudd_scaled = yaw_PID;
  C_pitch_sp = pitch_des_local - 0.15 * coeroll * roll_des;

  Eail1_scaled = -0.75 * coeroll * roll_PID + 0.42 * coeab * Phiab_PID -
                 coeac * 0.44 * Phiac_PID + 0.23 * coebd * Phibd_PID -
                 0.06 * coece * Phice_PID;
  Eail2_scaled = -1.0 * coeroll * roll_PID + 0.72 * coeab * Phiab_PID -
                 1.0 * coeac * Phiac_PID + 0.38 * coebd * Phibd_PID -
                 1.0 * coece * Phice_PID;
  Ethro_scaled = thro_des + 1.0 * yaw_PID;
  Erudd_scaled = yaw_PID;
  E_pitch_sp = pitch_des_local - 0.3 * coeroll * roll_des;

  /*
   else if (channel_6_pwm < 1600 && channel_6_pwm > 1400) {
     //策略2
     coeroll = 0.5;
     Dail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID - 0.72 *
   coeac * Phiac_PID + 1.0 * coebd * Phibd_PID - 0.38 * coece * Phice_PID;
     Dail2_scaled = 0.75 * coeroll * roll_PID + 0.44 * coeab * Phiab_PID - 0.42
   * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID - 0.23 * coece * Phice_PID;
     Dthro_scaled = thro_des + keeppositive(-1.8 * yaw_PID) + keeppositive(0.7 *
   roll_PID); Drudd_scaled = yaw_PID; D_pitch_sp = pitch_des_local + 80.0 *
   roll_PID ;

     Bail1_scaled = 0.417 * coeroll * roll_PID - 0.36 * coeab * Phiab_PID + 0.05
   * coeac * Phiac_PID - 0.7 * coebd * Phibd_PID - 0.01 * coece * Phice_PID;
     Bail2_scaled = 0.33 * coeroll * roll_PID - 0.76 * coeab * Phiab_PID + 0.31
   * coeac * Phiac_PID - 0.6 * coebd * Phibd_PID + 0.08 * coece * Phice_PID;
     Bthro_scaled = thro_des + keeppositive(-1.0 * yaw_PID) + keeppositive(0.4 *
   roll_PID); Brudd_scaled = yaw_PID; B_pitch_sp = pitch_des_local + 30.0 *
   roll_PID ;

     Aail1_scaled = 0.15 * coeroll * roll_PID - 0.88 * coeab * Phiab_PID + 0.66
   * coeac * Phiac_PID - 0.42 * coebd * Phibd_PID + 0.27 * coece * Phice_PID;
     Aail2_scaled = -0.15 * coeroll * roll_PID - 0.66 * coeab * Phiab_PID + 0.88
   * coeac * Phiac_PID - 0.27 * coebd * Phibd_PID + 0.42 * coece * Phice_PID;
     Aele_scaled = pitch_PID;
     Athro_scaled = thro_des;
     Arudd_scaled = yaw_PID;

     Cail1_scaled = -0.33 * coeroll * roll_PID - 0.31 * coeab * Phiab_PID + 0.76
   * coeac * Phiac_PID - 0.08 * coebd * Phibd_PID + 0.6 * coece * Phice_PID;
     Cail2_scaled = -0.417 * coeroll * roll_PID - 0.05 * coeab * Phiab_PID +
   0.36 * coeac * Phiac_PID - 0.01 * coebd * Phibd_PID + 0.7 * coece *
   Phice_PID; Cthro_scaled = thro_des + keeppositive(1.0 * yaw_PID) +
   keeppositive(-0.4 * roll_PID); Crudd_scaled = yaw_PID; C_pitch_sp =
   pitch_des_local - 30.0 * roll_PID;

     Eail1_scaled = -0.75 * coeroll * roll_PID + 0.42 * coeab * Phiab_PID -
   coeac * 0.44 * Phiac_PID + 0.23 * coebd * Phibd_PID - 0.06 * coece *
   Phice_PID; Eail2_scaled = -1.0 * coeroll * roll_PID + 0.72 * coeab *
   Phiab_PID - 1.0 * coeac * Phiac_PID + 0.38 * coebd * Phibd_PID - 1.0 * coece
   * Phice_PID; Ethro_scaled = thro_des + keeppositive(1.8 * yaw_PID) +
   keeppositive(-0.7 * roll_PID); Erudd_scaled = yaw_PID; E_pitch_sp =
   pitch_des_local - 80.0 * roll_PID;
   }

   else {
     //Serial.println("ss");
     //策略3 实时控制分配
     //期望虚拟控制量
     dw4 = roll_PID * 40.0;
     dw6 = yaw_PID * 20.0;
     dw7 = Phiab_PID * 130.0;
     dw10 = Phiac_PID * 130.0;
     dw13 = Phibd_PID * 110.0;
     dw16 = Phice_PID * 110.0;


     dw_att << dw4,
       dw6,
       0.0,
       0.0,
       0.0,
       0.0;

     dw_config << dw7,
       dw10,
       dw13,
       dw16;

     de_config = Bplusminismall_pinv * dw_config;
     de_att = Bplusfullsmall_pinv * dw_att;
     //printFullMatrix(de_config);
     Dail1_scaled = de_att(0) + de_config(0);
     Dail2_scaled = de_att(1) + de_config(1);
     Dthro_scaled = thro_des + keeppositive(-1.4 * yaw_PID) + keeppositive(0.5 *
   roll_PID); Drudd_scaled = yaw_PID; D_pitch_sp = pitch_des_local + 80.0 *
   roll_PID - 1.0;

     Bail1_scaled = de_att(2) + de_config(2);
     Bail2_scaled = de_att(3) + de_config(3);
     Bthro_scaled = thro_des + keeppositive(-0.8 * yaw_PID) + keeppositive(0.3 *
   roll_PID); Brudd_scaled = yaw_PID; B_pitch_sp = pitch_des_local + 30.0 *
   roll_PID - 1.0;

     Aail1_scaled = de_att(4) + de_config(4);
     Aail2_scaled = de_att(5) + de_config(5);
     Aele_scaled = pitch_PID;
     Athro_scaled = thro_des;
     Arudd_scaled = yaw_PID;

     Cail1_scaled = de_att(6) + de_config(6);
     Cail2_scaled = de_att(7) + de_config(7);
     Cthro_scaled = thro_des + keeppositive(0.8 * yaw_PID) + keeppositive(-0.3 *
   roll_PID); Crudd_scaled = yaw_PID; C_pitch_sp = pitch_des_local - 30.0 *
   roll_PID;

     Eail1_scaled = de_att(8) + de_config(8);
     Eail2_scaled = de_att(9) + de_config(9);
     Ethro_scaled = thro_des + keeppositive(1.4 * yaw_PID) + keeppositive(-0.5 *
   roll_PID); Erudd_scaled = yaw_PID; E_pitch_sp = pitch_des_local - 80.0 *
   roll_PID;
   }
     */

#elif defined SEVENPLANE

  // 7机一起飞 FDBACEG
  float coeab, coeac, coebd, coece, coedf, coeeg,
      coeroll; // bc的系数和de的系数不同 de的系数是ab的0.66倍
  if (abs(error_Phiab) > 20) {
    coeab = abs(error_Phiab) / 20.0;
  } else {
    coeab = 1.0;
  }

  if (abs(error_Phiac) > 20) {
    coeac = abs(error_Phiac) / 20.0;
  } else {
    coeac = 1.0;
  }

  if (abs(error_Phibd) > 20) {
    coebd = abs(error_Phibd) / 20.0;
  } else {
    coebd = 0.9;
  }

  if (abs(error_Phice) > 20) {
    coece = abs(error_Phice) / 20.0;
  } else {
    coece = 0.9;
  }

  if (abs(error_Phidf) > 20) {
    coedf = abs(error_Phidf) / 20.0;
  } else {
    coedf = 0.6;
  }

  if (abs(error_Phieg) > 20) {
    coeeg = abs(error_Phieg) / 20.0;
  } else {
    coeeg = 0.6;
  }

  // 策略1
  coeroll = 1.8;

  Fail1_scaled = 1.0 * coeroll * roll_PID + 1.0 * coeab * Phiab_PID -
                 0.77 * coeac * Phiac_PID + 1.0 * coebd * Phibd_PID -
                 0.44 * coece * Phice_PID + 1.0 * coedf * Phidf_PID -
                 0.24 * coeeg * Phieg_PID;
  Fail2_scaled = 0.87 * coeroll * roll_PID + 0.69 * coeab * Phiab_PID -
                 0.56 * coeac * Phiac_PID + 0.54 * coebd * Phibd_PID -
                 0.33 * coece * Phice_PID + 0.19 * coedf * Phidf_PID -
                 0.18 * coeeg * Phieg_PID;
  Fthro_scaled = 0.85 * thro_des - 1.5 * yaw_PID;
  Frudd_scaled = yaw_PID;
  F_pitch_sp = pitch_des_local + 0.5 * coeroll * roll_des;

  Dail1_scaled = 0.72 * coeroll * roll_PID + 0.31 * coeab * Phiab_PID -
                 0.32 * coeac * Phiac_PID + 0.06 * coebd * Phibd_PID -
                 0.21 * coece * Phice_PID - 0.27 * coedf * Phidf_PID -
                 0.12 * coeeg * Phieg_PID;
  Dail2_scaled = 0.55 * coeroll * roll_PID - 0.12 * coeab * Phiab_PID -
                 0.06 * coeac * Phiac_PID - 0.44 * coebd * Phibd_PID -
                 0.07 * coece * Phice_PID - 0.41 * coedf * Phidf_PID -
                 0.05 * coeeg * Phieg_PID;
  Dthro_scaled = 0.92 * thro_des - 1.0 * yaw_PID;
  Drudd_scaled = yaw_PID;
  D_pitch_sp = pitch_des_local + 0.3 * coeroll * roll_des;

  Bail1_scaled = 0.4 * coeroll * roll_PID - 0.50 * coeab * Phiab_PID +
                 0.2 * coeac * Phiac_PID - 0.65 * coebd * Phibd_PID +
                 0.05 * coece * Phice_PID - 0.41 * coedf * Phidf_PID +
                 0.01 * coeeg * Phieg_PID;
  Bail2_scaled = 0.23 * coeroll * roll_PID - 0.83 * coeab * Phiab_PID +
                 0.47 * coeac * Phiac_PID - 0.57 * coebd * Phibd_PID +
                 0.17 * coece * Phice_PID - 0.27 * coedf * Phidf_PID +
                 0.08 * coeeg * Phieg_PID;
  Bthro_scaled = thro_des - 0.8 * yaw_PID;
  Brudd_scaled = yaw_PID;
  B_pitch_sp = pitch_des_local + 0.15 * coeroll * roll_des;

  Aail1_scaled = 0.1 * coeroll * roll_PID - 0.89 * coeab * Phiab_PID +
                 0.70 * coeac * Phiac_PID - 0.46 * coebd * Phibd_PID +
                 0.31 * coece * Phice_PID - 0.18 * coedf * Phidf_PID +
                 0.14 * coeeg * Phieg_PID;
  Aail2_scaled = -0.1 * coeroll * roll_PID - 0.70 * coeab * Phiab_PID +
                 0.89 * coeac * Phiac_PID - 0.31 * coebd * Phibd_PID +
                 0.46 * coece * Phice_PID - 0.14 * coedf * Phidf_PID +
                 0.18 * coeeg * Phieg_PID;
  Aele_scaled = pitch_PID;
  Athro_scaled = thro_des;
  Arudd_scaled = yaw_PID;

  Cail1_scaled = -0.23 * coeroll * roll_PID - 0.47 * coeab * Phiab_PID +
                 0.83 * coeac * Phiac_PID - 0.17 * coebd * Phibd_PID +
                 0.57 * coece * Phice_PID - 0.08 * coedf * Phidf_PID +
                 0.27 * coeeg * Phieg_PID;
  Cail2_scaled = -0.4 * coeroll * roll_PID - 0.2 * coeab * Phiab_PID +
                 0.50 * coeac * Phiac_PID - 0.05 * coebd * Phibd_PID +
                 0.65 * coece * Phice_PID - 0.01 * coedf * Phidf_PID +
                 0.41 * coeeg * Phieg_PID;
  Cthro_scaled = thro_des + 0.8 * yaw_PID;
  Crudd_scaled = yaw_PID;
  C_pitch_sp = pitch_des_local - 0.15 * coeroll * roll_des;

  Eail1_scaled = -0.55 * coeroll * roll_PID + 0.06 * coeab * Phiab_PID +
                 coeac * 0.12 * Phiac_PID + 0.07 * coebd * Phibd_PID +
                 0.44 * coece * Phice_PID + 0.05 * coedf * Phidf_PID +
                 0.41 * coeeg * Phieg_PID;
  Eail2_scaled = -0.72 * coeroll * roll_PID + 0.32 * coeab * Phiab_PID -
                 0.31 * coeac * Phiac_PID + 0.21 * coebd * Phibd_PID -
                 0.06 * coece * Phice_PID + 0.12 * coedf * Phidf_PID +
                 0.27 * coeeg * Phieg_PID;
  Ethro_scaled = 0.92 * thro_des + 1.0 * yaw_PID;
  Erudd_scaled = yaw_PID;
  E_pitch_sp = pitch_des_local - 0.3 * coeroll * roll_des;

  Gail1_scaled = -0.87 * coeroll * roll_PID + 0.56 * coeab * Phiab_PID -
                 coeac * 0.69 * Phiac_PID + 0.33 * coebd * Phibd_PID -
                 0.54 * coece * Phice_PID + 0.18 * coedf * Phidf_PID -
                 0.19 * coeeg * Phieg_PID;
  Gail2_scaled = -1.0 * coeroll * roll_PID + 0.77 * coeab * Phiab_PID -
                 1.0 * coeac * Phiac_PID + 0.44 * coebd * Phibd_PID -
                 1.0 * coece * Phice_PID + 0.24 * coedf * Phidf_PID -
                 1.0 * coeeg * Phieg_PID;
  Gthro_scaled = 0.85 * thro_des + 1.5 * yaw_PID;
  Grudd_scaled = yaw_PID;
  G_pitch_sp = pitch_des_local - 0.5 * coeroll * roll_des;

#endif

#endif

  s6_command_scaled = 0;
  s7_command_scaled = 0;
}

void armedStatus() {
  // DESCRIPTION: Check if the throttle cut is off and the throttle input is low
  // to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
    armedFly = true;
  }
}

void IMUinit() {
// DESCRIPTION: Initialize IMU 有一个用一个，有两个用两个
/*
 * Don't worry about how this works.
 */
#if defined USE_MPU6050_I2C
  Wire.begin();
  // Wire.setClock(1000000);  //Note this is 2.5 times the spec sheet 400 kHz
  // max...

  // mpu6050.initialize();
  //  --- 插入 ICM-42688 初始化 --- [cite: 1833]
  // Wire.setClock(1000000); // 42688p建议 400kHz 比较稳
  // Wire.beginTransmission(ICM_ADDR);
  // Wire.write(REG_ICM_PWR_MGMT0);
  // Wire.write(0x0F); // 开启加速度计和陀螺仪的低噪声模式
  // Wire.endTransmission();
  delay(10);

  // 1. 初始化引脚
  pinMode(CS_ACC, OUTPUT);
  pinMode(CS_GYR, OUTPUT);
  digitalWrite(CS_ACC, HIGH);
  digitalWrite(CS_GYR, HIGH);

  // 2. 启动 SPI
  SPI.begin();
  delay(100);

  // 3. 唤醒加速度计 (必需序列)
  writeRegSPI(CS_ACC, 0x7D, 0x04); // ACC_PWR_CTRL: Active
  delay(10);
  writeRegSPI(CS_ACC, 0x7C, 0x00); // ACC_PWR_CONF: Active
  delay(10);

  // 4. 配置陀螺仪
  writeRegSPI(CS_GYR, 0x0F, 0x00); // ±2000 dps
  writeRegSPI(CS_GYR, 0x10, 0x02); // ODR 1000Hz, BW 116Hz

  Serial.println("BMI088 SPI Initialized.");

  /*
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }
    */

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu6050.setFullScaleGyroRange(GYRO_SCALE_6050);
  mpu6050.setFullScaleAccelRange(ACCEL_SCALE_6050);
#endif

#if defined USE_MPU9250_SPI
  int status = mpu9250.begin();

  if (status < 0) {
    Serial.println("MPU9250 initialization unsuccessful");
    Serial.println("Check MPU9250 wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {
    }
  }

  // From the reset state all registers should be 0x00, so we should be at
  // max sample rate with digital low pass filter(s) off.  All we need to
  // do is set the desired fullscale ranges
  mpu9250.setGyroRange(GYRO_SCALE_9250);
  mpu9250.setAccelRange(ACCEL_SCALE_9250);
  mpu9250.setMagCalX(MagErrorX_9250, MagScaleX_9250);
  mpu9250.setMagCalY(MagErrorY_9250, MagScaleY_9250);
  mpu9250.setMagCalZ(MagErrorZ_9250, MagScaleZ_9250);
  mpu9250.setSrd(
      0); // sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
}

void getIMUdata() {
  // DESCRIPTION: Request full dataset from IMU and LP filter gyro,
  // accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY,
   * AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. These values are scaled
   * according to the IMU datasheet to put them into correct units of g's,
   * deg/sec, and uT. A simple first-order low-pass filter is used to get rid of
   * high frequency noise in these raw signals. Generally you want to cut off
   * everything past 80Hz, but if your loop rate is not fast enough, the low
   * pass filter will cause a lag in the readings. The filter parameters B_gyro
   * and B_accel are set to be good for a 2kHz loop rate. Finally, the constant
   * errors found in calculate_IMU_error() on startup are subtracted from the
   * accelerometer and gyro readings.
   */
  // 读数的单位是G
  int16_t AcX_6050, AcY_6050, AcZ_6050, GyX_6050, GyY_6050, GyZ_6050;
  int16_t AcX_9250, AcY_9250, AcZ_9250, GyX_9250, GyY_9250, GyZ_9250, MgX_9250,
      MgY_9250, MgZ_9250;

#if defined USE_MPU6050_I2C
  mpu6050.getMotion6(&AcX_6050, &AcY_6050, &AcZ_6050, &GyX_6050, &GyY_6050,
                     &GyZ_6050);
#endif
#if defined USE_MPU9250_SPI
  mpu9250.getMotion9(&AcX_9250, &AcY_9250, &AcZ_9250, &GyX_9250, &GyY_9250,
                     &GyZ_9250, &MgX_9250, &MgY_9250, &MgZ_9250);
#endif

  // Accelerometer
  AccX_6050 = AcX_6050 / ACCEL_SCALE_FACTOR; // G's
  AccY_6050 = AcY_6050 / ACCEL_SCALE_FACTOR;
  AccZ_6050 = AcZ_6050 / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX_6050 = AccX_6050 - calAccGyroData.AccErrorX_6050;
  AccY_6050 = AccY_6050 - calAccGyroData.AccErrorY_6050;
  AccZ_6050 = AccZ_6050 - calAccGyroData.AccErrorZ_6050;
  // LP filter accelerometer data
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050;
  AccY_prev_6050 = AccY_6050;
  AccZ_prev_6050 = AccZ_6050;

  AccX_9250 = AcX_9250 / ACCEL_SCALE_FACTOR; // G's
  AccY_9250 = AcY_9250 / ACCEL_SCALE_FACTOR;
  AccZ_9250 = AcZ_9250 / ACCEL_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  AccX_9250 = AccX_9250 - calAccGyroData.AccErrorX_9250;
  AccY_9250 = AccY_9250 - calAccGyroData.AccErrorY_9250;
  AccZ_9250 = AccZ_9250 - calAccGyroData.AccErrorZ_9250;
  // LP filter accelerometer data
  AccX_9250 = (1.0 - B_accel_9250) * AccX_prev_9250 + B_accel_9250 * AccX_9250;
  AccY_9250 = (1.0 - B_accel_9250) * AccY_prev_9250 + B_accel_9250 * AccY_9250;
  AccZ_9250 = (1.0 - B_accel_9250) * AccZ_prev_9250 + B_accel_9250 * AccZ_9250;
  AccX_prev_9250 = AccX_9250;
  AccY_prev_9250 = AccY_9250;
  AccZ_prev_9250 = AccZ_9250;

  // Gyro
  GyroX_6050 = GyX_6050 / GYRO_SCALE_FACTOR; // deg/sec
  GyroY_6050 = GyY_6050 / GYRO_SCALE_FACTOR;
  GyroZ_6050 = GyZ_6050 / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX_6050 = GyroX_6050 - calAccGyroData.GyroErrorX_6050;
  GyroY_6050 = GyroY_6050 - calAccGyroData.GyroErrorY_6050;
  GyroZ_6050 = GyroZ_6050 - calAccGyroData.GyroErrorZ_6050;
  // LP filter gyro data
  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050;
  GyroY_prev_6050 = GyroY_6050;
  GyroZ_prev_6050 = GyroZ_6050;

  GyroX_9250 = GyX_9250 / GYRO_SCALE_FACTOR; // deg/sec
  GyroY_9250 = GyY_9250 / GYRO_SCALE_FACTOR;
  GyroZ_9250 = GyZ_9250 / GYRO_SCALE_FACTOR;
  // Correct the outputs with the calculated error values
  GyroX_9250 = GyroX_9250 - calAccGyroData.GyroErrorX_9250;
  GyroY_9250 = GyroY_9250 - calAccGyroData.GyroErrorY_9250;
  GyroZ_9250 = GyroZ_9250 - calAccGyroData.GyroErrorZ_9250;
  // LP filter gyro data
  GyroX_9250 = (1.0 - B_gyro_9250) * GyroX_prev_9250 + B_gyro_9250 * GyroX_9250;
  GyroY_9250 = (1.0 - B_gyro_9250) * GyroY_prev_9250 + B_gyro_9250 * GyroY_9250;
  GyroZ_9250 = (1.0 - B_gyro_9250) * GyroZ_prev_9250 + B_gyro_9250 * GyroZ_9250;
  GyroX_prev_9250 = GyroX_9250;
  GyroY_prev_9250 = GyroY_9250;
  GyroZ_prev_9250 = GyroZ_9250;

  // Magnetometer
  MagX_9250 = MgX_9250 / 6.0; // uT
  MagY_9250 = MgY_9250 / 6.0;
  MagZ_9250 = MgZ_9250 / 6.0;
  // Correct the outputs with the calculated error values
  MagX_9250 = (MagX_9250 - MagErrorX_9250) * MagScaleX_9250;
  MagY_9250 = (MagY_9250 - MagErrorY_9250) * MagScaleY_9250;
  MagZ_9250 = (MagZ_9250 - MagErrorZ_9250) * MagScaleZ_9250;
  // LP filter magnetometer data
  MagX_9250 = (1.0 - B_mag_9250) * MagX_prev_9250 + B_mag_9250 * MagX_9250;
  MagY_9250 = (1.0 - B_mag_9250) * MagY_prev_9250 + B_mag_9250 * MagY_9250;
  MagZ_9250 = (1.0 - B_mag_9250) * MagZ_prev_9250 + B_mag_9250 * MagZ_9250;
  MagX_prev_9250 = MagX_9250;
  MagY_prev_9250 = MagY_9250;
  MagZ_prev_9250 = MagZ_9250;
}

void calculate_IMU_error() {
  // DESCRIPTION: 采集 12000 样本计算 BMI088 的偏置，并以 90 度旋转后的轴系存入
  // EEPROM
  double sAX = 0, sAY = 0, sAZ = 0, sGX = 0, sGY = 0, sGZ = 0;
  int c = 0;
  uint8_t buf[6];

  Serial.println(
      "BMI088 SPI Calibrating (Rotated 90)... Keep vehicle flat and still.");

  while (c < 12000) {
    // 1. SPI 读取原始加速度
    SPI.beginTransaction(bmiSettings);
    digitalWrite(CS_ACC, LOW);
    SPI.transfer(0x12 | 0x80); // 读取加速度起始地址
    SPI.transfer(0x00); // 重要：BMI088 加速度计 SPI 读取必需的 Dummy Byte
    for (int i = 0; i < 6; i++)
      buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_ACC, HIGH);
    SPI.endTransaction();

    int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

    // 2. SPI 读取原始陀螺仪
    SPI.beginTransaction(bmiSettings);
    digitalWrite(CS_GYR, LOW);
    SPI.transfer(0x02 | 0x80); // 读取陀螺仪起始地址
    for (int i = 0; i < 6; i++)
      buf[i] = SPI.transfer(0x00);
    digitalWrite(CS_GYR, HIGH);
    SPI.endTransaction();

    int16_t rgx = (int16_t)(buf[0] | (buf[1] << 8));
    int16_t rgy = (int16_t)(buf[2] | (buf[3] << 8));
    int16_t rgz = (int16_t)(buf[4] | (buf[5] << 8));

    // 3. 映射与累加 (保持 90 度旋转逻辑: 新X=旧Y, 新Y=-旧X)
    sAX += ry / 5460.0;
    sAY += -rx / 5460.0;
    sAZ += rz / 5460.0;

    sGX += rgy / 16.384;
    sGY += -rgx / 16.384;
    sGZ += rgz / 16.384;

    if (c % 1000 == 0)
      Serial.print(".");
    c++;
    delayMicroseconds(100);
  }

  // 4. 计算均值并补偿 Z 轴重力 (假设平放时 Z 应为 1g)
  calAccGyroData.AccErrorX_6050 = (float)(sAX / c);
  calAccGyroData.AccErrorY_6050 = (float)(sAY / c);
  calAccGyroData.AccErrorZ_6050 = (float)((sAZ / c) - 1.0f);

  calAccGyroData.GyroErrorX_6050 = (float)(sGX / c);
  calAccGyroData.GyroErrorY_6050 = (float)(sGY / c);
  calAccGyroData.GyroErrorZ_6050 = (float)(sGZ / c);

  // 5. 存入 EEPROM
  int addr = 0;
  byte *p = (byte *)&calAccGyroData;
  for (unsigned int i = 0; i < sizeof(CalibrationAccGyroData); i++) {
    EEPROM.write(addr++, p[i]);
  }

  Serial.println(
      "\nBMI088 SPI Rotation Calibration Complete & Saved to EEPROM.");
}

void calibrateAttitude() {
  // DESCRIPTION: Used to warm up the main loop to allow the madwick filter to
  // converge before commands can be sent to the actuators Assuming vehicle is
  // powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is
   * what causes startup to take a few seconds to boot.
   */
  // Warm up IMU and madgwick filter in simulated main loop about 5s
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;
    current_time = micros();
    dt = (current_time - prev_time) / 1000000.0;
    // getIMUdata();
    // getICM42688data(); // <-- 插入这一行
    getBMI088data();
    Madgwick(dt);

    loopRate(2000); // do not exceed 2000Hz
  }
}
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

void getDesState() { // 调整了通道顺序
  // DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and
   * yaw_des. These are computed by using the raw RC pwm commands and scaling
   * them to be within our limits defined in setup. thro_des stays within 0 to 1
   * range. roll_des and pitch_des are scaled to be within max roll/pitch amount
   * in either degrees (angle mode) or degrees/sec (rate mode). yaw_des is
   * scaled to be within max yaw in degrees/sec. Also creates roll_passthru,
   * pitch_passthru, and yaw_passthru variables, to be used in commanding
   * motors/servos with direct unstabilized commands in controlMixer().
   */
  float GyroZ;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
#endif

#if defined APLANE // 是主机
  {
    thro_des_RAW = (channel_3_pwm - 1100.0) / 1000.0; // Between 0 and 1
    roll_des_RAW = (channel_1_pwm - 1520.0) / 500.0;  // Between -1 and 1
    pitch_des_RAW = (channel_2_pwm - 1520.0) / 500.0; // Between -1 and 1
    yaw_des_RAW = (channel_4_pwm - 1520.0) / 500.0;   // Between -1 and 1

    /*
    if (channel_6_pwm>1600) //水平构型
    {
      Phiab_des_nf = 0.0;
      Phiac_des_nf = 0.0;
      Phibd_des_nf = 0.0;
      Phice_des_nf = 0.0;
      Phidf_des_nf = 0.0;
      Phieg_des_nf = 0.0;
     }
     else if (channel_6_pwm<1600&&channel_6_pwm>1400) //下反
     {
        Phiab_des_nf = -5.0;
        Phiac_des_nf = 5.0;
        Phibd_des_nf = -5.0;
        Phice_des_nf = 5.0;
        Phidf_des_nf = -5.0;
        Phieg_des_nf = 5.0;
     }
     else //Z字
     {
        Phiab_des_nf = 10.0;
        Phiac_des_nf = 10.0;
        Phibd_des_nf = 10.0;
        Phice_des_nf = 10.0;
        Phidf_des_nf = 10.0;
        Phieg_des_nf = 10.0;
     }

     Phiab_des=0.02*Phiab_des_nf+(1-0.02)*Phiab_des;
     Phiac_des=0.02*Phiac_des_nf+(1-0.02)*Phiac_des;
     Phibd_des=0.02*Phibd_des_nf+(1-0.02)*Phibd_des;
     Phice_des=0.02*Phice_des_nf+(1-0.02)*Phice_des;
     Phidf_des=0.02*Phidf_des_nf+(1-0.02)*Phidf_des;
     Phieg_des=0.02*Phieg_des_nf+(1-0.02)*Phieg_des;
    */

    roll_passthru = roll_des_RAW / 2.0;   // Between -0.5 and 0.5
    pitch_passthru = pitch_des_RAW / 2.0; // Between -0.5 and 0.5
    yaw_passthru = yaw_des_RAW / 2.0;     // Between -0.5 and 0.5

    // Constrain within normalized bounds
    thro_des = constrain(thro_des_RAW, 0.0, 1.0); // Between 0 and 1
    roll_des = constrain(roll_des_RAW, -1.0, 1.0) *
               maxRoll; // Between -maxRoll and +maxRoll
    pitch_des_local = constrain(pitch_des_RAW, -1.0, 1.0) *
                      maxPitch; // Between -maxPitch and +maxPitch
    yaw_des = constrain(yaw_des_RAW, -1.0, 1.0) *
              maxYaw; // Between -maxYaw and +maxYaw
    roll_passthru = constrain(roll_passthru, -0.5, 0.5);
    pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
    yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);

    // Phiab_des=constrain(Phiab_des, -1.0, 1.0)*15.0; //Between -maxRoll and
    // +maxRoll Phiac_des=constrain(Phiac_des, -1.0, 1.0)*-15.0; //Between
    // -maxRoll and +maxRoll

    pitch_des_local += Trim_pitch_angle;

#if defined TEAM

    // pitch_des_local=pitch_des-0.1*roll_des;//2机
    pitch_des_local = pitch_des_local; // 3机
#endif

    // 1600以上是手动  1400-1600是速率 1400以下是增稳
    if (channel_5_pwm > 1600) {
      currentMode = MANUAL_MODE;
    } else {
      currentMode =
          (channel_5_pwm < 1400) ? STABILIZE_MODE : STABLIZE_MODE_NO_I;
    }

    if (currentMode != lastMode) {
      ModeChange = 1;
    } else {
      ModeChange = 0;
    }
    lastMode = currentMode;
  }
#else // 是从机
  {
    receiveCommandData();
    if (int_is_valid) {
      currentMode = STABILIZE_MODE;
    } else {
      currentMode = STABLIZE_MODE_NO_I;
    }
    if (force_manual) {
      currentMode = MANUAL_MODE;
    }
    if (currentMode != lastMode) {
      ModeChange = 1;
    } else {
      ModeChange = 0;
    }
    lastMode = currentMode;

    thro_des = 0;
    roll_des = 0;
    pitch_des_local = Local_pitch_des;
    // Serial.println(pitch_des_local);
    yaw_des = 0;
  }
#endif
}

void controlANGLE() {
  // DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states
   * roll_des, pitch_des, and yaw_des computed in getDesState(). Error is simply
   * the desired state minus the actual state (ex. roll_des - roll_IMU). Two
   * safety features are implimented here regarding the I terms. The I terms are
   * saturated within specified limits on startup to prevent excessive buildup.
   * This can be seen by holding the vehicle at an angle and seeing the motors
   * ramp up on one side until they've maxed out throttle...saturating I to a
   * specified limit fixes this. The second feature defaults the I terms to 0 if
   * the throttle is at the minimum setting. This means the motors will not
   * start spooling up on the ground, and the I terms will always start from 0
   * on takeoff. This function updates the variables roll_PID, pitch_PID, and
   * yaw_PID which can be thought of as 1-D stablized signals. They are mixed to
   * the configuration of the vehicle in controlMixer().
   */
  float GyroZ;
  float GyroY;
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
  GyroY = GyroY_6050;
  GyroX = -GyroX_6050; // 安装位置
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
  GyroY = GyroY_9250;
  GyroX = -GyroX_9250;
#endif
  // rotate_speed

  rotate_error = rotate_speed_des * 1000.0 - GyroZ;
  rotate_error = constrain(rotate_error, -100, 100); // 100度每秒
  // thro_des=0.01*kp_rotate*rotate_error;//0-1

  // Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_3_pwm <
      1160) { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll =
      constrain(integral_roll, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID =
      0.01 *
      (Kp_roll_angle * error_roll + Ki_roll_angle * integral_roll -
       Kd_roll_angle *
           derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_3_pwm <
      1160) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch =
      constrain(integral_pitch, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID =
      .01 *
      (Kp_pitch_angle * error_pitch + Ki_pitch_angle * integral_pitch -
       Kd_pitch_angle *
           derivative_pitch); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ

  // yaw_des=yaw_des;
  error_yaw =
      yaw_des -
      57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) * cos(pitch_des / 57.3)) -
      GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_3_pwm <
      1160) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev = integral_roll;
  // Update pitch variables
  integral_pitch_prev = integral_pitch;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void increase_Clp() {
  float GyroX;
#if defined EXTIMU
  GyroX = Gyro_X_EXT;

#endif
#if defined INTIMU

  GyroX = -GyroX_6050; // 安装位置
#endif

  Clp_PID = .01 * k_Clp * (0 - GyroX);
  Ail_Clp = Clp_PID * 125.0;
}

void controlANGLE2() {
  // DESCRIPTION: Computes control commands based on state error (angle) in
  // cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning.
   * Not reccommended for first-time setup. See the documentation for tuning
   * this controller.
   */
  float GyroZ;
  float GyroY;
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
  GyroY = GyroY_6050;
  GyroX = -GyroX_6050; // 安装位置

  GyroZ = gyroFiltZ.apply(GyroZ_6050);
  GyroY = gyroFiltY.apply(GyroY_6050);
  GyroX = -gyroFiltX.apply(GyroX_6050); // 安装位置

  // Serial.print(GyroY_6050);
  // Serial.print(" ");
  // Serial.println(GyroY);
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
  GyroY = GyroY_9250;
  GyroX = -GyroX_9250;
#endif

#if defined EXTIMU
  GyroX = Gyro_X_EXT;
  GyroY = Gyro_Y_EXT;
  GyroZ = Gyro_Z_EXT;
  roll_IMU = roll_IMU_EXT;
  pitch_IMU = pitch_IMU_EXT;
  yaw_IMU = yaw_IMU_EXT;
#endif

  // Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
// Roll
#if defined SINGLE
  roll_eq = roll_IMU;
#else

  float phiA, phiB, phiC, phiD, phiE, phiF, phiG;

#if defined userotatesensor
  phiA = roll_IMU / 57.3;
  phiB = (roll_IMU + relativeAngle_ready) / 57.3;
  phiC = (roll_IMU + phiac) / 57.3;
  phiD = (roll_IMU + relativeAngle_ready + phibd) / 57.3;
  phiE = (roll_IMU + phiac + phice) / 57.3;
#else
  // 直接测量的方式
  phiA = roll_IMU / 57.3;
  phiB = phiB_raw / 57.3;
  phiC = phiC_raw / 57.3;
  phiD = phiD_raw / 57.3;
  phiE = phiE_raw / 57.3;
  phiF = phiF_raw / 57.3;
  phiG = phiG_raw / 57.3;

#endif

// roll_eq=(roll_IMU+roll_IMU+relativeAngle_ready)/2.0; //2机等效姿态
#if defined THREEPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC)) /
                 (cos(phiA) + cos(phiB) + cos(phiC))) *
            57.3; // 3机等效姿态
#elif defined FOURPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD)) /
                 (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD))) *
            57.3; // 4机等效姿态
#elif defined FIVEPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD) + sin(phiE)) /
                 (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD) + cos(phiE))) *
            57.3; // 5机等效姿态
#elif defined SEVENPLANE
  roll_eq = atan((sin(phiA) + sin(phiB) + sin(phiC) + sin(phiD) + sin(phiE) +
                  sin(phiF) + sin(phiG)) /
                 (cos(phiA) + cos(phiB) + cos(phiC) + cos(phiD) + cos(phiE) +
                  cos(phiF) + cos(phiG))) *
            57.3; // 7机等效姿态
#endif
#endif

  error_roll = roll_des - roll_eq;
  integral_roll_ol = integral_roll_prev_ol + error_roll * dt; // I
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol =
      constrain(integral_roll_ol, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_roll_ol = 0;
  }
  derivative_roll = (roll_IMU - roll_IMU_prev) / dt;
  roll_des_ol = Kp_roll_angle * error_roll +
                i_valid * Ki_roll_angle *
                    integral_roll_ol; // - Kd_roll_angle*derivative_roll;

  // Pitch
  // Serial.println(pitch_des_local);

#if defined APLANE

#if defined EVEN
  error_pitch = pitch_des_local + central_pitch - pitch_IMU;
#elif defined ODD
  error_pitch = pitch_des_local - pitch_IMU;
#endif

#else
  error_pitch = pitch_des_local - pitch_IMU;
#endif

  // Serial.println(error_pitch);
  // error_pitch = pitch_des_local - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol =
      constrain(integral_pitch_ol, -i_limit,
                i_limit); // saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_pitch_ol = 0;
  }
  derivative_pitch = (pitch_IMU - pitch_IMU_prev) / dt;
  pitch_des_ol = Kp_pitch_angle * error_pitch +
                 i_valid * Ki_pitch_angle *
                     integral_pitch_ol; // - Kd_pitch_angle*derivative_pitch;

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl * roll_des_ol;
  pitch_des_ol = Kl * pitch_des_ol;
  pitch_des_ol = (pitch_des_ol + sin(roll_IMU / 57.3) * -GyroZ) /
                 constrain(cos(roll_IMU / 57.3), 0.5, 1);
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll) * roll_des_prev + B_loop_roll * roll_des_ol;
  pitch_des_ol =
      (1.0 - B_loop_pitch) * pitch_des_prev + B_loop_pitch * pitch_des_ol;

  // Inner loop - PID on rate
  // Roll
  float Rollrate;
#if defined SINGLE
  Rollrate = GyroX;
#else
// Rollrate=(GYRO_X_B+GyroX)/2.0;
#if defined THREEPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GyroX) / 3.0;
#elif defined FIVEPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GYRO_X_D + GyroX) / 4.0;
#elif defined FIVEPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GYRO_X_D + GYRO_X_E + GyroX) / 5.0;
#elif defined SEVENPLANE
  Rollrate = (GYRO_X_B + GYRO_X_C + GYRO_X_D + GYRO_X_E + GYRO_X_F + GYRO_X_G +
              GyroX) /
             7.0; // 7机等效滚转角速度
#endif
#endif
  // Serial.println(Rollrate);

  error_roll = roll_des_ol - Rollrate;
  integral_roll_il = integral_roll_prev_il + error_roll * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il =
      constrain(integral_roll_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_roll_il = 0;
  }
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID =
      .01 *
      (Kff_roll_rate * roll_des_ol + Kp_roll_rate * error_roll +
       i_valid * Ki_roll_rate * integral_roll_il +
       Kd_roll_rate *
           derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  float tau_roll_pid = 1.0f / (6.28f * roll_pid_lpf_fc);
  float alpha_roll_pid = dt / (tau_roll_pid + dt);
  roll_PID_lpf = roll_PID_lpf + alpha_roll_pid * (roll_PID - roll_PID_lpf);

  // 指令的变化率
  if (!roll_pid_dot_initialized) {
    roll_PID_prev = roll_PID;
    roll_PID_dot = 0.0f;
    roll_PID_dot_lpf = 0.0f;
    roll_pid_dot_initialized = true;
  } else {
    if (dt > 1.0e-6f) {
      roll_PID_dot = (roll_PID - roll_PID_prev) / dt;
    } else {
      roll_PID_dot = 0.0f;
    }

    if (roll_pid_dot_lpf_fc > 0.0f && dt > 1.0e-6f) {
      float tau_dot = 1.0f / (6.28f * roll_pid_dot_lpf_fc);
      float alpha_dot = dt / (tau_dot + dt);
      roll_PID_dot_lpf =
          roll_PID_dot_lpf + alpha_dot * (roll_PID_dot - roll_PID_dot_lpf);
    } else {
      roll_PID_dot_lpf = roll_PID_dot;
    }
    roll_PID_prev = roll_PID;
  }

  // Serial.println(roll_PID_dot_lpf);

  // Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il =
      constrain(integral_pitch_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_pitch_il = 0;
  }
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID =
      .01 *
      (Kff_pitch_rate * pitch_des_ol + Kp_pitch_rate * error_pitch +
       i_valid * Ki_pitch_rate * integral_pitch_il +
       Kd_pitch_rate *
           derivative_pitch); // Scaled by .01 to bring within -1 to 1 range
  // Serial.println(integral_pitch_il);
  // Yaw
  error_yaw =
      yaw_des -
      57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) * cos(pitch_des / 57.3)) -
      GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  if (ModeChange == 1) {
    integral_yaw = 0;
  }
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kff_yaw_rate * (yaw_des - 57.3 * (9.8 / V_cruise * tan(roll_des / 57.3) *
                                         cos(pitch_des / 57.3))) +
       Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  // Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlRATE() {
  // DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the
   * error is now the desired rate - raw gyro reading.
   */

  float GyroZ;
  float GyroY;
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroZ = GyroZ_6050;
  GyroY = GyroY_6050;
  GyroX = -GyroX_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroZ = GyroZ_9250;
  GyroY = GyroY_9250;
  GyroX = -GyroX_9250;
#endif
#if defined EXTIMU
  GyroX = Gyro_X_EXT;
  GyroY = Gyro_Y_EXT;
  GyroZ = Gyro_Z_EXT;
#endif

  // Roll
  error_roll = roll_des * 3.0 - GyroX;
  integral_roll = integral_roll_prev + error_roll * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll =
      constrain(integral_roll, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev) / dt;
  roll_PID =
      .01 *
      (Kff_roll_rate * roll_des + Kp_roll_rate * error_roll +
       Kd_roll_rate *
           derivative_roll); // Scaled by .01 to bring within -1 to 1 range

  // Pitch
  error_pitch = pitch_des_local * 3.0 - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch =
      constrain(integral_pitch, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev) / dt;
  pitch_PID =
      .01 *
      (Kff_pitch_rate * pitch_des + Kp_pitch_rate * error_pitch +
       Kd_pitch_rate *
           derivative_pitch); // Scaled by .01 to bring within -1 to 1 range

  // Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des * 3.0 - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw =
      constrain(integral_yaw, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev) / dt;
  yaw_PID =
      .01 *
      (Kff_yaw_rate * yaw_des + Kp_yaw * error_yaw + Ki_yaw * integral_yaw +
       Kd_yaw * derivative_yaw); // Scaled by .01 to bring within -1 to 1 range

  // Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  // GyroX_prev = GyroX;//这行有什么用啊 似乎没用 而且不会干扰getimudata吗
  // Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  // GyroY_prev = GyroY;//这行有什么用啊 似乎没用 而且不会干扰getimudata吗
  // Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

int PITCH_INDI_control() {
  // 1. 读取当前俯仰角速度原始值。
  // 这里先只统一到当前工程已经在用的角速度定义，不改变原有轴系约定。
  float GyroY_raw;
#if defined USE_MPU6050_I2C
  GyroY_raw = GyroY_6050;
#endif
#if defined USE_MPU9250_SPI
  GyroY_raw = GyroY_9250;
#endif
#if defined EXTIMU
  GyroY_raw = Gyro_Y_EXT;
#endif

  // 2. 使用独立的低通滤波器得到 INDI 专用的俯仰角速度 q。
  // 这样做有两个目的：
  // - 不复用现有 controlANGLE2() 里的滤波状态，避免互相串扰；
  // - 让 q 与 dq 的噪声口径更接近，减小 INDI 中 q 和 dq 不一致的问题。
  float GyroY_filt = 0.0f;
  const int queue_len = 64;
  static bool indi_initialized = false;
  static float delayed_pwm_queue[queue_len];
  static int queue_head = 0;
  static float delta_e_est_deg = 0.0f;
  static float delta_e_cmd_prev_deg = 0.0f;
  static float delta_e_cmd_lpf_prev_deg = 0.0f;

  // 3. 如果线性映射或舵效系数还没有填写，直接返回一个安全中位 PWM。
  // 这里是保护逻辑，避免参数未填完时函数输出发散。
  if (fabsf(indi_pitch_pwm_to_deg_k) < 1.0e-6f ||
      fabsf(indi_pitch_effectiveness) < 1.0e-6f) {
    indi_pitch_q_des_log = 0.0f;
    indi_pitch_q_filt_log = 0.0f;
    indi_pitch_dq_des_log = 0.0f;
    indi_pitch_dq_used_log = dq;
    indi_pitch_delta_e_cmd_deg_log = 0.0f;
    indi_pitch_delta_e_est_deg_log = eleprev;
    indi_pitch_pwm_cmd_log = 1520.0f + pwm_channel3_trim;
    return constrain((int)(1520 + pwm_channel3_trim), (int)indi_pitch_pwm_min,
                     (int)indi_pitch_pwm_max);
  }

  // 4. 计算 trim 对应的舵偏角。
  // 后面所有舵机状态初始化都以这个配平点为基准，而不是默认 0 度舵偏。
  const float pwm_trim_center = 1520.0f + pwm_channel3_trim;
  const float delta_e_trim_deg =
      indi_pitch_pwm_to_deg_k * pwm_trim_center + indi_pitch_pwm_to_deg_b;

  // 5. 第一次进入函数时：
  // - 初始化独立滤波器的内部状态
  // - 初始化纯延迟队列
  // - 初始化实际舵偏估计值和上一次指令值
  if (!indi_initialized) {
    gyroFiltYIndi.reset(GyroY_raw);
    for (int i = 0; i < queue_len; i++) {
      delayed_pwm_queue[i] = pwm_trim_center;
    }
    delta_e_est_deg = delta_e_trim_deg;
    delta_e_cmd_prev_deg = delta_e_trim_deg;
    delta_e_cmd_lpf_prev_deg = delta_e_trim_deg;
    eleprev = delta_e_trim_deg;
    indi_initialized = true;
  }
  GyroY_filt = gyroFiltYIndi.apply(GyroY_raw);

  // 6. 先计算俯仰角误差，再由角度误差生成期望俯仰角速度 q_des。
  // 这里做的是“外环角度误差 -> 内环角速度目标”的转换，
  // 所以不能直接拿 pitch_des_local 本身当作角速度目标，必须减去当前俯仰角。

  float pitch_des_local_rate_raw;
  static bool pitch_des_local_rate_lpf_initialized = false;
  if (dt > 1.0e-6f) {
    pitch_des_local_rate_raw = (pitch_des_local - pitch_des_local_last) / dt;
  } else {
    pitch_des_local_rate_raw = 0.0f;
  }
  pitch_des_local_last = pitch_des_local;
  pitch_des_local_rate_raw = constrain(pitch_des_local_rate_raw, -10.0, 10.0);

  if (!pitch_des_local_rate_lpf_initialized) {
    pitch_des_local_rate = pitch_des_local_rate_raw;
    pitch_des_local_rate_lpf_initialized = true;
  } else if (pitch_des_local_rate_lpf_fc > 0.0f && dt > 1.0e-6f) {
    float tau_pitch_des_rate =
        1.0f / (6.28318530718f * pitch_des_local_rate_lpf_fc);
    float alpha_pitch_des_rate = dt / (tau_pitch_des_rate + dt);
    pitch_des_local_rate = pitch_des_local_rate +
                           alpha_pitch_des_rate * (pitch_des_local_rate_raw -
                                                   pitch_des_local_rate);
  } else {
    pitch_des_local_rate = pitch_des_local_rate_raw;
  }

  float pitch_error_deg = pitch_des_local - pitch_IMU;
  float rate_des_deg_s = pitch_error_deg * 3.0f + pitch_des_local_rate;
  float q_des_deg_s = (rate_des_deg_s + sin(roll_IMU / 57.3) *
                                            -gyroFiltZIndi.apply(GyroZ_6050)) /
                      cos(roll_IMU / 57.3);
  q_des_deg_s = constrain(q_des_deg_s, -120, 120);

  // 7. 由速率误差生成期望角加速度 dq_des。
  // 这一步是 INDI 的外层：先问“我希望产生多大的俯仰角加速度”。
  float dq_des_deg_s2 = indi_pitch_q_gain * (q_des_deg_s - GyroY_filt);
  // Serial.println(dq_des_deg_s2);
  //  8. INDI 增量控制律。
  //  舵效 indi_pitch_effectiveness 的物理意义是：
  //    dq / delta_e
  //  所以这里用 (dq_des - dq_meas) / 舵效，得到还需要增加多少舵偏角。
  float delta_e_indi_deg =
      0.7f * (dq_des_deg_s2 - dq) / indi_pitch_effectiveness;

  // 9. 以“当前实际舵偏估计值”为基准，加上 INDI 算出的增量，得到新的舵偏目标。
  float delta_e_cmd_deg = delta_e_est_deg + delta_e_indi_deg;

  // 10. 对舵偏变化率做限幅。
  // 这一步的目的是避免 INDI 因 dq
  // 噪声或舵效估计误差，一次跳出非常大的舵偏命令。
  float max_delta_deg = indi_pitch_rate_limit_deg_s * dt;
  float delta_step_deg = delta_e_cmd_deg - delta_e_cmd_prev_deg;
  delta_step_deg = constrain(delta_step_deg, -max_delta_deg, max_delta_deg);
  delta_e_cmd_deg = delta_e_cmd_prev_deg + delta_step_deg;

  // 11. 对舵偏角本身做限幅。
  // 这里建议你后续直接填真实可用的升降舵最大/最小偏角。
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, indi_pitch_deflection_min_deg,
                              indi_pitch_deflection_max_deg);

  // 12. 对最终输出舵偏指令再做一阶低通。
  // 这一步压的是“打给舵机的命令带宽”，不是状态反馈带宽。
  // 这样可以减少 INDI 因 dq 噪声带来的高频抖舵。
  if (indi_pitch_cmd_lpf_fc_hz > 0.0f && dt > 1.0e-6f) {
    float tau_cmd = 1.0f / (6.28318530718f * indi_pitch_cmd_lpf_fc_hz);
    float alpha_cmd = dt / (tau_cmd + dt);
    delta_e_cmd_deg = delta_e_cmd_lpf_prev_deg +
                      alpha_cmd * (delta_e_cmd_deg - delta_e_cmd_lpf_prev_deg);
  }
  delta_e_cmd_deg = constrain(delta_e_cmd_deg, indi_pitch_deflection_min_deg,
                              indi_pitch_deflection_max_deg);
  delta_e_cmd_prev_deg = delta_e_cmd_deg;
  delta_e_cmd_lpf_prev_deg = delta_e_cmd_deg;

  // 13. 将目标舵偏角通过你测得的线性映射，反算成舵机 PWM。
  // 若你的关系是 delta_e = k * pwm + b，则反解为 pwm = (delta_e - b) / k。
  float pwm_cmd =
      (delta_e_cmd_deg - indi_pitch_pwm_to_deg_b) / indi_pitch_pwm_to_deg_k;
  pwm_cmd = constrain(pwm_cmd, indi_pitch_pwm_min, indi_pitch_pwm_max);

  // 14. 将“刚刚发出的 PWM 命令”压入纯延迟队列。
  // 后面实际舵偏估计器不会立刻看到这个命令，而是等 delay_samples
  // 个周期以后才看到。
  delayed_pwm_queue[queue_head] = pwm_cmd;
  queue_head = (queue_head + 1) % queue_len;

  // 15. 将你测到的 0.03s 舵机纯延迟，离散成若干个采样周期。
  int delay_samples = 0;
  if (dt > 1.0e-6f) {
    delay_samples = (int)(indi_pitch_servo_delay_s / dt + 0.5f);
  }
  delay_samples = constrain(delay_samples, 0, queue_len - 1);

  // 16. 从延迟队列中取出“经过纯延迟后才真正进入舵机的一条 PWM 命令”。
  int delayed_index = queue_head - 1 - delay_samples;
  if (delayed_index < 0) {
    delayed_index += queue_len;
  }
  float pwm_delayed = delayed_pwm_queue[delayed_index];
  float delta_e_delayed_deg =
      indi_pitch_pwm_to_deg_k * pwm_delayed + indi_pitch_pwm_to_deg_b;

  // 17. 用一阶舵机动态模型更新“当前实际舵偏估计值”。
  // 这就是你测到的 tau = 0.015s 所对应的执行器模型。
  float tau = indi_pitch_servo_tau_s;
  if (tau < 1.0e-4f) {
    delta_e_est_deg = delta_e_delayed_deg;
  } else {
    float alpha_servo = dt / (tau + dt);
    delta_e_est_deg += alpha_servo * (delta_e_delayed_deg - delta_e_est_deg);
  }

  // 18. eleprev 保留为“当前估计的实际升降舵偏角”，方便你后续打 log 或直接观察。
  eleprev = delta_e_est_deg;
  indi_pitch_q_des_log = q_des_deg_s;
  indi_pitch_q_filt_log = GyroY_filt;
  indi_pitch_dq_des_log = dq_des_deg_s2;
  indi_pitch_dq_used_log = dq;
  indi_pitch_delta_e_cmd_deg_log = delta_e_cmd_deg;
  indi_pitch_delta_e_est_deg_log = delta_e_est_deg;
  indi_pitch_pwm_cmd_log = pwm_cmd;

  // 18. 函数最终只返回一个独立计算好的升降舵 PWM。
  // 目前它不会自动接入现有控制链，只有你显式调用它时才会生效。
  return (int)(pwm_cmd + 0.5f);
}

void scaleCommands() {
  // DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo
  // protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us
   * for OneShot125 protocol. sX_command_scaled variables from the mixer
   * function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in
   * commandMotors(). sX_command_PWM are updated which are used to command the
   * servos.
   */

  // Scaled to 1100-1940  mid 1520 for servo library
  Aail1_PWM = 1520 + 1000 * (Aail1_scaled);
  Aail2_PWM = 1520 + 1000 * (Aail2_scaled);
  Aele_PWM = 1520 + 1000 * (Aele_scaled);
  Athro_PWM = 1100 + 1000 * (Athro_scaled);
  Arudd_PWM = 1520 + 1000 * (Arudd_scaled);

  Bail1_PWM = 1520 + 1000 * (Bail1_scaled);
  Bail2_PWM = 1520 + 1000 * (Bail2_scaled);
  Bthro_PWM = 1100 + 1000 * (Bthro_scaled);
  Brudd_PWM = 1520 + 1000 * (Brudd_scaled);
  B_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Bail1_scaled + Bail2_scaled) / 2;

  Cail1_PWM = 1520 + 1000 * (Cail1_scaled);
  Cail2_PWM = 1520 + 1000 * (Cail2_scaled);
  Cthro_PWM = 1100 + 1000 * (Cthro_scaled);
  Crudd_PWM = 1520 + 1000 * (Crudd_scaled);
  C_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Cail1_scaled + Cail2_scaled) / 2;

  Dail1_PWM = 1520 + 1000 * (Dail1_scaled);
  Dail2_PWM = 1520 + 1000 * (Dail2_scaled);
  Dthro_PWM = 1100 + 1000 * (Dthro_scaled);
  Drudd_PWM = 1520 + 1000 * (Drudd_scaled);
  D_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Dail1_scaled + Dail2_scaled) / 2;

  Eail1_PWM = 1520 + 1000 * (Eail1_scaled);
  Eail2_PWM = 1520 + 1000 * (Eail2_scaled);
  Ethro_PWM = 1100 + 1000 * (Ethro_scaled);
  Erudd_PWM = 1520 + 1000 * (Erudd_scaled);
  E_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Eail1_scaled + Eail2_scaled) / 2;

  Fail1_PWM = 1520 + 1000 * (Fail1_scaled);
  Fail2_PWM = 1520 + 1000 * (Fail2_scaled);
  Fthro_PWM = 1100 + 1000 * (Fthro_scaled);
  Frudd_PWM = 1520 + 1000 * (Frudd_scaled);
  F_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Fail1_scaled + Fail2_scaled) / 2;

  Gail1_PWM = 1520 + 1000 * (Gail1_scaled);
  Gail2_PWM = 1520 + 1000 * (Gail2_scaled);
  Gthro_PWM = 1100 + 1000 * (Gthro_scaled);
  Grudd_PWM = 1520 + 1000 * (Grudd_scaled);
  G_ele_command_PWM_Manual = 1520 + 1000 * (Aele_scaled) +
                             0.5 * 1000 * (Gail1_scaled + Gail2_scaled) / 2;

  s6_command_PWM = s6_command_scaled * 180;
  s7_command_PWM = s7_command_scaled * 180;
  // Constrain commands to servos within servo library bounds
  Aail1_PWM = constrain(Aail1_PWM, 1100, 1920);
  Aail2_PWM = constrain(Aail2_PWM, 1100, 1920);
  Aele_PWM = constrain(Aele_PWM, 1100, 1920); // 贵的飞机限幅
  Athro_PWM = constrain(Athro_PWM, 1100, 1920);
  Arudd_PWM = constrain(Arudd_PWM, 1100, 1920);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);
}

void getCommands() {
  // DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands.
   * channel_x_pwm is the raw command used in the rest of the loop. If using a
   * PWM or PPM receiver, the radio commands are retrieved from a function in
   * the readPWM file separate from this one which is running a bunch of
   * interrupts to continuously update the radio readings. If using an SBUS
   * receiver, the alues are pulled from the SBUS library directly. The raw
   * radio commands are filtered with a first order low-pass filter to eliminate
   * any really high frequency noise.
   */

#if defined USE_PPM_RX || defined USE_PWM_RX
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
  channel_5_pwm = getRadioPWM(5);
  channel_6_pwm = getRadioPWM(6);

#elif defined USE_SBUS_RX
  if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame)) {
    // sBus scaling below is for Taranis-Plus and X4R-SB
    float scale = 0.615;
    float bias = 895.0;
    channel_1_pwm = sbusChannels[0] * scale + bias;
    channel_2_pwm = sbusChannels[1] * scale + bias;
    channel_3_pwm = sbusChannels[2] * scale + bias;
    channel_4_pwm = sbusChannels[3] * scale + bias;
    channel_5_pwm = sbusChannels[4] * scale + bias;
    channel_6_pwm = sbusChannels[5] * scale + bias;
    channel_7_pwm = sbusChannels[6] * scale + bias;
    channel_8_pwm = sbusChannels[7] * scale + bias;
  }

#elif defined USE_DSM_RX
  if (DSM.timedOut(micros())) {
    // Serial.println("*** DSM RX TIMED OUT ***");
  } else if (DSM.gotNewFrame()) {
    uint16_t values[num_DSM_channels];
    DSM.getChannelValues(values, num_DSM_channels);

    channel_1_pwm = values[0];
    channel_2_pwm = values[1];
    channel_3_pwm = values[2];
    channel_4_pwm = values[3];
    channel_5_pwm = values[4];
    channel_6_pwm = values[5];
  }
#endif

  // Low-pass the critical commands and update previous values
  float b = 0.7; // Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b) * channel_1_pwm_prev + b * channel_1_pwm;
  channel_2_pwm = (1.0 - b) * channel_2_pwm_prev + b * channel_2_pwm;
  channel_3_pwm = (1.0 - b) * channel_3_pwm_prev + b * channel_3_pwm;
  channel_4_pwm = (1.0 - b) * channel_4_pwm_prev + b * channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  // DESCRIPTION: If radio gives garbage values, set all commands to default
  // values
  /*
   * Radio connection failsafe used to check if the getCommands() function is
   * returning acceptable pwm values. If any of the commands are lower than 800
   * or higher than 2200, then we can be certain that there is an issue with the
   * radio connection (most likely hardware related). If any of the channels
   * show this failure, then all of the radio commands channel_x_pwm are set to
   * default failsafe values specified in the setup. Comment out this function
   * when troubleshooting your radio connection in case any extreme values are
   * triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;
  int check7 = 0;
  int check8 = 0;
  int check9 = 0;
  int check10 = 0;

  // Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal)
    check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal)
    check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal)
    check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal)
    check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal)
    check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal)
    check6 = 1;
  if (channel_7_pwm > maxVal || channel_7_pwm < minVal)
    check7 = 1;
  if (channel_8_pwm > maxVal || channel_8_pwm < minVal)
    check8 = 1;

  // If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6 + check7 + check8) >
      0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
    channel_7_pwm = channel_7_fs;
    channel_8_pwm = channel_8_fs;
  }
}

void commandMotors() {}

void armMotors() {}

void calibrateESCs() {}

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

void switchRollYaw(int reverseRoll, int reverseYaw) {
  // DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type
  // configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired
   * reversing of the roll axis and yaw axis, respectively. Reversing of the
   * roll or yaw axis may be needed when switching between the two for some
   * dynamic configurations. Inputs of 1, 1 does not reverse either of them,
   * while -1, 1 will reverse the output corresponding to the new roll axis.
   * This function may be replaced in the future by a function that switches the
   * IMU data instead (so that angle can also be estimated with the IMU tilted
   * 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw * roll_des;
  roll_des = reverseRoll * switch_holder;
}

void throttleCut() {}

void calibrateMagnetometer() {
#if defined USE_MPU9250_SPI
  float success;
  Serial.println("Beginning magnetometer calibration in");
  Serial.println("3...");
  delay(1000);
  Serial.println("2...");
  delay(1000);
  Serial.println("1...");
  delay(1000);
  Serial.println("Rotate the IMU about all axes until complete.");
  Serial.println(" ");
  success = mpu9250.calibrateMag();
  if (success) {
    Serial.println("Calibration Successful!");
    Serial.println("Please comment out the calibrateMagnetometer() function "
                   "and copy these values into the code:");
    Serial.print("float MagErrorX_9250 = ");
    Serial.print(mpu9250.getMagBiasX_uT());
    Serial.println(";");
    Serial.print("float MagErrorY_9250 = ");
    Serial.print(mpu9250.getMagBiasY_uT());
    Serial.println(";");
    Serial.print("float MagErrorZ_9250 = ");
    Serial.print(mpu9250.getMagBiasZ_uT());
    Serial.println(";");
    Serial.print("float MagScaleX_9250 = ");
    Serial.print(mpu9250.getMagScaleFactorX());
    Serial.println(";");
    Serial.print("float MagScaleY_9250 = ");
    Serial.print(mpu9250.getMagScaleFactorY());
    Serial.println(";");
    Serial.print("float MagScaleZ_9250 = ");
    Serial.print(mpu9250.getMagScaleFactorZ());
    Serial.println(";");
    Serial.println(" ");

    Serial.println("If you are having trouble with your attitude estimate at a "
                   "new flying location, repeat this process as needed.");
  } else {
    Serial.println(
        "Calibration Unsuccessful. Please reset the board and try again.");
  }

  while (1)
    ; // Halt code so it won't enter main loop until this function commented out
#endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent "
                 "magnetometer.");
  while (1)
    ; // Halt code so it won't enter main loop until this function commented out
}

void loopRate(int freq) {
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable
   * and whatnot. Interrupt routines running in the background cause the loop
   * rate to fluctuate. This function basically just waits at the end of every
   * loop iteration until the correct time has passed since the start of the
   * current loop for the desired loop rate in Hz. 2kHz is a good rate to be at
   * because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us
   * have a little room to add extra computations and remain above 2kHz, without
   * needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); // Pin 13 is built in LED

    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
    } else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink(int numBlinks, int upTime, int downTime) {
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6:"));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX_6050:"));
    Serial.print(GyroX_6050);
    Serial.print(F(" GyroY_6050:"));
    Serial.print(GyroY_6050);
    Serial.print(F(" GyroZ_6050:"));
    Serial.println(GyroZ_6050);

    Serial.print(F("GyroX_9250:"));
    Serial.print(GyroX_9250);
    Serial.print(F(" GyroY_9250:"));
    Serial.print(GyroY_9250);
    Serial.print(F(" GyroZ_9250:"));
    Serial.println(GyroZ_9250);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX_6050:"));
    Serial.print(AccX_6050);
    Serial.print(F(" AccY_6050:"));
    Serial.print(AccY_6050);
    Serial.print(F(" AccZ_6050:"));
    Serial.println(AccZ_6050);

    Serial.print(F("AccX_9250:"));
    Serial.print(AccX_9250);
    Serial.print(F(" AccY_9250:"));
    Serial.print(AccY_9250);
    Serial.print(F(" AccZ_9250:"));
    Serial.println(AccZ_9250);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(MagX_9250);
    Serial.print(F(" MagY:"));
    Serial.print(MagY_9250);
    Serial.print(F(" MagZ:"));
    Serial.println(MagZ_9250);
  }
}

void printConfigurationData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
#if defined userotatesensor
    Serial.print(F("phiab:"));
    Serial.print(relativeAngle_ready);
    Serial.print(F("phiac:"));
    Serial.print(phiac);
    Serial.print(F("phibd:"));
    Serial.print(phibd);
    Serial.print(F("phice:"));
    Serial.print(phice);
    Serial.print(F("phidf:"));
    Serial.print(phidf);
    Serial.print(F("phieg:"));
    Serial.print(phieg);
#else
    Serial.print(F("phiab:"));
    Serial.print(Phiab_Mea);
    Serial.print(F("phiac:"));
    Serial.print(Phiac_Mea);
    Serial.print(F("phibd:"));
    Serial.print(Phibd_Mea);
    Serial.print(F("phice:"));
    Serial.print(Phice_Mea);
    Serial.print(F("phidf:"));
    Serial.print(Phidf_Mea);
    Serial.print(F("phieg:"));
    Serial.print(Phieg_Mea);
    Serial.print(F("phieq:"));
    Serial.print(roll_eq);
#endif
    Serial.print(F("Pb:"));
    Serial.print(GYRO_X_B);
    Serial.print(F("Pc:"));
    Serial.print(GYRO_X_C);
    Serial.print(F("Pd:"));
    Serial.print(GYRO_X_D);
    Serial.print(F("Pe:"));
    Serial.print(GYRO_X_E);
    Serial.print(F("Pf:"));
    Serial.print(GYRO_X_F);
    Serial.print(F("Pg:"));
    Serial.println(GYRO_X_G);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.print(yaw_PID);
    Serial.print(F(" Phiab_PID:"));
    Serial.print(Phiab_PID);
    Serial.print(F(" Phiac_PID:"));
    Serial.println(Phiac_PID);
    Serial.print(F(" Phibd_PID:"));
    Serial.print(Phibd_PID);
    Serial.print(F(" Phice_PID:"));
    Serial.println(Phice_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command:"));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command:"));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command:"));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Aail1:"));
    Serial.print(Aail1_PWM);
    Serial.print(F(" Aail2:"));
    Serial.print(Aail2_PWM);
    Serial.print(F(" Aele:"));
    Serial.print(Aele_PWM);
    Serial.print(F(" Athro:"));
    Serial.print(Athro_PWM);
    Serial.print(F(" Arudd:"));
    Serial.print(Arudd_PWM);
    Serial.print(F(" s6_command:"));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command:"));
    Serial.println(s7_command_PWM);
  }
}

void printQuaternion() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("q0: "));
    Serial.print(q0);
    Serial.print(F(" q1: "));
    Serial.print(q1);
    Serial.print(F(" q2: "));
    Serial.print(q2);
    Serial.print(F(" q3: "));
    Serial.println(q3);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt * 1000000.0); // 一个微秒microsecons=1/1000000s
  }
}

void getairspeed() {
  static unsigned long lastAirspeedRead = 0;
  if (micros() - lastAirspeedRead > 20000) {
    lastAirspeedRead = micros();
    airspeed_A = airspeedSensor.getAirspeed();
    // Serial.println(airspeed_A);
  }
}
void loggerSINGLE() {
  float invFreq = 1.0 / logfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastLogTime < invFreq)
    return;
  lastLogTime = checker;

  // dataFile = SD.open(filename_sd, FILE_WRITE);
  // 光打开不close 拔电就没了，但是每次都打开又close很浪费时间。
  //  read three sensors and append to the string:
  dataString =
      String(current_time) + "," + String(q0) + "," + String(q1) + "," +
      String(q2) + "," + String(q3) + "," + String(roll_IMU) + "," +
      String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(roll_des) + "," +
      String(pitch_des_local) + "," + String(yaw_des) + "," +
      String(channel_1_pwm) + "," + String(channel_2_pwm) + "," +
      String(channel_3_pwm) + "," + String(channel_4_pwm) + "," +
      String(channel_5_pwm) + "," + String(Aail1_PWM - pwm_channel1_trim) +
      "," + String(Aail2_PWM - pwm_channel2_trim) + "," +
      String(Aele_PWM - pwm_channel3_trim) + "," + String(Athro_PWM) + "," +
      String(Arudd_PWM) + "," + String(-GyroX_6050) + "," + String(GyroY_9250) +
      "," + String(GyroZ_9250) + "," + String(Pab) + "," +
      String(relativeAngle_ready) + "," + String(Phiab_des) + "," +
      String(-AccX_9250) + "," + String(AccY_9250) + "," + String(AccZ_9250);
  // Serial.println(dataString);
  dataFile.println(dataString);

  // if the file is available, write to it:
  /*
  if (dataFile) {
    dataFile.println(dataString);
    if(++sd_counter >= 50) { //每50次写入刷新一次
      dataFile.flush();
      sd_counter = 0;
    }
     //dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  */

  // 在loop()最后添加定期关闭/重新打开

  if (millis() - fileCycle > 2000) { // 每2秒重新打开
    dataFile.flush();
    /*
  dataFile.close();
  dataFile = SD.open(filename_sd, FILE_WRITE);
  if(!SD.open(filename_sd, FILE_WRITE))
  {
    display.fillRect(0, 0, 120, 8, SSD1306_BLACK);
    displaySD("BLACK BOX MISS");
    }
    */
    fileCycle = millis();
  }
}

void loggerTEAM() {
  float invFreq = 1.0 / logfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastLogTime < invFreq)
    return;
  lastLogTime = checker;

  float pitch_des_local_log;
#if defined APLANE

#if defined EVEN
  pitch_des_local_log = pitch_des_local + central_pitch;
#elif defined ODD
  pitch_des_local_log = pitch_des_local;
#endif

#else
  pitch_des_local_log = pitch_des_local;

#endif
  A_pitch_sp = pitch_des_local_log;
  // dataFile = SD.open(filename_sd, FILE_WRITE);
  // 光打开不close 拔电就没了，但是每次都打开又close很浪费时间。
  //  read three sensors and append to the string:
  float Phiab_Mea_logger;
  float Phiac_Mea_logger;
  float Phibd_Mea_logger;
  float Phice_Mea_logger;
  float Phidf_Mea_logger;
  float Phieg_Mea_logger;

#if defined userotatesensor
  Phiab_Mea_logger = relativeAngle_ready; // phiab是A机自己测的。
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea_logger = phiac;
  Phibd_Mea_logger = phibd;
  Phice_Mea_logger = phice;
  Phidf_Mea_logger = phidf;
  Phieg_Mea_logger = phieg;
#else
  Phiab_Mea_logger = phiB_raw - roll_IMU; //
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea_logger = phiC_raw - roll_IMU; //
  Phibd_Mea_logger = phiD_raw - phiB_raw;
  Phice_Mea_logger = phiE_raw - phiC_raw;
  Phidf_Mea_logger = phiF_raw - phiD_raw;
  Phieg_Mea_logger = phiG_raw - phiE_raw;
#endif

  dataString =
      String(current_time) + "," + String(roll_IMU) + "," + String(roll_eq) +
      "," + String(pitch_IMU) + "," + String(yaw_IMU) + "," + String(roll_des) +
      "," + String(pitch_des_local_log) + "," + String(yaw_des) + "," +
      String(channel_1_pwm) + "," + String(channel_2_pwm) + "," +
      String(channel_3_pwm) + "," + String(channel_4_pwm) + "," +
      String(channel_5_pwm) + "," + String(channel_6_pwm) + "," +
      String(channel_7_pwm) + "," + String(channel_8_pwm) + "," +
      String(Aail1_PWM - pwm_channel1_trim) + "," +
      String(Aail2_PWM - pwm_channel2_trim) + "," +
      String(Aele_PWM - pwm_channel3_trim) + "," + String(Athro_PWM) + "," +
      String(Arudd_PWM) + "," + String(Bail1_PWM) + "," + String(Bail2_PWM) +
      "," + String(Bthro_PWM) + "," + String(Brudd_PWM) + "," +
      String(Cail1_PWM) + "," + String(Cail2_PWM) + "," + String(Cthro_PWM) +
      "," + String(Crudd_PWM) + "," + String(Dail1_PWM) + "," +
      String(Dail2_PWM) + "," + String(Dthro_PWM) + "," + String(Drudd_PWM) +
      "," + String(Eail1_PWM) + "," + String(Eail2_PWM) + "," +
      String(Ethro_PWM) + "," + String(Erudd_PWM) + "," + String(Fail1_PWM) +
      "," + String(Fail2_PWM) + "," + String(Fthro_PWM) + "," +
      String(Frudd_PWM) + "," + String(Gail1_PWM) + "," + String(Gail2_PWM) +
      "," + String(Gthro_PWM) + "," + String(Grudd_PWM) + "," + String(Pab) +
      "," + String(Pac) + "," + String(Pbd) + "," + String(Pce) + "," +
      String(Pdf) + "," + String(Peg) + "," + String(Phiab_Mea_logger) + "," +
      String(Phiab_des) + "," + String(Phiac_Mea_logger) + "," +
      String(Phibd_Mea_logger) + "," + String(Phice_Mea_logger) + "," +
      String(Phidf_Mea_logger) + "," + String(Phieg_Mea_logger) + "," +
      String(A_pitch_sp) + "," + String(B_pitch_sp) + "," + String(C_pitch_sp) +
      "," + String(D_pitch_sp) + "," + String(E_pitch_sp) + "," +
      String(F_pitch_sp) + "," + String(G_pitch_sp) + "," + String(thetaB_raw) +
      "," + String(thetaC_raw) + "," + String(thetaD_raw) + "," +
      String(thetaE_raw) + "," + String(thetaF_raw) + "," + String(thetaG_raw) +
      "," + String(Bele_PWM - pwm_channel3B_trim) + "," +
      String(Cele_PWM - pwm_channel3C_trim) + "," +
      String(Dele_PWM - pwm_channel3D_trim) + "," +
      String(Eele_PWM - pwm_channel3E_trim) + "," +
      String(Fele_PWM - pwm_channel3F_trim) + "," +
      String(Gele_PWM - pwm_channel3G_trim) + "," + String(rollAB_rad_Qua) +
      "," + String(roll_IMU_EXT) + "," + String(pitch_IMU_EXT) + "," +
      String(yaw_IMU_EXT) + "," + String(-AccX_6050) + "," + String(AccY_6050) +
      "," + String(AccZ_6050) + "," + String(-GyroX_6050) + "," +
      String(GyroY_6050) + "," + String(GyroZ_6050) + "," + String(Gyro_X_EXT) +
      "," + String(Gyro_Y_EXT) + "," + String(Gyro_Z_EXT) + "," +
      String(Phiab_PID) + "," + String(Phiac_PID) + "," + String(Phibd_PID) +
      "," + String(Phice_PID) + "," + String(Phidf_PID) + "," +
      String(Phieg_PID) + "," + String(roll_PID) + "," + String(pitch_PID) +
      "," + String(airspeed_A) + "," + String(Strain_value1) + "," +
      String(Strain_value2) + "," + String(Strain_value3) + "," +
      String(Strain_value4) + "," + String(Strain_value5) + "," +
      String(airdata.aoa) + "," + String(airdata.aos) + "," +
      String(airdata.tas) + "," + String(dp) + "," + String(dq) + "," +
      String(dr) + "," + String(indi_pitch_q_des_log) + "," +
      String(indi_pitch_q_filt_log) + "," + String(indi_pitch_dq_des_log) +
      "," + String(indi_pitch_dq_used_log) + "," +
      String(indi_pitch_delta_e_cmd_deg_log) + "," +
      String(indi_pitch_delta_e_est_deg_log) + "," +
      String(indi_pitch_pwm_cmd_log);
  dataFile.println(dataString);

  // if the file is available, write to it:

  // 在loop()最后添加定期关闭/重新打开

  if (millis() - fileCycle > 2000) { // 每2秒重新打开
    dataFile.flush();

    fileCycle = millis();
  }
}

void findMaxFileNumber() {
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry)
      break;

    String name = entry.name();
    if (name.startsWith("datalog") && name.endsWith(".txt")) {
      // 提取编号部分
      String numStr = name.substring(7, 10); // 从第8个字符开始取3位

      // 验证是否为纯数字
      bool valid = true;
      for (uint8_t i = 0; i < 3; i++) {
        if (!isDigit(numStr.charAt(i))) {
          valid = false;
          break;
        }
      }

      if (valid) {
        uint16_t num = numStr.toInt();
        if (num > fileNumber) {
          fileNumber = num;
        }
      }
    }
    entry.close();
  }
  root.close();

  fileNumber++;                               // 新文件编号递增
  fileNumber = constrain(fileNumber, 1, 999); // 限制在1-999之间
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

//=========================================================================================//

// HELPER FUNCTIONS

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

void displayID(char *ss) {
  // display.clearDisplay();
  display.setTextSize(2, 2);           // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(100, 12);          // Start at top-left corner
  display.println(ss);                 // 直接用指针的形式。
  display.display();
}

void displaySD(char *ss) {
  // display.clearDisplay();
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(100, 0);           // Start at top-left corner
  display.println(ss);                 // 直接用指针的形式。
  // display.println(F("sdsds")); //用字符串
  display.display();
}

void displayfilenum() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(100, 0);
  display.print(fileNumber);
  display.print((char)247); // 度符号°
}

void displayAttitude() {

  if (!isdisplay) {
    return;
  }
  float invFreq = 1.0 / displayfreq * 1000000.0;
  unsigned long checker = micros();

  if (checker - lastdispTime < invFreq)
    return;
  lastdispTime = checker;

  // 第一行：相对滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 0, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 0);
  display.print("Relat: ");
  display.print(relativeAngle_ready, 1);
  display.print((char)247); // 度符号°

  // 第二行：滚转角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 8, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 8);
  display.print("Roll: ");
  display.print(roll_IMU, 1);
  display.print((char)247); // 度符号°

  // 第三行：俯仰角
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 16, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 16);
  display.print("Pitch:");
  display.print(pitch_IMU, 1);
  display.print((char)247);

  // 第四行：偏航角速度
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.fillRect(30, 24, 50, 8, SSD1306_BLACK);
  display.setCursor(0, 24);
  display.print("Yaw:  ");
  display.print(yaw_IMU, 1);
  display.print((char)247);

  // 图形化指示（简易人工地平仪）
  // drawArtificialHorizon(roll, pitch);

  display.display();
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

void getbarodata() {
#if defined USE_BAROMETER
  // 快速读取原始数据
  temp = bmp.readTemperature();  // °C
  pressure = bmp.readPressure(); // Pa
  // 计算海拔高度(国际标准大气模型)
  filteredAltitude = bmp.readAltitude(1013.25);
  // 输出调试信息
  Serial.printf("Temp: %.1fC | Pressure: %.2fPa | Alt: %.2fm\n", temp, pressure,
                filteredAltitude);
#endif
}

void initBAROMETER() {
#if defined USE_BAROMETER

  BMP280_I2C.begin();
  BMP280_I2C.setClock(400000); // 提升至400kHz
  if (!bmp.begin(0x76)) {      // 尝试默认地址0x76
    Serial.println("BMP280未找到，尝试0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("BMP280初始化失败!");
      // while(1);
    }
  }

  // 配置高性能模式
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,   // 连续采样
                  Adafruit_BMP280::SAMPLING_X2,   // 温度采样
                  Adafruit_BMP280::SAMPLING_X16,  // 气压采样
                  Adafruit_BMP280::FILTER_X16,    // IIR滤波
                  Adafruit_BMP280::STANDBY_MS_500 // 待机时间
  );
#endif
}

void displaythumbsup() {
  display.drawBitmap(0,       // 居中X位置
                     0,       // 居中Y位置
                     logo,    // 位图数据
                     128, 32, // 宽度和高度
                     WHITE    // 颜色
  );

  // 显示绘制的内容
  display.display();

  delay(1000);
  display.clearDisplay();
}

void sendGYROxANGLE() // 从机要做的
{

  float GyroX;
#if defined USE_MPU6050_I2C
  GyroX = -GyroX_6050; // 安装位置
#endif
#if defined USE_MPU9250_SPI
  GyroX = -GyroX_9250;
#endif

#if defined EXTIMU
  GyroX = Gyro_X_EXT;
#endif

  // 数据包结构：头(0x55) + 类型(0x71) + 15个int16_t(各2字节) + 校验和 = 15字节
  uint8_t buffer[33];
  uint8_t pos = 0;
  uint8_t checksum = 0;

#if defined BPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = relativeAngle_ready;
  RelativeAngleAll[2] = phidf;
  GYROAll[0] = GyroX;                      // 自己
  GYROAll[1] = GYRO_X_D;                   // 听来的
  GYROAll[2] = GYRO_X_F;                   // 听来的
  PHIALL[0] = roll_IMU;                    // 自己
  PHIALL[1] = phiD_raw;                    // 听来的
  PHIALL[2] = phiF_raw;                    // 听来的
  THETAALL[0] = pitch_IMU;                 // 自己
  THETAALL[1] = thetaD_raw;                // 听来的
  THETAALL[2] = thetaF_raw;                // 听来的
  ELEPWM[0] = Aele_PWM + Local_ele_ff_PWM; // 自己
  ELEPWM[1] = Dele_PWM;                    // 听来的
  ELEPWM[2] = Fele_PWM;                    // 听来的

#elif defined CPLANE
  RelativeAngleAll[0] = relativeAngle_ready;
  RelativeAngleAll[1] = phice;
  RelativeAngleAll[2] = phieg;
  GYROAll[0] = GyroX;                      // 自己
  GYROAll[1] = GYRO_X_E;                   // 听来的
  GYROAll[2] = GYRO_X_G;                   // 听来的
  PHIALL[0] = roll_IMU;                    // 自己
  PHIALL[1] = phiE_raw;                    // 听来的
  PHIALL[2] = phiG_raw;                    // 听来的
  THETAALL[0] = pitch_IMU;                 // 自己
  THETAALL[1] = thetaE_raw;                // 听来的
  THETAALL[2] = thetaG_raw;                // 听来的
  ELEPWM[0] = Aele_PWM + Local_ele_ff_PWM; // 自己
  ELEPWM[1] = Eele_PWM;                    // 听来的
  ELEPWM[2] = Gele_PWM;                    // 听来的

#elif defined DPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = relativeAngle_ready;
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = GyroX;                      // 自己的
  GYROAll[2] = GYRO_X_F;                   // 听来的
  PHIALL[0] = 0.0;                         // 自己
  PHIALL[1] = roll_IMU;                    // 自己
  PHIALL[2] = phiF_raw;                    // 自己
  THETAALL[0] = 0.0;                       // 自己
  THETAALL[1] = pitch_IMU;                 // 自己
  THETAALL[2] = thetaF_raw;                // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = Aele_PWM + Local_ele_ff_PWM; // 自己的
  ELEPWM[2] = Fele_PWM;                    // 听来的
#elif defined EPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = relativeAngle_ready;
  RelativeAngleAll[2] = phieg;
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = GyroX;                      // 自己的
  GYROAll[2] = GYRO_X_G;                   // 听来的
  PHIALL[0] = 0.0;                         // 自己
  PHIALL[1] = roll_IMU;                    // 自己
  PHIALL[2] = phiG_raw;                    // 听来的
  THETAALL[0] = 0.0;                       // 自己
  THETAALL[1] = pitch_IMU;                 // 自己
  THETAALL[2] = thetaG_raw;                // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = Aele_PWM + Local_ele_ff_PWM; // 自己的
  ELEPWM[2] = Gele_PWM;                    // 听来的
#elif defined FPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = relativeAngle_ready;
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = 0.0;                        // 空
  GYROAll[2] = GyroX;                      // 自己的
  PHIALL[0] = 0.0;                         //
  PHIALL[1] = 0.0;                         //
  PHIALL[2] = roll_IMU;                    // 自己
  THETAALL[0] = 0.0;                       //
  THETAALL[1] = 0.0;                       //
  THETAALL[2] = pitch_IMU;                 // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = 0;                           //
  ELEPWM[2] = Aele_PWM + Local_ele_ff_PWM; // 听来的
#elif defined GPLANE
  RelativeAngleAll[0] = 0.0;
  RelativeAngleAll[1] = 0.0;
  RelativeAngleAll[2] = relativeAngle_ready;
  GYROAll[0] = 0.0;                        // 空
  GYROAll[1] = 0.0;                        // 空
  GYROAll[2] = GyroX;                      // 自己的
  PHIALL[0] = 0.0;                         // 自己
  PHIALL[1] = 0.0;                         // 自己
  PHIALL[2] = roll_IMU;                    // 自己
  THETAALL[0] = 0.0;                       //
  THETAALL[1] = 0.0;                       //
  THETAALL[2] = pitch_IMU;                 // 自己
  ELEPWM[0] = 0;                           //
  ELEPWM[1] = 0;                           //
  ELEPWM[2] = Aele_PWM + Local_ele_ff_PWM; // 听来的
#endif

  float invFreq = 1.0 / Freqsendback * 1000000.0;
  unsigned long checker2 = micros();
  if (checker2 - lastsendTime < invFreq)
    return;
  lastsendTime = checker2;

  /* 如果代码没问题就改成这个
    // 应该使用unsigned long类型，避免浮点数比较：
  uint32_t interval = 1000000 / Freqsendback;  // 微秒
  if (micros() - lastsendTime < interval) return;
  lastsendTime = micros();
  */

  //  float anglerand = random(-200000, 200001) / 1000.0;  // 范围 -200.000 ~
  //  +200.000

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x71;
  checksum += 0x71; // 新类型标识

  // 1. 打包相对转角 (3个浮点数，保留1位小数)

  // 2. 打包相对转角 (转换为int16_t，保留1位小数)
  for (int i = 0; i < 3; i++) {
    int16_t val = RelativeAngleAll[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 3. 打包角速度 (同样处理)
  for (int i = 0; i < 3; i++) {
    int16_t val = GYROAll[i] * 10.0f;
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 4.打包滚转欧拉角
  for (int i = 0; i < 3; i++) {
    int16_t val = PHIALL[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 5.打包俯仰欧拉角
  for (int i = 0; i < 3; i++) {
    int16_t val = THETAALL[i] * 10.0f; // 放大10倍保留1位小数
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8);
    checksum += buffer[pos - 1];
  }

  // 5.打包升降舵pwm
  for (int i = 0; i < 3; i++) {
    int16_t val = ELEPWM[i];
    buffer[pos++] = val & 0xFF;
    checksum += buffer[pos - 1];
    buffer[pos++] = (val >> 8) & 0xFF;
    checksum += buffer[pos - 1];
  }

  // 6. 校验和
  buffer[pos] = checksum;
  // uint8_t checksum1 = 0x55 + 0x71;
  // for (int i = 2; i < 20; i++) checksum1 += buffer[i];

  // 5. 发送
  Serial6.write(buffer, sizeof(buffer));
}

void telemetry() // 主机数传
{
  // 数据包结构：头(0x55) + 类型(0x71) + 8个int16_t(各2字节) + 校验和 = 21字节
  uint8_t buffer[19];
  uint8_t pos = 0;
  uint8_t checksum = 0;

  float invFreq = 1.0 / Freqtelemetry * 1000000.0;
  unsigned long checker3 = micros();

  if (checker3 - lasttelemetryTime < invFreq)
    return;
  lasttelemetryTime = checker3;

#if defined userotatesensor
  configuration_tele[0] = relativeAngle_ready; // phiab是A机自己测的。
  configuration_tele[1] = phiac;
  configuration_tele[2] = phibd;
  configuration_tele[3] = phice;
  configuration_tele[4] = phidf;
  configuration_tele[5] = phieg;
#else
  configuration_tele[0] = phiB_raw - roll_IMU; //
  configuration_tele[1] = phiC_raw - roll_IMU; //
  configuration_tele[2] = phiD_raw - phiB_raw;
  configuration_tele[3] = phiE_raw - phiC_raw;
  configuration_tele[4] = phiF_raw - phiD_raw;
  configuration_tele[5] = phiG_raw - phiE_raw;
#endif

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
  Serial8.write(buffer, sizeof(buffer));
}

void getRotateSensor1() // 每个飞机都要做的
{

  while (Serial7.available() > 0) {
    // Serial.println("sbb");
    char incomingChar = Serial7.read();

    if (incomingChar == '\n') { // 检测到行结束符
      if (serialBuffer.startsWith("Angle:")) {
        String angleStr = serialBuffer.substring(6); // 提取 "X.YYY"
        relativeAngle_raw = angleStr.toFloat();

        // 打印到硬件串口（调试）
        // Serial.print("Received Angle: ");
        // Serial.print(relativeAngle_raw, 3);
        // Serial.println("°");
      }
      serialBuffer = "";               // 清空缓冲区
    } else if (incomingChar != '\r') { // 忽略 \r
      serialBuffer += incomingChar;
    }
  }

#if defined CPLANE || defined EPLANE || defined GPLANE
  relativeAngle_ready = (relativeAngle_raw - relativeAngle_offset);
#elif defined APLANE || defined BPLANE || defined DPLANE
  relativeAngle_ready = -(relativeAngle_raw - relativeAngle_offset);
#endif
}

void ResetRotateSensor() {
  relativeAngle_offset = relativeAngle_raw;         // 计算新偏移量
  EEPROM.put(eepromAddress1, relativeAngle_offset); // 存储到 EEPROM
  Serial.println("Zero Set! Offset: " + String(relativeAngle_offset));
}

void ProcessButtonState() {

  int buttonState = digitalRead(36);
  int buttonState1 = digitalRead(31);

  // 2. 检测按键按下（下降沿）
  if (buttonState == LOW && !buttonActive) {
    buttonActive = true;
    buttonPressTime = millis();
    delay(DEBOUNCE_DELAY); // 消抖延迟
    Serial.println("按键按下");
  }

  // 3. 检测按键释放（上升沿）
  if (buttonState == HIGH && buttonActive) {
    buttonActive = false;

    // 判断是短按还是长按释放
    if (millis() - buttonPressTime < LONG_PRESS_TIME) {
      if (millis() - buttonPressTime >= SHORT_PRESS_TIME) {
        ResetRotateSensor();
      }
      // 如果短于SHORT_PRESS_TIME，不视为有效短按
    }
    Serial.println("按键释放");
  }

  // 4. 检测长按（持续按下）
  if (buttonActive && !longPressActive &&
      (millis() - buttonPressTime >= LONG_PRESS_TIME)) {
    longPressActive = true;
    Serial.println("长按一次");
    isdisplay = !isdisplay;
  }

  // 5. 重置长按状态
  if (buttonState == HIGH && longPressActive) {
    longPressActive = false;
  }

  // 第二个按键

  // 2. 检测按键按下（下降沿）
  if (buttonState1 == LOW && !buttonActive1) {
    buttonActive1 = true;
    buttonPressTime1 = millis();
    delay(DEBOUNCE_DELAY); // 消抖延迟
    Serial.println("按键2按下");
  }

  // 3. 检测按键释放（上升沿）
  if (buttonState1 == HIGH && buttonActive1) {
    buttonActive1 = false;
    Serial.println("按键2释放");
  }

  // 4. 检测长按（持续按下）
  if (buttonActive1 && !longPressActive1 &&
      (millis() - buttonPressTime1 >= LONG_PRESS_TIME)) {
    longPressActive1 = true;
    // Serial.println("长按一次");
    //  第一行：相对滚转角
    Serial.println("3s后开始校准IMU");
    // delay(500);
    Serial.println("2s开始校准IMU");
    // delay(500);
    Serial.println("1s开始校准IMU");
    // delay(500);
    calculate_IMU_error();
    Serial.println("校准IMU完成");
    // delay(500);
  }

  // 5. 重置长按状态
  if (buttonState1 == HIGH && longPressActive1) {
    longPressActive1 = false;
  }
}

void controlFlapMotion() {
  // DESCRIPTION: Computes control commands based on state error (angle) in
  // cascaded scheme FlapMotion!
  float GyroX;
#if defined USE_MPU6050_I2C
  GyroX = -GyroX_6050; // 安装位置
#endif
#if defined USE_MPU9250_SPI
  GyroX = -GyroX_9250;
#endif

#if defined EXTIMU
  GyroX = Gyro_X_EXT;
#endif

#if defined userotatesensor
  Phiab_Mea = relativeAngle_ready; // phiab是A机自己测的。
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea = phiac;
  Phibd_Mea = phibd;
  Phice_Mea = phice;
#else
  Phiab_Mea = phiB_raw - roll_IMU; //
  // Phiab_Mea=rollAB_rad_Qua;
  Phiac_Mea = phiC_raw - roll_IMU; //
  Phibd_Mea = phiD_raw - phiB_raw;
  Phice_Mea = phiE_raw - phiC_raw;
  Phidf_Mea = phiF_raw - phiD_raw;
  Phieg_Mea = phiG_raw - phiE_raw;
#endif

  Pab = GYRO_X_B - (GyroX);
  Pac = GYRO_X_C - (GyroX);
  Pbd = GYRO_X_D - GYRO_X_B;
  Pce = GYRO_X_E - GYRO_X_C;
  Pdf = GYRO_X_F - GYRO_X_D;
  Peg = GYRO_X_G - GYRO_X_E;

  // Outer loop - P
  // float Phiab_des_ol, Phiac_des_ol;  //outerloop
  // Phiab
  error_Phiab = Phiab_des - Phiab_Mea;
  Phiab_des_ol =
      Kp_Flap * error_Phiab; //+ Ki_Phiab_angle*integral_FlapMotion_ol;// -
                             // Kd_roll_angle*derivative_roll;

  // Phiac
  error_Phiac = Phiac_des - Phiac_Mea;
  Phiac_des_ol = Kp_Flap * error_Phiac; // + Ki_pitch_angle*integral_pitch_ol;//
                                        // - Kd_pitch_angle*derivative_pitch;
  // Phibd
  error_Phibd = Phibd_des - Phibd_Mea;
  Phibd_des_ol = Kp_Flap * error_Phibd;
  // Phice
  error_Phice = Phice_des - Phice_Mea;
  Phice_des_ol = Kp_Flap * error_Phice;
  // Phidf
  error_Phidf = Phidf_des - Phidf_Mea;
  Phidf_des_ol = Kp_Flap * error_Phidf;
  // Phieg
  error_Phieg = Phieg_des - Phieg_Mea;
  Phieg_des_ol = Kp_Flap * error_Phieg;

  // Serial.println(Phibd_Mea);

  // Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  Phiab_des_ol = Kl * Phiab_des_ol;
  Phiac_des_ol = Kl * Phiac_des_ol;
  Phibd_des_ol = Kl * Phibd_des_ol;
  Phice_des_ol = Kl * Phice_des_ol;
  Phidf_des_ol = Kl * Phidf_des_ol;
  Phieg_des_ol = Kl * Phieg_des_ol;
  Phiab_des_ol = constrain(Phiab_des_ol, -240.0, 240.0);
  Phiac_des_ol = constrain(Phiac_des_ol, -240.0, 240.0);
  Phibd_des_ol = constrain(Phibd_des_ol, -240.0, 240.0);
  Phice_des_ol = constrain(Phice_des_ol, -240.0, 240.0);
  Phidf_des_ol = constrain(Phidf_des_ol, -240.0, 240.0);
  Phieg_des_ol = constrain(Phieg_des_ol, -240.0, 240.0);

  // 内环 PI
  // Phiab
  float coeffconfiguration =
      1.0 + constrain((channel_7_pwm - 1520) / 500.0, -1, 1);
  error_Phiab_RATE = Phiab_des_ol - Pab;
  integral_Phiab_RATE_il = integral_Phiab_RATE_prev_il + error_Phiab_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phiab_RATE_il = 0;
  }
  integral_Phiab_RATE_il =
      constrain(integral_Phiab_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_roll = (error_roll - error_roll_prev)/dt;
  Phiab_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phiab_des_ol + Kp_FLAP_RATE * error_Phiab_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phiab_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phiac
  error_Phiac_RATE = Phiac_des_ol - Pac;
  integral_Phiac_RATE_il = integral_Phiac_RATE_prev_il + error_Phiac_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phiac_RATE_il = 0;
  }
  integral_Phiac_RATE_il =
      constrain(integral_Phiac_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phiac_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phiac_des_ol + Kp_FLAP_RATE * error_Phiac_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phiac_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phibd
  error_Phibd_RATE = Phibd_des_ol - Pbd;
  integral_Phibd_RATE_il = integral_Phibd_RATE_prev_il + error_Phibd_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phibd_RATE_il = 0;
  }
  integral_Phibd_RATE_il =
      constrain(integral_Phibd_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phibd_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phibd_des_ol + Kp_FLAP_RATE * error_Phibd_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phibd_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phice
  error_Phice_RATE = Phice_des_ol - Pce;
  integral_Phice_RATE_il = integral_Phice_RATE_prev_il + error_Phice_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phice_RATE_il = 0;
  }
  integral_Phice_RATE_il =
      constrain(integral_Phice_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phice_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phice_des_ol + Kp_FLAP_RATE * error_Phice_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phice_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phidf
  error_Phidf_RATE = Phidf_des_ol - Pdf;
  integral_Phidf_RATE_il = integral_Phidf_RATE_prev_il + error_Phidf_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phice_RATE_il = 0;
  }
  integral_Phidf_RATE_il =
      constrain(integral_Phidf_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phidf_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phidf_des_ol + Kp_FLAP_RATE * error_Phidf_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phidf_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Phieg
  error_Phieg_RATE = Phieg_des_ol - Peg;
  integral_Phieg_RATE_il = integral_Phieg_RATE_prev_il + error_Phieg_RATE * dt;
  if (channel_1_pwm <
      1060) { // Don't let integrator build if throttle is too low
    integral_Phieg_RATE_il = 0;
  }
  integral_Phieg_RATE_il =
      constrain(integral_Phieg_RATE_il, -i_limit,
                i_limit); // Saturate integrator to prevent unsafe buildup
  // derivative_pitch = (error_pitch - error_pitch_prev)/dt;
  Phieg_PID = coeffconfiguration * .01 *
              (Kff_FLAP_RATE * Phieg_des_ol + Kp_FLAP_RATE * error_Phieg_RATE +
               i_valid * Ki_FLAP_RATE *
                   integral_Phieg_RATE_il); // Scaled by .01 to bring within -1
                                            // to 1 range

  // Update Flap variables
  Phiab_des_prev = Phiab_des_ol;
  Phiac_des_prev = Phiac_des_ol;
  Phibd_des_prev = Phibd_des_ol;
  Phice_des_prev = Phice_des_ol;
  Phidf_des_prev = Phidf_des_ol;
  Phieg_des_prev = Phieg_des_ol;
  integral_Phiab_RATE_prev_il = integral_Phiab_RATE_il;
  integral_Phiac_RATE_prev_il = integral_Phiac_RATE_il;
  integral_Phibd_RATE_prev_il = integral_Phibd_RATE_il;
  integral_Phice_RATE_prev_il = integral_Phice_RATE_il;
  integral_Phidf_RATE_prev_il = integral_Phidf_RATE_il;
  integral_Phieg_RATE_prev_il = integral_Phieg_RATE_il;
  relativeAngle_ready_prev = relativeAngle_ready;
}

void sendAllDataleft(int servoCommandsleft[12], float pitchAnglesleft[3],
                     int elePwmLeft[3], int eleFFPwmLeft[3],
                     bool lightSignalsleft[3], bool int_is_valid) {
  float invFreq2 = 1.0 / transfreq * 1000000.0;
  unsigned long checker2 = micros();
#if defined APLANE
  force_manual = (currentMode == MANUAL_MODE);
#else
  force_manual = recvData.Force_manual;
#endif

  if (checker2 - lasttransTime < invFreq2)
    return;
  lasttransTime = checker2;
  // 固定大小缓冲区：2头字节 + 1空速有效bool + 1强制手动bool + 12舵机×2 +
  // 3角度×2 + 3升降舵PWM×2 + 3升降舵前馈PWM×2 + 3灯光×1 + 1校验和 = 50字节
  uint8_t buffer[50];
  uint8_t checksum = 0;
  int pos = 0;

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x60;
  checksum += 0x60;

  // 1.5 空速有效
  buffer[pos++] = int_is_valid ? 0x01 : 0x00; // NEW: 添加这行
  checksum += buffer[pos - 1];                // NEW: 添加这行

  // 1.6 强制手动
  buffer[pos++] = force_manual ? 0x01 : 0x00;
  checksum += buffer[pos - 1];

  // 2. 舵机指令(12个) - 直接写入高低字节
  for (int i = 0; i < 12; i++) {
    buffer[pos++] = servoCommandsleft[i] & 0xFF;        // 低字节
    buffer[pos++] = (servoCommandsleft[i] >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 3. 俯仰角(3个) - 直接写入高低字节
  for (int i = 0; i < 3; i++) {
    int16_t pitchAnglesleft_int = pitchAnglesleft[i] * 10.0f;
    buffer[pos++] = pitchAnglesleft_int & 0xFF;        // 低字节
    buffer[pos++] = (pitchAnglesleft_int >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 4. 升降舵PWM(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = elePwmLeft[i] & 0xFF;
    buffer[pos++] = (elePwmLeft[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 5. 升降舵前馈PWM(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = eleFFPwmLeft[i] & 0xFF;
    buffer[pos++] = (eleFFPwmLeft[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 6. 灯光信号(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = lightSignalsleft[i] ? 0x01 : 0x00;
    checksum += buffer[pos - 1];
  }

  // 7. 校验和
  buffer[pos] = checksum;
  // 8. 一次性发送
  Serial3.write(buffer, sizeof(buffer));
}

void sendAllDataright(int servoCommandsright[12], float pitchAnglesright[3],
                      int elePwmRight[3], int eleFFPwmRight[3],
                      bool lightSignalsright[3], bool int_is_valid) {
  float invFreq3 = 1.0 / transfreq * 1000000.0;
  unsigned long checker3 = micros();
#if defined APLANE
  force_manual = (currentMode == MANUAL_MODE);
#else
  force_manual = recvData.Force_manual;
#endif

  if (checker3 - lasttransTime3 < invFreq3)
    return;
  lasttransTime3 = checker3;

  // 固定大小缓冲区：2头字节 + 1空速有效bool + 1强制手动bool + 12舵机×2 +
  // 3角度×2 + 3升降舵PWM×2 + 3升降舵前馈PWM×2 + 3灯光×1 + 1校验和 = 50字节
  uint8_t buffer[50];
  uint8_t checksum = 0;
  int pos = 0;

  // 1. 数据头
  buffer[pos++] = 0x55;
  checksum += 0x55;
  buffer[pos++] = 0x60;
  checksum += 0x60;

  // 1.5 空速有效
  buffer[pos++] = int_is_valid ? 0x01 : 0x00; // NEW: 添加这行
  checksum += buffer[pos - 1];                // NEW: 添加这行

  // 1.6 强制手动
  buffer[pos++] = force_manual ? 0x01 : 0x00;
  checksum += buffer[pos - 1];

  // 2. 舵机指令(12个) - 直接写入高低字节
  for (int i = 0; i < 12; i++) {
    buffer[pos++] = servoCommandsright[i] & 0xFF;        // 低字节
    buffer[pos++] = (servoCommandsright[i] >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 3. 俯仰角(3个) - 直接写入高低字节
  for (int i = 0; i < 3; i++) {
    int16_t pitchAnglesright_int = pitchAnglesright[i] * 10.0f;
    buffer[pos++] = pitchAnglesright_int & 0xFF;        // 低字节
    buffer[pos++] = (pitchAnglesright_int >> 8) & 0xFF; // 高字节
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 4. 升降舵PWM(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = elePwmRight[i] & 0xFF;
    buffer[pos++] = (elePwmRight[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 5. 升降舵前馈PWM(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = eleFFPwmRight[i] & 0xFF;
    buffer[pos++] = (eleFFPwmRight[i] >> 8) & 0xFF;
    checksum += buffer[pos - 2] + buffer[pos - 1];
  }

  // 6. 灯光信号(3个)
  for (int i = 0; i < 3; i++) {
    buffer[pos++] = lightSignalsright[i] ? 0x01 : 0x00;
    checksum += buffer[pos - 1];
  }

  // 7. 校验和
  buffer[pos] = checksum;
  // 8. 一次性发送
  Serial5.write(buffer, sizeof(buffer));
}

// 接受指令数据
void receiveCommandData() {

  static uint8_t buffer[50];
  static uint8_t pos = 0;

  while (Serial6.available()) {
    uint8_t byte = Serial6.read();
    // Serial.println("ss");
    //  检查数据头
    if (pos == 0 && byte != 0x55)
      continue;
    if (pos == 1 && byte != 0x60) {
      pos = 0;
      continue;
    }

    buffer[pos++] = byte;

    // 完整数据包接收
    if (pos == 50) {
      pos = 0;
      // Serial.println("ok");
      //  计算校验和 (0x55 + 0x60 + 数据字节)
      uint8_t checksum = 0x55 + 0x60;
      for (int i = 2; i < 49; i++) {
        checksum += buffer[i];
      }
      // Serial.print(checksum);
      // Serial.print(" ");
      // Serial.println(buffer[49]);
      //  验证校验和
      if (checksum == buffer[49]) {
        int bufPos = 2; // 数据起始位置

        // 新增：提取int_is_valid字段  // NEW: 添加这行
        recvData.int_is_valid = (buffer[bufPos] == 0x01); // NEW: 添加这行
        bufPos += 1;                                      // NEW: 添加这行
        recvData.Force_manual = (buffer[bufPos] == 0x01);
        bufPos += 1;

        // 1. 提取舵机指令 (12个int16_t)
        for (int i = 0; i < 12; i++) {
          recvData.servo[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 2. 提取俯仰角 (3个int16_t) 前面的(int16_t)不能少，另外合适的缩放系数
        // 最大的数字不能超过37268
        for (int i = 0; i < 3; i++) {
          recvData.pitch[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }

        // 3. 提取升降舵PWM (3个int16_t)
        for (int i = 0; i < 3; i++) {
          recvData.ele_pwm[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 4. 提取升降舵前馈PWM (3个int16_t)
        for (int i = 0; i < 3; i++) {
          recvData.ele_ff_pwm[i] =
              (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        // 5. 提取灯光信号 (3个bool)
        for (int i = 0; i < 3; i++) {
          recvData.lights[i] = (buffer[bufPos] == 0x01);
          bufPos += 1;
        }

// 拿出自己的
#if defined BPLANE || defined CPLANE
        Local_ail1_PWM = recvData.servo[0];
        Local_ail2_PWM = recvData.servo[1];
        Local_thro_PWM = recvData.servo[2];
        Local_rudd_PWM = recvData.servo[3];
        Local_pitch_des = recvData.pitch[0];
        Local_ele_PWM = recvData.ele_pwm[0];
        Local_ele_ff_PWM = recvData.ele_ff_pwm[0];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
#elif defined DPLANE || defined EPLANE
        Local_ail1_PWM = recvData.servo[4];
        Local_ail2_PWM = recvData.servo[5];
        Local_thro_PWM = recvData.servo[6];
        Local_rudd_PWM = recvData.servo[7];
        Local_pitch_des = recvData.pitch[1];
        Local_ele_PWM = recvData.ele_pwm[1];
        Local_ele_ff_PWM = recvData.ele_ff_pwm[1];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
#elif defined FPLANE || defined GPLANE
        Local_ail1_PWM = recvData.servo[8];
        Local_ail2_PWM = recvData.servo[9];
        Local_thro_PWM = recvData.servo[10];
        Local_rudd_PWM = recvData.servo[11];
        Local_pitch_des = recvData.pitch[2];
        Local_ele_PWM = recvData.ele_pwm[2];
        Local_ele_ff_PWM = recvData.ele_ff_pwm[2];
        int_is_valid = recvData.int_is_valid;
        force_manual = recvData.Force_manual;
#endif

        printReceivedData();
      }
    }
  }
}

void printReceivedData() {
  static uint32_t frameCount = 0;
  Serial.printf("\n=== 帧#%d ===\n", ++frameCount);
  Serial.printf("int_valid:%d force_manual:%d\n", recvData.int_is_valid ? 1 : 0,
                recvData.Force_manual ? 1 : 0);

  // 舵机指令
  Serial.print("舵机: ");
  for (int i = 0; i < 12; i++)
    Serial.printf("%d ", recvData.servo[i]);

  // 俯仰角
  Serial.print("\n俯仰: ");
  for (int i = 0; i < 3; i++)
    Serial.printf("%.3f ", recvData.pitch[i]);

  // 升降舵PWM
  Serial.print("\nELEPWM: ");
  for (int i = 0; i < 3; i++)
    Serial.printf("%d ", recvData.ele_pwm[i]);

  Serial.print("\nELEFFPWM: ");
  for (int i = 0; i < 3; i++)
    Serial.printf("%d ", recvData.ele_ff_pwm[i]);

  // 灯光
  Serial.print("\n灯光: ");
  for (int i = 0; i < 3; i++)
    Serial.print(recvData.lights[i] ? "1 " : "0 ");

  Serial.println("\n=============");
}

void getGYROxANGLEleft() {
  static uint8_t buffer[33];
  static uint8_t pos = 0;

  float relativeangleleft[3];
  float pleft[3];
  float phi_raw[3];
  float theta_raw[3];
  float ele_pwm[3];
  while (Serial3.available()) {
    uint8_t byte = Serial3.read();

    // 检查数据头
    if (pos == 0 && byte != 0x55)
      continue;
    if (pos == 1 && byte != 0x71) {
      pos = 0;
      continue;
    }

    buffer[pos++] = byte;

    // 完整数据包处理
    if (pos == 33) {
      pos = 0;

      // 计算校验和
      uint8_t checksum = 0x55 + 0x71;
      for (int i = 2; i < 32; i++) {
        checksum += buffer[i];
      }

      if (checksum == buffer[32]) {
        int bufPos = 2;
        // Serial.println("ok");
        //  提取角度数据
        for (int i = 0; i < 3; i++) {
          relativeangleleft[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }

        // 提取角速度数据
        for (int i = 0; i < 3; i++) {
          pleft[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }
        // 提取当地欧拉角度数据
        for (int i = 0; i < 3; i++) {
          phi_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;
          bufPos += 2;
        }
        for (int i = 0; i < 3; i++) {
          theta_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        // 提取舵机指令 (3个int16_t)
        for (int i = 0; i < 3; i++) {
          ele_pwm[i] = (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        phibd = relativeangleleft[1];
        phidf = relativeangleleft[2];
        GYRO_X_B = pleft[0];
        GYRO_X_D = pleft[1];
        GYRO_X_F = pleft[2];
        phiB_raw = phi_raw[0];
        phiD_raw = phi_raw[1];
        phiF_raw = phi_raw[2];
        thetaB_raw = theta_raw[0];
        thetaD_raw = theta_raw[1];
        thetaF_raw = theta_raw[2];
        Bele_PWM = ele_pwm[0];
        Dele_PWM = ele_pwm[1];
        Fele_PWM = ele_pwm[2];
      }
    }
  }
}

void getGYROxANGLEright() {
  static uint8_t buffer[33]; // 2+1+6+6+6+6+6;
  static uint8_t pos = 0;

  float relativeangleright[3];
  float pright[3];
  float phi_raw[3];
  float theta_raw[3];
  float ele_pwm[3];
  while (Serial5.available()) {
    uint8_t byte = Serial5.read();

    // 检查数据头
    if (pos == 0 && byte != 0x55)
      continue;
    if (pos == 1 && byte != 0x71) {
      pos = 0;
      continue;
    }

    buffer[pos++] = byte;

    // 完整数据包处理
    if (pos == 33) {
      pos = 0;

      // 计算校验和
      uint8_t checksum = 0x55 + 0x71;
      for (int i = 2; i < 32; i++) {
        checksum += buffer[i];
      }

      if (checksum == buffer[32]) {
        int bufPos = 2;

        // 提取角度数据
        for (int i = 0; i < 3; i++) {
          relativeangleright[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        // 提取角速度数据
        for (int i = 0; i < 3; i++) {
          pright[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }
        // 提取当地欧拉角度数据
        for (int i = 0; i < 3; i++) {
          phi_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        for (int i = 0; i < 3; i++) {
          theta_raw[i] =
              ((int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos]) / 10.0f;

          bufPos += 2;
        }

        // 提取舵机指令 (3个int16_t)
        for (int i = 0; i < 3; i++) {
          ele_pwm[i] = (int16_t)(buffer[bufPos + 1] << 8) | buffer[bufPos];
          bufPos += 2;
        }

        phiac = relativeangleright[0];
        phice = relativeangleright[1];
        phieg = relativeangleright[2];
        GYRO_X_C = pright[0];
        GYRO_X_E = pright[1];
        GYRO_X_G = pright[2];
        phiC_raw = phi_raw[0];
        phiE_raw = phi_raw[1];
        phiG_raw = phi_raw[2];
        thetaC_raw = theta_raw[0];
        thetaE_raw = theta_raw[1];
        thetaG_raw = theta_raw[2];
        Cele_PWM = ele_pwm[0];
        Eele_PWM = ele_pwm[1];
        Gele_PWM = ele_pwm[2];
      }
    }
  }
}

/*
void getQuaternion() {
  //Serial.println("ss");
  static String buffer;
  while (Serial3.available() > 0) {
    //Serial.println("ssb");
    char c = Serial3.read();
    if (c == '\n') {
      // 示例：解析"0.707107,0.707107,0.000000,0.000000"
      float w, x, y, z;
      if (sscanf(buffer.c_str(), "%f,%f,%f,%f", &q0B, &q1B, &q2B, &q3B) == 4) {
        // 成功解析到四元数
        //Serial.println(w);
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }

  Eigen::Quaternionf qA;
  qA.w() = q0;  // cos(90°/2) = √2/2 ≈ 0.7071
  qA.x() = q1;  // sin(90°/2) = √2/2 ≈ 0.7071
  qA.y() = q2;
  qA.z() = q3;

  // 示例：q2绕X轴旋转90度（roll=90°）
  Eigen::Quaternionf qB;
  qB.w() = q0B;  // cos(90°/2) = √2/2 ≈ 0.7071
  qB.x() = q1B;  // sin(90°/2) = √2/2 ≈ 0.7071
  qB.y() = q2B;
  qB.z() = q3B;

  // 计算姿态差欧拉角

  quatDiffToEuler(qA, qB, rollAB_rad_Qua, pitchAB_rad_Qua, yawAB_rad_Qua);

  // 转换为角度输出
  //Serial.println("姿态差欧拉角 (Z-Y-X顺序):");
  //Serial.print("Roll (X): ");  Serial.print((rollAB_rad_Qua));
Serial.println("°");
  //Serial.print("Pitch (Y): "); Serial.print((pitchAB_rad_Qua));
Serial.println("°");
  //Serial.print("Yaw (Z): ");   Serial.print((yawAB_rad_Qua));
Serial.println("°");
}

void setQuaternion() {
  float invFreqQua = 1.0 / 100 * 1000000.0;
  unsigned long checkerQua = micros();

  if (checkerQua - lastsendTimeQua < invFreqQua) return;
  lastsendTimeQua = checkerQua;

  //  float anglerand = random(-200000, 200001) / 1000.0;  // 范围 -200.000 ~
+200.000

  char bufferQua[64];
  snprintf(bufferQua, sizeof(bufferQua),
           "%.4f,%.4f,%.4f,%.4f\n",
           q0,
           q1,
           q2,
           q3);
  Serial3.print(bufferQua);
  //Serial.println(bufferQua);
}

*/

void quatDiffToEuler(const Eigen::Quaternionf &q1, const Eigen::Quaternionf &q2,
                     float &roll, float &pitch, float &yaw) {
  // 计算相对旋转四元数: q_diff = q2 * q1.conjugate()
  // Eigen::Quaternionf q_diff =  q1.conjugate()*q2;
  // float recipN = invSqrt(q_diff.w() * q_diff.w() + q_diff.x() * q_diff.x() +
  // q_diff.y() * q_diff.y() + q_diff.z() * q_diff.z()); //normalise step
  // magnitude

  // Eigen::Matrix3f RA = q1.toRotationMatrix();
  // Eigen::Matrix3f RB = q2.toRotationMatrix();
  // Eigen::Matrix3f R;
  // R=RA*RB.transpose();

  yaw_IMU_B = atan2(q2.x() * q2.y() + q2.w() * q2.z(),
                    0.5f - q2.y() * q2.y() - q2.z() * q2.z()) *
              57.29577951; // degrees
  roll_IMU_B = 57.3 * atan2(2.0f * (q2.w() * q2.x() + q2.y() * q2.z()),
                            1.0f - 2.0f * (q2.x() * q2.x() + q2.y() * q2.y()));
  roll_diff = -roll_IMU_B - roll_IMU;

  Eigen::Quaternionf q_rot = eulertoqua(0, 0, (yaw_IMU - yaw_IMU_B) / 57.295);
  Eigen::Quaternionf q2_revised = q_rot * q2;
  Eigen::Quaternionf q_diff = q2_revised * q1.conjugate();
  q_diff = q1.conjugate() * q_diff * q1; // 投影到q1上。

  // Serial.println(String(q1.w())+" "+String(q1.x())+" "+String(q1.y())+"
  // "+String(q1.z())+"sssss"+String(q2.w())+" "+String(q2.x())+"
  // "+String(q2.y())+" "+String(q2.z())); Serial.print(yaw_IMU); Serial.print("
  // "); Serial.println(yaw_IMU_B);

  // 四元数转欧拉角（Z-Y-X顺序）
  float w = q_diff.w(), x = q_diff.x(), y = q_diff.y(), z = q_diff.z();
  float w2 = q2.w(), x2 = q2.x(), y2 = q2.y(), z2 = q2.z();
  float w1 = q1.w(), x1 = q1.x(), y1 = q1.y(), z1 = q1.z();
  // roll  = 57.3*atan2(2.0f * (w*x ), 1.0f - 2.0f * (x*x ));

  // roll_IMU = -atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951;
  // //degrees 参考

  // 计算欧拉角（弧度）

  // roll  = 57.3*atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
  float N = invSqrt(w * w + x * x);
  w *= N;
  x *= N;
  roll = -57.3 * atan2(2.0f * (w * x), 1.0f - 2.0f * (x * x)); // 加个负号
  pitch = 57.3 * asin(2.0f * (w * y - z * x));
  yaw = 57.3 * atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

  // Serial.print(roll);
  // Serial.print("   ");
  // Serial.print(relativeAngle_ready);
  // Serial.print("   ");
  //  Serial.println(roll_diff);

  // printFullMatrix(R);
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

void getIMUdata_EXT() {
  static uint8_t buf[11], pos = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    // 检查数据头
    if (pos == 0 && b != 0x55)
      continue;
    if (pos == 1 && b != 0x52 && b != 0x59 && b != 0x51 && b != 0x53) {
      pos = 0;
      continue;
    }

    buf[pos++] = b;

    // 完整数据包处理
    if (pos == 11) {
      pos = 0;

      // 计算校验和
      uint8_t sum = buf[0] + buf[1];
      for (int i = 2; i < 10; i++)
        sum += buf[i];
      if (sum == buf[10]) {

        if (buf[1] == 0x59) { // 四元数数据
          float q0 = ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f;
          float q1 = ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f;
          float q2 = ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f;
          float q3 = ((int16_t)(buf[9] << 8) | buf[8]) / 32768.0f;
          // Serial.print("Q:"); Serial.print(q0,3); Serial.print(",");
          // Serial.print(q1,3); Serial.print(","); Serial.print(q2,3);
          // Serial.print(","); Serial.println(q3,3);
          roll_IMU_EXT = -atan2(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2) *
                         57.29577951; // degrees
          pitch_IMU_EXT = asin(constrain(-2.0f * (q1 * q3 - q0 * q2), -0.999999,
                                         0.999999)) *
                          57.29577951; // degrees
          yaw_IMU_EXT = atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) *
                        57.29577951; // degrees
          // Serial.print("R:");
          // Serial.println(roll_IMU_EXT);
        } else if (buf[1] == 0x52) { // 角速度数据
          Gyro_X_EXT =
              -1 * (((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 2000.0f);
          Gyro_Y_EXT = ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 2000.0f;
          Gyro_Z_EXT = ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 2000.0f;
          // Serial.print("X:");
          // Serial.println(Gyro_X_EXT);
          // Serial.print(wy,1); Serial.print(","); Serial.println(wz,1);
        }

        else if (buf[1] == 0x51) { // 加速度数据
          Acc_X_EXT =
              ((int16_t)(buf[3] << 8) | buf[2]) / 32768.0f * 16.0f * 9.8f;
          Acc_Y_EXT =
              ((int16_t)(buf[5] << 8) | buf[4]) / 32768.0f * 16.0f * 9.8f;
          Acc_Z_EXT =
              ((int16_t)(buf[7] << 8) | buf[6]) / 32768.0f * 16.0f * 9.8f;
          // Serial.print("A:");
          // Serial.println(Acc_X_EXT);
          // Serial.print("A:"); Serial.print(ax,1); Serial.print(",");
          // Serial.print(ay,1); Serial.print(","); Serial.println(az,1);
        }
      }
    }
  }
}

float keeppositive(float command) {
  if (command > 0.0)
    return command;
  else
    return 0.0;
}

// 应变传感器
// 进制转换
int32_t parseChannel(uint8_t *buf, int index) {
  // index: 第几个通道（0~4）

  int offset = 3 + index * 4;

  union {
    int32_t val;
    uint8_t b[4];
  } u;

  // 按手册：低位在前，高位在后
  u.b[0] = buf[offset + 1];
  u.b[1] = buf[offset + 0];
  u.b[2] = buf[offset + 3];
  u.b[3] = buf[offset + 2];

  return u.val;
}

// ===== CRC16 (Modbus) =====
uint16_t modbusCRC(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (int i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ===== 发送03读寄存器 =====
void sendStrainCommand(uint8_t addr, uint16_t fcode, uint16_t reg,
                       uint16_t num) {

  uint8_t frame[8];

  frame[0] = addr;
  frame[1] = fcode;
  frame[2] = reg >> 8;
  frame[3] = reg & 0xFF;
  frame[4] = num >> 8;
  frame[5] = num & 0xFF;

  uint16_t crc = modbusCRC(frame, 6);

  frame[6] = crc & 0xFF; // CRC低位
  frame[7] = crc >> 8;   // CRC高位

  // Serial.write(frame, 8);
  Serial7.write(frame, 8);
}

void Strain_read_all() {
  float invFreq = 1.0 / read_Strain_freq * 1000000.0;
  unsigned long checker = micros();

  if (checker - last_read_Strain_Time < invFreq)
    return;
  last_read_Strain_Time = checker;
  // 读取寄存器0x0000，1个寄存器
  sendStrainCommand(1, 0x03, 0x0000, 10);
  uint8_t buf[32];
  int len = 0;

  while (Serial7.available()) {
    // Serial.println("available ");
    buf[len++] = Serial7.read();
  }

  // 解析
  if (len >= 25 && buf[1] == 0x03) {

    Strain_value1 = parseChannel(buf, 0);
    Strain_value2 = parseChannel(buf, 1);
    Strain_value3 = parseChannel(buf, 2);
    Strain_value4 = parseChannel(buf, 3);
    Strain_value5 = parseChannel(buf, 4);
  }
}

void getICM42688data() {
  // 1. 强制切换到 Bank 0
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_BANK_SEL);
  Wire.write(0x00);
  Wire.endTransmission();

  // 2. 读取 12 字节数据 (6字节加速 + 6字节陀螺仪) [cite: 1514]
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(REG_ICM_ACC_DATA);
  if (Wire.endTransmission(false) != 0)
    return; // 通讯失败直接退出

  if (Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)12) == 12) {
    // ICM-42688 为 Big-Endian (高位在前) [cite: 1810]
    int16_t raw_ax = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_ay = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_az = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gx = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gy = (int16_t)(Wire.read() << 8 | Wire.read());
    int16_t raw_gz = (int16_t)(Wire.read() << 8 | Wire.read());

    // 3. 映射到你原有的 6050 变量中 (单位转换)
    // 默认量程: Accel ±16g (2048 LSB/g), Gyro ±2000dps (16.4 LSB/dps) [cite:
    // 168, 188]
    AccX_6050 = raw_ax / 2048.0;
    AccY_6050 = raw_ay / 2048.0;
    AccZ_6050 = raw_az / 2048.0;

    GyroX_6050 = raw_gx / 16.4;
    GyroY_6050 = raw_gy / 16.4;
    GyroZ_6050 = raw_gz / 16.4;

    // 4. 应用低通滤波 (保留你原有的 B_accel_6050 逻辑)
    /*
    AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 *
    AccX_6050; AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050
    * AccY_6050; AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 +
    B_accel_6050 * AccZ_6050; AccX_prev_6050 = AccX_6050; AccY_prev_6050 =
    AccY_6050; AccZ_prev_6050 = AccZ_6050;

    GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 *
    GyroX_6050; GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050
    * GyroY_6050; GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 +
    B_gyro_6050 * GyroZ_6050; GyroX_prev_6050 = GyroX_6050; GyroY_prev_6050 =
    GyroY_6050; GyroZ_prev_6050 = GyroZ_6050;
    */
  }
}

void getBMI088data() {
  // Serial.println("sssss");
  uint8_t buf[6];
  const float inv_acc = 1.0f / 5460.0f; // ±6g
  const float inv_gyr = 1.0f / 16.384f; // ±2000dps
  static unsigned long lastBMI088AccDebug = 0;
  static unsigned long lastBMI088GyrDebug = 0;

  // A. 读取加速度计 (注意 Dummy Byte)
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(0x12 | SPI_READ);
  SPI.transfer(0x00); // 重要：哑字节
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_ACC, HIGH);
  SPI.endTransaction();

  int16_t rx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t ry = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rz = (int16_t)(buf[4] | (buf[5] << 8));

  // B. 读取陀螺仪
  SPI.beginTransaction(bmiSettings);
  digitalWrite(CS_GYR, LOW);
  SPI.transfer(0x02 | SPI_READ);
  for (int i = 0; i < 6; i++)
    buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_GYR, HIGH);
  SPI.endTransaction();

  int16_t rgx = (int16_t)(buf[0] | (buf[1] << 8));
  int16_t rgy = (int16_t)(buf[2] | (buf[3] << 8));
  int16_t rgz = (int16_t)(buf[4] | (buf[5] << 8));

  // C. 坐标轴映射 (保持你之前的 90 度旋转逻辑)
  // 原逻辑: 新X=旧Y, 新Y=-旧X, Z不变
  AccX_6050 = (ry * inv_acc) - calAccGyroData.AccErrorX_6050;
  AccY_6050 = (-rx * inv_acc) - calAccGyroData.AccErrorY_6050;
  AccZ_6050 = (rz * inv_acc) - calAccGyroData.AccErrorZ_6050;

  GyroX_6050 = (rgy * inv_gyr) - calAccGyroData.GyroErrorX_6050;
  GyroY_6050 = (-rgx * inv_gyr) - calAccGyroData.GyroErrorY_6050;
  GyroZ_6050 = (rgz * inv_gyr) - calAccGyroData.GyroErrorZ_6050;

  // --- C. 应用低通滤波 (保留你原有的 B_accel_6050 参数逻辑) ---
  /*
  AccX_6050 = (1.0 - B_accel_6050) * AccX_prev_6050 + B_accel_6050 * AccX_6050;
  AccY_6050 = (1.0 - B_accel_6050) * AccY_prev_6050 + B_accel_6050 * AccY_6050;
  AccZ_6050 = (1.0 - B_accel_6050) * AccZ_prev_6050 + B_accel_6050 * AccZ_6050;
  AccX_prev_6050 = AccX_6050; AccY_prev_6050 = AccY_6050; AccZ_prev_6050 =
  AccZ_6050;

  GyroX_6050 = (1.0 - B_gyro_6050) * GyroX_prev_6050 + B_gyro_6050 * GyroX_6050;
  GyroY_6050 = (1.0 - B_gyro_6050) * GyroY_prev_6050 + B_gyro_6050 * GyroY_6050;
  GyroZ_6050 = (1.0 - B_gyro_6050) * GyroZ_prev_6050 + B_gyro_6050 * GyroZ_6050;
  GyroX_prev_6050 = GyroX_6050; GyroY_prev_6050 = GyroY_6050; GyroZ_prev_6050 =
  GyroZ_6050;
  */
}

void printBMI088Raw(const char *tag, const uint8_t *buf, int len, int16_t x,
                    int16_t y, int16_t z) {
  Serial.print(tag);
  Serial.print(F(" raw:"));
  for (int i = 0; i < len; i++) {
    if (buf[i] < 0x10)
      Serial.print('0');
    Serial.print(buf[i], HEX);
    if (i < len - 1)
      Serial.print(' ');
  }
  Serial.print(F(" val:"));
  Serial.print(x);
  Serial.print(F(","));
  Serial.print(y);
  Serial.print(F(","));
  Serial.println(z);
}

// 辅助函数：SPI 写寄存器
void writeRegSPI(int cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(bmiSettings);
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

// 通用 I2C 寄存器写入辅助函数
void writeReg(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  bus.endTransmission();
}

void getairdata() {
  while (Serial8.available() >= sizeof(FC_Binary_Packet)) {

    // 检查第一个帧头
    if (Serial8.read() == 0xAA) {
      // 检查第二个帧头
      if (Serial8.peek() == 0x55) {
        FC_Binary_Packet candidate = {};
        uint8_t *ptr = reinterpret_cast<uint8_t *>(&candidate);

        Serial8.read(); // 跳过第二个帧头
        ptr[0] = 0xAA;
        ptr[1] = 0x55;

        const size_t payloadLen = sizeof(FC_Binary_Packet) - 2;
        if (Serial8.readBytes(&ptr[2], payloadLen) != payloadLen) {
          continue;
        }

        // 验证校验和
        uint8_t calcSum = 0;
        for (int i = 2; i < (sizeof(FC_Binary_Packet) - 1); i++) {
          calcSum += ptr[i];
        }

        if (calcSum == candidate.checksum) {
          if (!(candidate.tas >= 0.0f && candidate.tas < 200.0f)) {
            candidate.tas = 0.0f;
          }
          airdata = candidate;
          Serial.println(airdata.aoa);
        }
      }
    }
  }
}
