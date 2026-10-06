#pragma once

#include <Arduino.h>

// 从相邻机体接收的姿态、舵机及期望状态，供控制器和日志读取。
extern float rollAB_rad_Qua;
extern float Local_pitch_des;
extern int Local_ail1_PWM, Local_ail2_PWM, Local_thro_PWM, Local_rudd_PWM,
    Local_ele_PWM, Local_ele_ff_PWM;

extern float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw, phiG_raw;
extern float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw,
    thetaG_raw;
extern int Bele_PWM, Cele_PWM, Dele_PWM, Eele_PWM, Fele_PWM, Gele_PWM;
extern float GYRO_X_B, GYRO_X_C, GYRO_X_D, GYRO_X_E, GYRO_X_F, GYRO_X_G;

void beginParentLink();
void beginChildLinks();
void sendGYROxANGLE();
void receiveCommandData();
void getGYROxANGLEleft();
void getGYROxANGLEright();
// index=0/1/2 对应同侧由近到远的三架子机；舵机指令单位为 PWM 微秒数。
void setLeftChildCommand(unsigned int index, int aileron1, int aileron2,
                         int throttle, int rudder, float pitch,
                         int elevatorManual, int elevatorFeedForward);
void setRightChildCommand(unsigned int index, int aileron1, int aileron2,
                          int throttle, int rudder, float pitch,
                          int elevatorManual, int elevatorFeedForward);
void sendPreparedChildCommands(bool intIsValid);
void forwardReceivedChildCommands(bool intIsValid);
void receiveAdjacentAircraftStates();
