#pragma once

// Used by USB configuration to recognize a standalone bench-powered slave.
bool hasReceivedParentCommand();

#include <Arduino.h>

// 从相邻机体接收的姿态、舵机及期望状态，供控制器和日志读取。
extern float rollAB_rad_Qua;
extern float Local_pitch_des;
// 本机有符号控制偏移（μs）：A 机每周期计算，B～G 由上级接收。
// 在本机叠加补偿并应用本机 rev/trim。
extern int Local_ail1_control_us, Local_ail2_control_us, Local_thro_control_us, Local_rudd_control_us,
    Local_ele_control_us, Local_ele_ff_control_us;

extern float phiB_raw, phiC_raw, phiD_raw, phiE_raw, phiF_raw, phiG_raw;
extern float thetaB_raw, thetaC_raw, thetaD_raw, thetaE_raw, thetaF_raw,
    thetaG_raw;
extern int B_ele_PWM, C_ele_PWM, D_ele_PWM, E_ele_PWM, F_ele_PWM, G_ele_PWM;
extern float GYRO_X_B, GYRO_X_C, GYRO_X_D, GYRO_X_E, GYRO_X_F, GYRO_X_G;

void beginParentLink();
void beginChildLinks();
void sendGYROxANGLE();
void receiveCommandData();
void getGYROxANGLEleft();
void getGYROxANGLEright();
// index=0/1/2 对应同侧由近到远的三架子机；控制偏移单位 μs，不含中位/rev/trim。
void setLeftChildCommand(unsigned int index, int aileron1, int aileron2,
                         int throttle, int rudder, float pitch,
                         int elevatorManual, int elevatorFeedForward);
void setRightChildCommand(unsigned int index, int aileron1, int aileron2,
                          int throttle, int rudder, float pitch,
                          int elevatorManual, int elevatorFeedForward);
void sendPreparedChildCommands(bool intIsValid);
void forwardReceivedChildCommands(bool intIsValid);
void receiveAdjacentAircraftStates();
