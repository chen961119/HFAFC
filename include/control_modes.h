#pragma once

// 期望状态、控制律与混控；输入为遥控/机间命令，输出为归一化舵量与前馈控制偏移。
void getDesState();
// void controlANGLE();
void controlANGLE2();
// void controlRATE();
int PITCH_INDI_control();
void controlMixer();
void failSafe();
void increase_Clp();
void controlFlapMotion();
void commandMotors();
void armMotors();
void calibrateESCs();
void switchRollYaw(int reverseRoll, int reverseYaw);
void throttleCut();
void initializeRadioFailsafeChannels();
void initializeInitialControlMode(); // 上电默认手动模式。
// 根据 currentMode 选择手动、无积分增稳或有积分增稳。
void runSelectedControlMode();
