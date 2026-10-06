#pragma once

// 期望状态、控制律与混控；输入为遥控/机间命令，输出为归一化舵量和 PWM。
unsigned long getRadioPWM(int ch_num);
void getDesState();
void controlANGLE();
void controlANGLE2();
void controlRATE();
int PITCH_INDI_control();
void controlMixer();
void scaleCommands();
void getCommands();
void failSafe();
void armedStatus();
void increase_Clp();
void controlFlapMotion();
void commandMotors();
void armMotors();
void calibrateESCs();
void switchRollYaw(int reverseRoll, int reverseYaw);
void throttleCut();
void initializeRadioFailsafeChannels();
// 根据 currentMode 选择手动、无积分增稳或有积分增稳。
void runSelectedControlMode();
