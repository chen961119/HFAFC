#pragma once

// 串口诊断输出；打印节流计时由 debug_print.cpp 独立维护。
void printRadioData();
void printDesiredState();
void printConfigurationData();
void printPIDoutput();
void printMotorCommands();
void printServoCommands();
void printLoopRate();
void printLocalThrottle();
void printGyroData();
void printAccelData();
void printMagData();
void printRollPitchYaw();
void printQuaternion();
