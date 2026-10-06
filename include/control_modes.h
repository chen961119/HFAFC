#pragma once

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
void runSelectedControlMode();
