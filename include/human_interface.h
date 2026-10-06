#pragma once

#include "flight_config.h"

// OLED 屏幕参数和人机接口函数；按键与显示状态均由实现文件维护。
constexpr int SCREEN_WIDTH = 128;
constexpr int SCREEN_HEIGHT = 32;
constexpr int OLED_RESET = -1;
constexpr int SCREEN_ADDRESS = 0x3C;

void beginHumanInterfaceLinks();
void initializeHumanInterface();
void displayAircraftIdentity();
void ProcessButtonState();
void loopBlink();
void setupBlink(int numBlinks, int upTime, int downTime);
void displaythumbsup();
void displayID(char *ss);
void displaySD(char *ss);
void displayfilenum();
void displayAttitude();
void displayFlightModeIndicators(FlightMode mode);
