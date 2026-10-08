#pragma once

#include "flight_config.h"
#include <Arduino.h>
#include <SBUS.h>
#include <DSMRX.h>

// 接收机硬件实例由 radio_comm.cpp 持有；接收类型由 flight_config.h 统一选择。
#if defined USE_SBUS_RX
extern SBUS sbus;
extern uint16_t sbusChannels[16];
extern bool sbusFailSafe, sbusLostFrame;
#endif
#if defined USE_DSM_RX
extern DSM1024 DSM;
#endif

void radioSetup();
// 读取接收机通道并更新滤波后的 channel_*_pwm，供下一控制周期使用。
void getCommands();
unsigned long getRadioPWM(int ch_num);
