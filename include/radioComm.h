#pragma once

#include "flight_config.h"
#include <Arduino.h>
#include <SBUS.h>
#include <DSMRX.h>

// 接收机硬件实例由 radioComm.cpp 持有；接收类型由 flight_config.h 统一选择。
#if defined USE_SBUS_RX
extern SBUS sbus;
extern uint16_t sbusChannels[16];
extern bool sbusFailSafe, sbusLostFrame;
#endif
#if defined USE_DSM_RX
extern DSM1024 DSM;
#endif

void radioSetup();
unsigned long getRadioPWM(int ch_num);
