#pragma once

#include "flight_config.h"
#include <Arduino.h>
#include <SBUS.h>
#include <DSMRX.h>

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
