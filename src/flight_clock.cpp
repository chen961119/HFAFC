#include "flight_clock.h"
#include <Arduino.h>

float dt = 0.0f;
unsigned long current_time = 0;
static unsigned long prev_time = 0;

// 记录本周期起始时间，并计算相邻两周期的时间差 dt（秒）。
void updateFlightClock() {
  // micros() 的无符号减法可跨计数器回绕计算间隔，dt 单位为秒。
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0f;
}
