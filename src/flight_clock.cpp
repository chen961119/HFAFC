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

// 从飞行时钟 current_time 起忙等到目标周期时长；freq 单位为 Hz。
// 若处理已超时则立即返回，不会补偿遗漏的周期。
void loopRate(int freq) {
  // micros() 的无符号差值可处理计数器回绕；忙等期间 CPU 不执行其他循环任务。
  const float intervalMicros = 1000000.0f / freq;
  unsigned long checker = micros();
  while (intervalMicros > (checker - current_time)) {
    checker = micros();
  }
}
