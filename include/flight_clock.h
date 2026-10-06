#pragma once

// current_time 为本轮起点的 micros() 时间戳（μs），dt 为相邻两轮间隔（秒）。
extern float dt;
extern unsigned long current_time;

void updateFlightClock(); // 使用无符号差值计算，兼容 micros() 回绕。
