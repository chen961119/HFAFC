#pragma once
#include <cstdint>
#define LOW 0
#define constrain(x, lo, hi) ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))
inline void digitalWrite(int,int) {}
inline void delay(int) {}
