#include "flight_clock.h"
#include <Arduino.h>

float dt = 0.0f;
unsigned long current_time = 0;
static unsigned long prev_time = 0;

void updateFlightClock() {
  prev_time = current_time;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0f;
}
