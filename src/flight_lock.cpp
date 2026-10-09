#include "flight_lock.h"

namespace {
bool flightLocked = true;
}

bool isFlightLocked() { return flightLocked; }
void setFlightLocked(bool locked) { flightLocked = locked; }
