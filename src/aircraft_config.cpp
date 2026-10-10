#include "aircraft_config.h"

int32_t aircraftIdParameter = 0; // Unconfigured, never silently boot as A.
int32_t aircraftCountParameter = 3;
namespace { int activeId = 0, activeCount = 3; }
bool validAircraftConfiguration(int id, int count) {
  const int maximum = count == 1 ? 1 : count == 3 ? 3 : count == 4 ? 4 : count == 5 ? 5 : count == 7 ? 7 : 0;
  return maximum && id >= 0 && id <= maximum;
}
void activateAircraftConfiguration() {
  activeId = validAircraftConfiguration(aircraftIdParameter, aircraftCountParameter) ? aircraftIdParameter : 0;
  activeCount = validAircraftConfiguration(aircraftIdParameter, aircraftCountParameter) ? aircraftCountParameter : 3;
}
bool aircraftConfigurationValid() { return activeId > 0; }
int aircraftId() { return activeId; }
int aircraftCount() { return activeCount; }
bool aircraftIsSingle() { return activeCount == 1; }
bool aircraftIsLeft() { return activeId == 2 || activeId == 4 || activeId == 6; }
bool aircraftIsRight() { return activeId == 3 || activeId == 5 || activeId == 7; }
const char *aircraftName() {
  static const char *const names[] = {"UNCONFIGURED", "A", "B", "C", "D", "E", "F", "G"};
  return names[activeId];
}
