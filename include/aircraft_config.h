#pragma once
#include <stdint.h>

// Parameter values are desired boot configuration; active values are latched.
extern int32_t aircraftIdParameter, aircraftCountParameter;
void activateAircraftConfiguration();
bool aircraftConfigurationValid();
int aircraftId();
int aircraftCount();
bool aircraftIsSingle();
bool aircraftIsLeft();
bool aircraftIsRight();
bool validAircraftConfiguration(int id, int count);
const char *aircraftName();
