#pragma once
#include "parameter_registry.h"

// EEPROM 0..127 is unused; retain slot addresses for current-format snapshots.
// Two 2048-byte slots at 128 and 2176; 2024 bytes of shared text records per slot.
bool initializeEEPROMParameters();
bool readEEPROMParameter(const FlightParameter &parameter, double &value);
bool hasEEPROMMigrationSource(const FlightParameter &parameter);
bool saveEEPROMParameters(const FlightParameter *table, size_t count, const double *values, bool sdVerified = false);
const char *eepromParameterState();
