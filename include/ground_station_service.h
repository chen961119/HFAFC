#pragma once
#include <stddef.h>

// Shares the parameter service's bounded USB RX/TX; never read Serial here.
void handleGroundStationCommand(char *line, char *response, size_t capacity);

enum class GroundStationResult { NotImplemented };
// V1 contracts only. Implement asynchronous jobs/progress in the next version.
GroundStationResult requestGroundCalibration(const char *sensor);
GroundStationResult requestGroundSensorSetting(const char *sensor, const char *setting);
GroundStationResult requestGroundRadioCalibration();
GroundStationResult requestGroundFlightModes(const char *low, const char *middle, const char *high);
