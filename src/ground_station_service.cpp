#include "aircraft_config.h"
#include "ground_station_service.h"
#include "control_state.h"
#include "filters.h"
#include "flight_lock.h"
#include "device_reboot.h"
#include "sensor_processing.h"
#include "interaircraft_comm.h"
#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

GroundStationResult requestGroundCalibration(const char *) { return GroundStationResult::NotImplemented; }
GroundStationResult requestGroundSensorSetting(const char *, const char *) { return GroundStationResult::NotImplemented; }
GroundStationResult requestGroundRadioCalibration() { return GroundStationResult::NotImplemented; }
GroundStationResult requestGroundFlightModes(const char *, const char *, const char *) { return GroundStationResult::NotImplemented; }

namespace {
bool validId(const char *text, uint32_t &id) {
  if (!*text) return false;
  uint64_t value = 0;
  for (; *text; ++text) {
    if (*text < '0' || *text > '9') return false;
    value = value * 10 + (*text - '0');
    if (value > UINT32_MAX) return false;
  }
  id = static_cast<uint32_t>(value);
  return true;
}
bool validMode(const char *mode) {
  return !strcmp(mode, "MANUAL") || !strcmp(mode, "STABILIZE_NO_I") || !strcmp(mode, "STABILIZE");
}
bool activeSensor(const char *name) {
  if (!strcmp(name, "AIRSPEED") || !strcmp(name, "ROTATE")) return true;
#if defined INTIMU
  if (!strcmp(name, "IMU") || !strcmp(name, "GYRO") || !strcmp(name, "ACCEL")) return true;
#endif
#if defined EXTIMU
  if (!strcmp(name, "EXTIMU")) return true;
#endif
  return false;
}
}

void handleGroundStationCommand(char *line, char *response, size_t capacity) {
  char *tokens[8], *save;
  size_t count = 0;
  uint32_t id = 0;
  for (char *p = strtok_r(line, " \t", &save); p; p = strtok_r(nullptr, " \t", &save)) {
    if (count == 8) { count = 0; break; }
    tokens[count++] = p;
  }
  if (count < 3 || !validId(tokens[2], id)) {
    snprintf(response, capacity, "@COFLY\t0\tERROR\tSYNTAX\tUse GCS CAPS/DATA id\n");
    return;
  }
  const char *code = "SYNTAX", *message = "Invalid GCS arguments";
  if (!strcmp(tokens[1], "REBOOT") && count == 3) {
    if (requestDeviceReboot())
      snprintf(response, capacity, "@COFLY\t%lu\tREBOOT\tstatus=REBOOTING\n", static_cast<unsigned long>(id));
    else
      snprintf(response, capacity, "@COFLY\t%lu\tERROR\tLOCK_REQUIRED\tLock aircraft before reboot\n", static_cast<unsigned long>(id));
    return;
  }
  if (!strcmp(tokens[1], "CAPS") && count == 3) {
    // main.loop samples BMI088 into the legacy *_6050 variables. Do not
    // claim MPU6050/MPU9250 or BMP280 is sampled based on old config names.
#if defined INTIMU && defined USE_MPU6050_I2C
    const char *imu = "BMI088_SPI";
#elif defined INTIMU
    const char *imu = "BMI088_SAMPLING_CONFIG_MISMATCH";
#else
    const char *imu = "DISABLED";
#endif
#if defined EXTIMU
    const char *external = "ENABLED";
#else
    const char *external = "DISABLED";
#endif
#if defined USE_BAROMETER
    const char *barometer = "INITIALIZED_NOT_SAMPLED";
#else
    const char *barometer = "DISABLED";
#endif
#if defined USE_SBUS_RX
    const char *radio = "SBUS";
#elif defined USE_PPM_RX
    const char *radio = "PPM";
#elif defined USE_PWM_RX
    const char *radio = "PWM";
#elif defined USE_DSM_RX
    const char *radio = "DSM";
#else
    const char *radio = "DISABLED";
#endif
    snprintf(response, capacity,
      "@COFLY\t%lu\tCAPS\tversion=1\timu=%s\texternal=%s\tairspeed=MS4525\trotate=MT6701\tbarometer=%s\tradio=%s\tcalibration=STUB\tsensor_setting=STUB\tradio_calibration=STUB\tflight_modes=STUB\treboot=SUPPORTED\n",
      static_cast<unsigned long>(id), imu, external, barometer, radio);
    return;
  }
  if (!strcmp(tokens[1], "DATA") && count == 3) {
    const char *mode = currentMode == MANUAL_MODE ? "MANUAL" :
      currentMode == STABLIZE_MODE_NO_I ? "STABILIZE_NO_I" : "STABILIZE";
    // Snapshot existing measurements only. No invented ground speed/altitude.
    float relativeRoll = 0;
    const bool relativeValid = localWingRelativeAngle(relativeRoll);
    const int written = snprintf(response, capacity,
      "@COFLY\t%lu\tDATA\tms=%lu\troll=%.3f\tpitch=%.3f\tyaw=%.3f\tgx=%.3f\tgy=%.3f\tgz=%.3f\tax=%.3f\tay=%.3f\taz=%.3f\tairspeed=%.3f\tangle=%.3f\tangle_valid=%d\trc1=%d\trc2=%d\trc3=%d\trc4=%d\trc5=%d\trc6=%d\tmode=%s\tlocked=%d\n",
      static_cast<unsigned long>(id), millis(), roll_IMU, pitch_IMU, yaw_IMU,
      GyroX_6050, GyroY_6050, GyroZ_6050, AccX_6050, AccY_6050, AccZ_6050,
      airspeed_A, relativeRoll, relativeValid, channel_1_pwm,
      channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm,
      mode, isFlightLocked());
    if (written < 0 || static_cast<size_t>(written) >= capacity)
      snprintf(response, capacity, "@COFLY\t%lu\tERROR\tDATA_TOO_LONG\tMeasurement exceeds frame capacity\n",
               static_cast<unsigned long>(id));
    return;
  }
  if (!strcmp(tokens[1], "CONFIG") && count == 3) {
    const unsigned master = aircraftConfigurationValid() && aircraftId() == 1;
    const int written = snprintf(response, capacity,
      "@COFLY\t%lu\tCONFIG\tcount=%u\tmaster=%u\troll_a=%.3f\tab=%.3f\tac=%.3f\tbd=%.3f\tce=%.3f\tdf=%.3f\teg=%.3f\tleft_valid=%d\tright_valid=%d\taircraft_id=%d\tconfig_valid=%d\n",
      static_cast<unsigned long>(id), static_cast<unsigned>(aircraftCount()), master, roll_IMU,
      phiB_raw - roll_IMU, phiC_raw - roll_IMU, phiD_raw - phiB_raw,
      phiE_raw - phiC_raw, phiF_raw - phiD_raw, phiG_raw - phiE_raw,
      wingStateFresh(true), wingStateFresh(false), aircraftId(), aircraftConfigurationValid());
    if (written < 0 || static_cast<size_t>(written) >= capacity)
      snprintf(response, capacity, "@COFLY\t%lu\tERROR\tDATA_TOO_LONG\tConfiguration exceeds frame capacity\n",
               static_cast<unsigned long>(id));
    return;
  }
  if (!strcmp(tokens[1], "CALIBRATE") && count == 4 && activeSensor(tokens[3])) {
    requestGroundCalibration(tokens[3]); code = "NOT_IMPLEMENTED"; message = "Calibration USB job reserved for V2";
  } else if (!strcmp(tokens[1], "SENSOR_SET") && count == 5 && activeSensor(tokens[3])) {
    requestGroundSensorSetting(tokens[3], tokens[4]); code = "NOT_IMPLEMENTED"; message = "Sensor setting reserved for V2";
  } else if (!strcmp(tokens[1], "RADIO_CALIBRATE") && count == 3) {
    requestGroundRadioCalibration(); code = "NOT_IMPLEMENTED"; message = "Radio calibration reserved for V2";
  } else if (!strcmp(tokens[1], "FLIGHT_MODES") && count == 6 &&
             validMode(tokens[3]) && validMode(tokens[4]) && validMode(tokens[5])) {
    requestGroundFlightModes(tokens[3], tokens[4], tokens[5]); code = "NOT_IMPLEMENTED"; message = "Flight mode mapping reserved for V2";
  }
  snprintf(response, capacity, "@COFLY\t%lu\tERROR\t%s\t%s\n", static_cast<unsigned long>(id), code, message);
}
