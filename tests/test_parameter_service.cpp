// Runs the actual src/parameter_service.cpp with in-memory serial/SD drivers.
// Test commands live inside assertions, so keep them enabled in optimized builds.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "Arduino.h"
#include "SD.h"
#include "parameter_registry.h"
#include "parameter_service.h"
#include <cassert>
#include <iostream>
#include <vector>
#include <cfloat>

MockSerial Serial;
MockDisk disk;
MockSD SD;
float roll = 0.25f, pitch = 0.11f, effectiveness = -113.65f;
int32_t flag = 0;
size_t extendedCount = 0;
float extendedValues[513];
char extendedNames[513][97];
std::vector<FlightParameter> extendedTable;
const FlightParameter *controlParameterTable(size_t &count) {
  if (extendedCount) {
    count = extendedCount;
    return extendedTable.data();
  }
  static const FlightParameter parameters[] = {
      {"Kp_roll_angle", &roll, 0, 10, "Attitude", "Roll P"},
      {"Kp_pitch_rate", &pitch, 0, 10, "Attitude", "Pitch P"},
      {"indi_pitch_effectiveness", &effectiveness, -10000, -0.001f, "INDI", "Effectiveness"},
      {"flag", &flag, INT32_MIN, INT32_MAX, "Debug", "Integer flag"},
  };
  count = 4;
  return parameters;
}

void ticks(unsigned n = 300) {
  for (unsigned i = 0; i < n; ++i) pollParameterService();
}
std::string command(const std::string &text) {
  Serial.output.clear();
  Serial.feed(text + "\n");
  ticks(1100);
  return Serial.output;
}
void defaults() { roll = 0.25f; pitch = 0.11f; effectiveness = -113.65f; flag = 0; }
void reset(bool ready = true) {
  disk = {};
  Serial = {};
  defaults();
  initializeParameterService(ready);
}
bool contains(const std::string &text, const std::string &part) {
  return text.find(part) != std::string::npos;
}
uint32_t crc32(const std::string &text) {
  uint32_t result = 0xffffffff;
  for (unsigned char c : text) {
    result ^= c;
    for (int i = 0; i < 8; ++i)
      result = result & 1 ? (result >> 1) ^ 0xedb88320 : result >> 1;
  }
  return ~result;
}
void replaceAndReseal(std::string &file, const std::string &from, const std::string &to) {
  size_t pos = file.find(from);
  assert(pos != std::string::npos);
  file.replace(pos, from.size(), to);
  file.resize(file.find("CRC32="));
  char footer[32];
  snprintf(footer, sizeof(footer), "CRC32=%08x\n", crc32(file));
  file += footer;
}

int main() {
  reset();
  auto response = command("PARAM READ 7");
  assert(contains(response, "@HFAFC\t7\tBEGIN\tF-TEAM-7-INDI-EXP\t4\tSD_READY\tDEFAULTS"));
  assert(contains(response, "VALUE\tKp_roll_angle\tfloat\t0.25"));
  assert(contains(response, "VALUE\tflag\tint\t0\t-2147483648\t2147483647"));
  assert(contains(response, "@HFAFC\t7\tEND\t4"));

  // Fragmented input and USB backpressure never block or lose the response.
  Serial.output.clear();
  for (char c : std::string("PARAM\tREAD\t8")) { Serial.feed(std::string(1, c)); ticks(1); }
  assert(Serial.output.empty());
  Serial.feed("\r\n");
  Serial.capacity = 0;
  ticks();
  assert(Serial.output.empty());
  Serial.capacity = 512;
  ticks();
  assert(contains(Serial.output, "@HFAFC\t8\tEND"));

  // Range, NaN/Inf, overflow, invalid names/IDs, overlong lines; no SD mutation.
  for (const char *bad : {"nan", "inf", "-inf", "1e50", "1e-50", "2junk", "11", "-1"}) {
    assert(contains(command(std::string("PARAM SET 9 Kp_roll_angle ") + bad), "ERROR\tRANGE"));
    assert(roll == 0.25f && disk.files.empty());
  }
  assert(contains(command("PARAM SET 9 missing 0.5"), "UNKNOWN_PARAMETER"));
  assert(contains(command("PARAM READ 4294967296"), "ERROR\tSYNTAX"));
  assert(contains(command(std::string(600, 'x')), "LINE_TOO_LONG"));
  assert(contains(command("PARAM READ 10"), "@HFAFC\t10\tEND"));
  assert(roll == 0.25f && disk.files.empty());
  reset(false);
  assert(contains(command("PARAM SET 11 Kp_roll_angle 0.5"), "NO_SD"));
  assert(roll == 0.25f && disk.files.empty());

  // A successful write saves all registered parameters and is restored on boot.
  reset();
  assert(contains(command("PARAM SET 12 Kp_roll_angle 0.5"), "OK\tKp_roll_angle\tfloat\t0.5\tSAVED"));
  assert(roll == 0.5f && disk.files.count("params.cfg") == 1);
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f && pitch == 0.11f);
  assert(contains(command("PARAM SET 13 Kp_roll_angle 0.75"), "SAVED"));
  assert(disk.files.count("params_backup.cfg") == 1);
  defaults();
  initializeParameterService(true);
  assert(roll == 0.75f);
  // Torn primary: recover the previous complete backup.
  disk.files["params.cfg"].resize(40);
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f);

  // Backup failures must leave the primary and RAM intact.
  for (int fault = 0; fault < 5; ++fault) {
    reset();
    command("PARAM SET 14 Kp_roll_angle 0.5");
    std::string committed = disk.files.at("params.cfg");
    disk.failOpen = fault == 0;
    disk.shortWrite = fault == 1;
    disk.shortRead = fault == 2;
    disk.corruptRead = fault == 3;
    disk.failTruncate = fault == 4;
    assert(contains(command("PARAM SET 15 Kp_roll_angle 0.75"), "ERROR\tSD_WRITE"));
    assert(roll == 0.5f && disk.files.at("params.cfg") == committed);
    disk.failOpen = disk.shortWrite = disk.shortRead = disk.corruptRead = disk.failTruncate = false;
    defaults();
    initializeParameterService(true);
    assert(roll == 0.5f);
  }

  // A failed primary save preserves the validated backup; retry and boot recover.
  reset();
  command("PARAM SET 26 Kp_roll_angle 0.5");
  disk.writesBeforeFailure = 1; // Backup succeeds, primary is partially written.
  assert(contains(command("PARAM SET 27 Kp_roll_angle 0.75"), "SD_WRITE"));
  assert(roll == 0.5f && disk.files.at("params_backup.cfg").find("=0.5") != std::string::npos);
  assert(disk.files.count("params.cfg") == 0);
  disk.writesBeforeFailure = -1;
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f);
  const auto recoveredBackup = disk.files.at("params_backup.cfg");
  assert(contains(command("PARAM SET 28 Kp_roll_angle 0.75"), "SAVED"));
  assert(disk.files.at("params_backup.cfg") == recoveredBackup);
  defaults(); initializeParameterService(true); assert(roll == 0.75f);

  // Load existing slot files, selecting the newer generation, then migrate on SET.
  reset();
  command("PARAM SET 29 Kp_roll_angle 0.5");
  command("PARAM SET 30 Kp_roll_angle 0.75");
  disk.files["params0.cfg"] = disk.files.at("params_backup.cfg");
  disk.files["params1.cfg"] = disk.files.at("params.cfg");
  disk.files.erase("params.cfg"); disk.files.erase("params_backup.cfg");
  defaults(); initializeParameterService(true); assert(roll == 0.75f);
  assert(disk.files.count("params.cfg") == 0); // Loading alone does not write.
  assert(contains(command("PARAM SET 31 flag 1"), "SAVED"));
  assert(disk.files.count("params.cfg") == 1 && disk.files.count("params_backup.cfg") == 1);
  defaults(); initializeParameterService(true); assert(roll == 0.75f && flag == 1);

  // Even with a correct CRC, reject wrong profile, out-of-range and partial sets.
  for (int corruption = 0; corruption < 4; ++corruption) {
    reset();
    command("PARAM SET 16 Kp_roll_angle 0.5");
    auto &file = disk.files.at("params.cfg");
    if (corruption == 0) replaceAndReseal(file, "F-TEAM", "A-TEAM");
    if (corruption == 1) replaceAndReseal(file, "Kp_roll_angle:float=0.5", "Kp_roll_angle:float=20");
    if (corruption == 2) replaceAndReseal(file, "Kp_pitch_rate:float=0.11\n", "");
    if (corruption == 3) replaceAndReseal(file, "Kp_roll_angle:float=0.5", "Kp_roll_angle:float=nan");
    defaults();
    initializeParameterService(true);
    assert(roll == 0.25f && pitch == 0.11f && effectiveness == -113.65f);
  }
  // Primary wins even across generation wrap; the backup retains the old values.
  reset();
  command("PARAM SET 17 Kp_roll_angle 0.5");
  command("PARAM SET 18 Kp_roll_angle 0.75");
  replaceAndReseal(disk.files.at("params_backup.cfg"), "GEN=1", "GEN=4294967295");
  replaceAndReseal(disk.files.at("params.cfg"), "GEN=2", "GEN=0");
  defaults();
  initializeParameterService(true);
  assert(roll == 0.75f);

  // A valid primary has priority even if backup reports a higher generation.
  replaceAndReseal(disk.files.at("params_backup.cfg"), "GEN=4294967295", "GEN=100");
  defaults(); initializeParameterService(true); assert(roll == 0.75f);

  // Nine significant digits round-trip float values without silent RAM/SD drift.
  for (const char *value : {"0.123456789", "1.23456789", "9.87654321"}) {
    assert(contains(command(std::string("PARAM SET 19 Kp_roll_angle ") + value), "SAVED"));
    float expected = roll;
    defaults();
    initializeParameterService(true);
    assert(memcmp(&expected, &roll, sizeof(float)) == 0);
  }
  assert(contains(command("PARAM SET 20 Kp_roll_angle 0.2"), "float\t0.2\tSAVED"));
  for (const char *value : {"0.00001", "0.0002", "0.0000000000000000000000001"}) {
    auto plain = command(std::string("PARAM SET 20 Kp_roll_angle ") + value);
    assert(contains(plain, std::string("float\t") + value + "\tSAVED"));
    assert(contains(disk.files.at("params.cfg"), std::string("Kp_roll_angle:float=") + value));
    float expected = roll;
    defaults(); initializeParameterService(true);
    assert(memcmp(&expected, &roll, sizeof(float)) == 0);
  }
  // Full int32 range survives the SD path exactly and rejects fractional input.
  for (const char *value : {"2147483647", "-2147483648", "1", "0"}) {
    assert(contains(command(std::string("PARAM SET 21 flag ") + value), "SAVED"));
    int32_t expected = flag;
    defaults();
    initializeParameterService(true);
    assert(flag == expected);
  }
  const auto committed = disk.files;
  for (const char *value : {"1.0", "0.5", "1e0", "2147483648", "-2147483649", "nan"}) {
    assert(contains(command(std::string("PARAM SET 22 flag ") + value), "ERROR\tRANGE"));
    assert(flag == 0 && disk.files == committed);
  }
  disk.failOpen = true;
  assert(contains(command("PARAM SET 23 flag 1"), "SD_WRITE"));
  assert(flag == 0);
  // Legacy V1 float-only file migrates with new integer defaults.
  reset();
  command("PARAM SET 24 Kp_roll_angle 0.5");
  auto &legacy = disk.files.at("params.cfg");
  replaceAndReseal(legacy, "HFAFC_PARAMS_V3", "HFAFC_PARAMS_V1");
  replaceAndReseal(legacy, "COUNT=4\n", "");
  replaceAndReseal(legacy, "Kp_roll_angle:float", "Kp_roll_angle");
  replaceAndReseal(legacy, "Kp_pitch_rate:float", "Kp_pitch_rate");
  replaceAndReseal(legacy, "indi_pitch_effectiveness:float", "indi_pitch_effectiveness");
  replaceAndReseal(legacy, "flag:int=0\n", "");
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f && flag == 0);
  assert(contains(command("PARAM SET 25 flag 1"), "SAVED"));
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f && flag == 1);

  // The complete 512-entry table, including a >8 KiB file and long names,
  // survives read, primary/backup verification and reboot without drift.
  extendedTable.reserve(513);
  for (size_t i = 0; i < 513; ++i) {
    snprintf(extendedNames[i], sizeof(extendedNames[i]),
             "ground_parameter_%03u_with_a_long_name_for_capacity_and_file_buffer_checks", unsigned(i));
    extendedValues[i] = 0.2f;
    extendedTable.emplace_back(extendedNames[i], &extendedValues[i], 0, 10, "Ground", "Tuning value");
  }
  extendedCount = 512;
  disk = {}; Serial = {};
  initializeParameterService(true);
  response = command("PARAM READ 30");
  assert(contains(response, "BEGIN\tF-TEAM-7-INDI-EXP\t512"));
  assert(contains(response, extendedNames[511]));
  assert(contains(response, "END\t512"));
  assert(contains(command(std::string("PARAM SET 31 ") + extendedNames[511] + " 0.75"), "SAVED"));
  assert(disk.files.at("params.cfg").size() > 8192);
  assert(contains(command(std::string("PARAM SET 32 ") + extendedNames[0] + " 0.5"), "SAVED"));
  for (auto &value : extendedValues) value = 0.2f;
  initializeParameterService(true);
  assert(extendedValues[511] == 0.75f && extendedValues[0] == 0.5f);
  // New registry fields default while all previously saved V3 values survive.
  extendedCount = 513;
  initializeParameterService(true);
  assert(contains(Serial.output, "Registry exceeds 512"));
  // Compatible table growth below capacity.
  extendedCount = 511; disk = {}; initializeParameterService(true);
  assert(contains(command(std::string("PARAM SET 33 ") + extendedNames[510] + " 0.8"), "SAVED"));
  extendedCount = 512;
  extendedValues[510] = extendedValues[511] = 0.2f;
  initializeParameterService(true);
  assert(extendedValues[510] == 0.8f && extendedValues[511] == 0.2f);
  // V2 upgrade preserves the original 28-entry prefix.
  extendedCount = 28; disk = {}; initializeParameterService(true);
  assert(contains(command(std::string("PARAM SET 34 ") + extendedNames[0] + " 0.9"), "SAVED"));
  auto &v2 = disk.files.at("params.cfg");
  replaceAndReseal(v2, "HFAFC_PARAMS_V3", "HFAFC_PARAMS_V2");
  replaceAndReseal(v2, "COUNT=28\n", "");
  extendedCount = 512; extendedValues[0] = 0.2f;
  initializeParameterService(true);
  assert(extendedValues[0] == 0.9f);
  // Long fixed decimals must fit the value/file buffers and preserve float bits.
  extendedTable[0] = FlightParameter("decimal_extremes", &extendedValues[0], -FLT_MAX, FLT_MAX, "Test", "Full float32 decimal range");
  extendedCount = 1; disk = {}; initializeParameterService(true);
  for (const char *text : {"340282350000000000000000000000000000000",
                           "-340282350000000000000000000000000000000",
                           "0.000000000000000000000000000000000000011754944"}) {
    response = command(std::string("PARAM SET 35 decimal_extremes ") + text);
    assert(contains(response, "SAVED"));
    const auto start = response.find("\tfloat\t") + 7;
    const auto valueText = response.substr(start, response.find('\t', start) - start);
    assert(valueText.find_first_of("eE") == std::string::npos);
    float expected = extendedValues[0];
    extendedValues[0] = 0;
    initializeParameterService(true);
    assert(memcmp(&expected, &extendedValues[0], sizeof(float)) == 0);
  }
  // Direction is an integer but zero must never enter the actuator divider.
  extendedTable[0] = FlightParameter("pwm_channel3_rev", &flag, -1, 1, "Actuator", "Direction");
  extendedCount = 1; flag = 1; disk = {};
  initializeParameterService(true);
  assert(contains(command("PARAM SET 35 pwm_channel3_rev 0"), "ERROR\tRANGE"));
  assert(flag == 1 && disk.files.empty());
  assert(contains(command("PARAM SET 36 pwm_channel3_rev -1"), "SAVED"));
  flag = 1; initializeParameterService(true); assert(flag == -1);

  std::cout << "PASS: firmware protocol, bounds, fast-math NaN/Inf rejection, SD failure rollback, boot recovery\n";
}
