// Runs the actual src/parameter_service.cpp with in-memory serial/SD drivers.
#include "Arduino.h"
#include "SD.h"
#include "parameter_registry.h"
#include "parameter_service.h"
#include <cassert>
#include <iostream>

MockSerial Serial;
MockDisk disk;
MockSD SD;
float roll = 0.25f, pitch = 0.11f, effectiveness = -113.65f;
int32_t flag = 0;
const FlightParameter *controlParameterTable(size_t &count) {
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
  ticks(300);
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
  assert(contains(command(std::string(400, 'x')), "LINE_TOO_LONG"));
  assert(contains(command("PARAM READ 10"), "@HFAFC\t10\tEND"));
  assert(roll == 0.25f && disk.files.empty());
  reset(false);
  assert(contains(command("PARAM SET 11 Kp_roll_angle 0.5"), "NO_SD"));
  assert(roll == 0.25f && disk.files.empty());

  // A successful write saves all registered parameters and is restored on boot.
  reset();
  assert(contains(command("PARAM SET 12 Kp_roll_angle 0.5"), "OK\tKp_roll_angle\tfloat\t0.5\tSAVED"));
  assert(roll == 0.5f && disk.files.count("params0.cfg") == 1);
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f && pitch == 0.11f);
  assert(contains(command("PARAM SET 13 Kp_roll_angle 0.75"), "SAVED"));
  assert(disk.files.count("params1.cfg") == 1);
  defaults();
  initializeParameterService(true);
  assert(roll == 0.75f);
  // Torn newest file: use the older complete slot.
  disk.files["params1.cfg"].resize(40);
  defaults();
  initializeParameterService(true);
  assert(roll == 0.5f);

  // Every I/O failure preserves RAM and the previously committed slot.
  for (int fault = 0; fault < 5; ++fault) {
    reset();
    command("PARAM SET 14 Kp_roll_angle 0.5");
    std::string committed = disk.files.at("params0.cfg");
    disk.failOpen = fault == 0;
    disk.shortWrite = fault == 1;
    disk.shortRead = fault == 2;
    disk.corruptRead = fault == 3;
    disk.failTruncate = fault == 4;
    assert(contains(command("PARAM SET 15 Kp_roll_angle 0.75"), "ERROR\tSD_WRITE"));
    assert(roll == 0.5f && disk.files.at("params0.cfg") == committed);
    disk.failOpen = disk.shortWrite = disk.shortRead = disk.corruptRead = disk.failTruncate = false;
    defaults();
    initializeParameterService(true);
    assert(roll == 0.5f);
  }

  // Even with a correct CRC, reject wrong profile, out-of-range and partial sets.
  for (int corruption = 0; corruption < 4; ++corruption) {
    reset();
    command("PARAM SET 16 Kp_roll_angle 0.5");
    auto &file = disk.files.at("params0.cfg");
    if (corruption == 0) replaceAndReseal(file, "F-TEAM", "A-TEAM");
    if (corruption == 1) replaceAndReseal(file, "Kp_roll_angle:float=0.5", "Kp_roll_angle:float=20");
    if (corruption == 2) replaceAndReseal(file, "Kp_pitch_rate:float=0.11\n", "");
    if (corruption == 3) replaceAndReseal(file, "Kp_roll_angle:float=0.5", "Kp_roll_angle:float=nan");
    defaults();
    initializeParameterService(true);
    assert(roll == 0.25f && pitch == 0.11f && effectiveness == -113.65f);
  }
  // Sequence rollover correctly identifies generation zero as newer than max.
  reset();
  command("PARAM SET 17 Kp_roll_angle 0.5");
  command("PARAM SET 18 Kp_roll_angle 0.75");
  replaceAndReseal(disk.files.at("params0.cfg"), "GEN=1", "GEN=4294967295");
  replaceAndReseal(disk.files.at("params1.cfg"), "GEN=2", "GEN=0");
  defaults();
  initializeParameterService(true);
  assert(roll == 0.75f);

  // Nine significant digits round-trip float values without silent RAM/SD drift.
  for (const char *value : {"0.123456789", "1.23456789", "9.87654321"}) {
    assert(contains(command(std::string("PARAM SET 19 Kp_roll_angle ") + value), "SAVED"));
    float expected = roll;
    defaults();
    initializeParameterService(true);
    assert(memcmp(&expected, &roll, sizeof(float)) == 0);
  }
  assert(contains(command("PARAM SET 20 Kp_roll_angle 0.2"), "float\t0.2\tSAVED"));
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
  auto &legacy = disk.files.at("params0.cfg");
  replaceAndReseal(legacy, "HFAFC_PARAMS_V2", "HFAFC_PARAMS_V1");
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
  std::cout << "PASS: firmware protocol, bounds, fast-math NaN/Inf rejection, SD failure rollback, boot recovery\n";
}
