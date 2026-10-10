#include "parameter_record.h"
#include "aircraft_config.h"
#include "imu_calibration_integrity.h"
#include "parameter_eeprom.h"
#include "sensor_processing.h"
#include "flight_lock.h"
#include "serial_ports.h"
#include "parameter_service.h"
#include "ground_station_service.h"
#include "device_reboot.h"
#include "parameter_registry.h"
#include "flight_config.h"
#include <Arduino.h>
#include <SD.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

namespace {
constexpr size_t MAX_PARAMETERS = 512;
constexpr size_t LINE_SIZE = 512;
// Plain decimal float32 text needs up to 56 bytes, including very small values.
// Capacity is derived from the compatible name length, record size and row limit.
// Shared by read/write only after the file is closed.
constexpr size_t FILE_SIZE = MAX_PARAMETERS * PARAMETER_RECORD_SIZE + 256;
constexpr size_t VALUE_SIZE = PARAMETER_VALUE_SIZE;
const char *const primaryPath = "params.cfg";
const char *const backupPath = "params_backup.cfg";

#if defined TEAM
#define PARAM_CONTROL "TEAM"
#else
#define PARAM_CONTROL "SINGLE"
#endif
#if defined TESTINDI
#define PARAM_INDI "INDI"
#else
#define PARAM_INDI "PID"
#endif
#if defined expensive
#define PARAM_MODEL "EXP"
#else
#define PARAM_MODEL "STD"
#endif
char profile[128];
bool eepromLoaded[MAX_PARAMETERS];

const FlightParameter *table;
size_t count;
bool storageReady;
bool rebootRequired = false;
const char *activePath = nullptr;
uint32_t generation;
char rxLine[LINE_SIZE];
size_t rxLength;
bool rxOverflow;
char txLine[LINE_SIZE];
size_t txLength;
size_t listIndex;
bool listing;
uint32_t listId;
double snapshot[MAX_PARAMETERS];
double defaults[MAX_PARAMETERS];
// Main-loop-only, non-reentrant service: large buffers must never live on stack.
char fileData[FILE_SIZE];
double candidate[MAX_PARAMETERS];
double previous[MAX_PARAMETERS];
double verified[MAX_PARAMETERS];
double guarded[MAX_PARAMETERS];
bool fileSeen[MAX_PARAMETERS];
const char *fileNames[MAX_PARAMETERS];
const char *loadState = "DEFAULTS";

struct MigrationStats {
  size_t matched;
  size_t defaultsUsed;
  size_t skipped;
};

MigrationStats loadStats;

// Bit inspection is intentional: the project enables -ffast-math, which can
// optimize away ordinary isfinite() checks. Reject NaN/Inf before comparisons.
using ParameterRecord::finiteFloat;
using ParameterRecord::parseFloat;
using ParameterRecord::parseUnsigned;
bool validValue(size_t index,double value) {
  return validParameterValue(table[index],value) &&
      (table[index].id!=1002 || value==1 || value==3 || value==4 || value==5 || value==7);
}
bool parseValue(size_t index,const char *text,double &value) {
  return ParameterRecord::parseValue(table[index],text,value) && validValue(index,value);
}
void formatValue(size_t index,double value,char *text,size_t capacity) {
  ParameterRecord::formatValue(table[index],value,text,capacity);
}

uint32_t checksum(const char *bytes, size_t length) {
  uint32_t crc = UINT32_MAX;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint8_t>(bytes[i]);
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320UL & (0UL - (crc & 1)));
  }
  return ~crc;
}

// Restore current-format records by name and type; missing entries use defaults.
bool readParameterFile(const char *path, double *values, uint32_t &sequence,
                       bool *needsMigration = nullptr,
                       MigrationStats *stats = nullptr) {
  if (needsMigration) *needsMigration = false;
  MigrationStats parsedStats{0, count, 0};
  char (&data)[FILE_SIZE] = fileData;
  File file = SD.open(path, FILE_READ);
  if (!file || file.size() == 0 || file.size() >= sizeof(data)) return false;
  const size_t length = file.size();
  const size_t read = file.read(reinterpret_cast<uint8_t *>(data), length);
  file.close();
  if (read != length || data[length - 1] != '\n' || memchr(data, '\0', length))
    return false;
  data[length] = '\0';
  char *footer = strstr(data, "CRC32=");
  if (!footer || footer == data || footer[-1] != '\n') return false;
  // CRC footer has exactly eight hexadecimal digits and a final LF.
  if (strlen(footer) != 15) return false;
  uint32_t expected = 0;
  for (int i = 6; i < 14; ++i) {
    char c = footer[i];
    unsigned digit;
    if (c >= '0' && c <= '9') digit = c - '0';
    else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
    else return false;
    expected = (expected << 4) | digit;
  }
  if (checksum(data, footer - data) != expected) return false;
  *footer = '\0';
  char *save;
  char *line = strtok_r(data, "\n", &save);
  char header[128];
  snprintf(header, sizeof(header), "COFLY_PARAMS_V5 %s", profile);
  if (!line || strcmp(line,header)) return false;
  line = strtok_r(nullptr, "\n", &save);
  if (!line || strncmp(line, "GEN=", 4) || !parseUnsigned(line + 4, sequence))
    return false;
  memcpy(values, defaults, count * sizeof(double));
  memset(fileSeen, 0, sizeof(fileSeen));
  uint32_t declared;
  line = strtok_r(nullptr, "\n", &save);
  if (!line || strncmp(line,"COUNT=",6) || !parseUnsigned(line+6,declared) || declared>MAX_PARAMETERS) return false;
  size_t sdCount=0;
  for (size_t i=0;i<count;++i) if (table[i].storage==ParameterStorage::SD) ++sdCount;
  if (needsMigration && declared!=sdCount) *needsMigration=true;
  for (size_t row=0;row<declared;++row) {
    line=strtok_r(nullptr,"\n",&save);
    char *name,*type,*text;
    if (!line || !ParameterRecord::split(line,name,type,text)) return false;
    for (size_t j=0;j<row;++j) if (!strcmp(name,fileNames[j])) return false;
    fileNames[row]=name;
    size_t index=0;
    while (index<count && strcmp(name,table[index].name)) ++index;
    if (index==count) {
      ++parsedStats.skipped;
      if (needsMigration) *needsMigration=true;
      continue;
    }
    if (fileSeen[index]) return false;
    fileSeen[index]=true;
    if (strcmp(type,table[index].type()) || !parseValue(index,text,values[index])) {
      fileSeen[index]=false;values[index]=defaults[index];++parsedStats.skipped;
      if (needsMigration) *needsMigration=true;
    } else { ++parsedStats.matched;--parsedStats.defaultsUsed; }
  }
  if (strtok_r(nullptr,"\n",&save)) return false;
  if (stats) *stats=parsedStats;
  return true;
}

bool sameSDValues(const double *a, const double *b) {
  for (size_t i=0; i<count; ++i)
    if (table[i].storage == ParameterStorage::SD && a[i] != b[i]) return false;
  return true;
}

bool writeSnapshot(const char *path, const double *values, uint32_t sequence) {
  char (&data)[FILE_SIZE] = fileData;
  size_t sdCount = 0; for (size_t i=0; i<count; ++i) if (table[i].storage == ParameterStorage::SD) ++sdCount;
  size_t used = snprintf(data, sizeof(data), "COFLY_PARAMS_V5 %s\nGEN=%lu\nCOUNT=%u\n",
                         profile, static_cast<unsigned long>(sequence), static_cast<unsigned>(sdCount));
  for (size_t i = 0; i < count; ++i) {
    if (table[i].storage != ParameterStorage::SD) continue;
    int n = ParameterRecord::encode(table[i], values[i], data + used, sizeof(data) - used);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(data) - used) return false;
    used += n;
  }
  const uint32_t crc = checksum(data, used);
  int n = snprintf(data + used, sizeof(data) - used, "CRC32=%08lx\n",
                   static_cast<unsigned long>(crc));
  if (n < 0 || static_cast<size_t>(n) >= sizeof(data) - used) return false;
  used += n;
  File file = SD.open(path, FILE_WRITE_BEGIN);
  if (!file || !file.truncate(0)) return false;
  const size_t written = file.write(reinterpret_cast<const uint8_t *>(data), used);
  file.flush();
  file.close();
  uint32_t verifiedGeneration = 0;
  if (written != used || !readParameterFile(path, verified, verifiedGeneration) ||
      verifiedGeneration != sequence ||
      !sameSDValues(values, verified)) {
    // Remove an incomplete replacement; the other validated file remains intact.
    SD.remove(path);
    return false;
  }
  return true;
}

// Keep the exact validated old file as a rollback copy during schema migration.
bool copyValidatedSnapshot(const char *sourcePath, const char *destinationPath,
                           const double *expectedValues, uint32_t expectedGeneration) {
  File source = SD.open(sourcePath, FILE_READ);
  if (!source || source.size() == 0 || source.size() >= sizeof(fileData)) return false;
  const size_t length = source.size();
  const size_t read = source.read(reinterpret_cast<uint8_t *>(fileData), length);
  source.close();
  if (read != length) return false;
  File destination = SD.open(destinationPath, FILE_WRITE_BEGIN);
  if (!destination || !destination.truncate(0)) return false;
  const size_t written = destination.write(reinterpret_cast<const uint8_t *>(fileData), length);
  destination.flush();
  destination.close();
  uint32_t verifiedGeneration = 0;
  if (written != length ||
      !readParameterFile(destinationPath, verified, verifiedGeneration) ||
      verifiedGeneration != expectedGeneration ||
      memcmp(expectedValues, verified, count * sizeof(double))) {
    SD.remove(destinationPath);
    return false;
  }
  return true;
}

bool saveSnapshot(const double *values, bool preserveStoredFile = false) {
  if (!storageReady) return false;
  // Preserve the last successfully persisted values before replacing the primary.
  // When boot recovered from backup, leave that backup intact.
  if (activePath) {
    uint32_t previousGeneration = 0;
    if (!readParameterFile(activePath, previous, previousGeneration) ||
        previousGeneration != generation) return false;
    if (activePath != backupPath &&
        !(preserveStoredFile
              ? copyValidatedSnapshot(activePath, backupPath, previous, previousGeneration)
              : writeSnapshot(backupPath, previous, previousGeneration))) return false;
  }
  const uint32_t nextGeneration = generation + 1;
  if (!writeSnapshot(primaryPath, values, nextGeneration)) {
    // The primary may have been truncated; retry from the preserved backup.
    if (activePath) activePath = backupPath;
    return false;
  }
  activePath = primaryPath;
  generation = nextGeneration;
  loadState = preserveStoredFile ? "MIGRATED" : "SAVED";
  return true;
}

size_t parameterIndex(uint16_t key) {
  size_t i=0; while (i<count && table[i].id != key) ++i; return i;
}

bool retireMigratedSources(const double *values) {
  for (size_t i=0; i<count; ++i)
    if (table[i].storage==ParameterStorage::SD && hasEEPROMMigrationSource(table[i]))
      return saveEEPROMParameters(table,count,values,true);
  return true;
}

bool persist(const double *values, bool sd, bool ee) {
  // EEPROM first: a failed SD write leaves its old file available as migration source.
  if (ee && !saveEEPROMParameters(table, count, values)) return false;
  if (sd && !saveSnapshot(values)) return false;
  if (sd && !retireMigratedSources(values)) { loadState="MIGRATION_PENDING"; return false; }
  return true;
}

void applyCommittedParameters() {
  for (size_t i=0; i<count; ++i) {
    if (table[i].effect == ParameterEffect::Reboot && table[i].read() != candidate[i])
      rebootRequired = true;
    table[i].write(candidate[i]);
  }
  if (rebootRequired) setFlightLocked(true);
}

enum class SaveResult { Rejected, RamOnly, Saved };
SaveResult saveOne(size_t index, double value) {
  if (index == count || table[index].readOnly || !validValue(index, value)) return SaveResult::Rejected;
  if (!table[index].integer) value=static_cast<float>(value);
  if ((table[index].storage == ParameterStorage::EEPROM || table[index].id >= 1000 ||
       table[index].effect == ParameterEffect::Reboot) && !isFlightLocked()) return SaveResult::Rejected;
  for (size_t i=0; i<count; ++i) candidate[i] = table[i].read();
  candidate[index] = value;
  const size_t idIndex = parameterIndex(1001), countIndex = parameterIndex(1002);
  if (idIndex < count && countIndex < count &&
      !validAircraftConfiguration(candidate[idIndex], candidate[countIndex])) return SaveResult::Rejected;
  bool ee = table[index].storage == ParameterStorage::EEPROM;
  bool sd = !ee;
  const bool immediate = table[index].effect == ParameterEffect::Immediate;
  const SaveResult failed = immediate ? SaveResult::RamOnly : SaveResult::Rejected;
  const bool calibrationOffset = table[index].id >= 1101 && table[index].id <= 1112;
  const size_t marker = parameterIndex(1113);
  if (calibrationOffset && marker == count) return SaveResult::Rejected;
  const double oldValue = table[index].read();
  // Runtime edits are independent of persistence; validate before changing RAM.
  if (immediate) {
    table[index].write(value);
    if (calibrationOffset) { table[marker].write(0); loadImuCalibration(); }
  }
  if (calibrationOffset) {
    candidate[marker] = 0;
    ee |= table[marker].storage == ParameterStorage::EEPROM;
    sd |= table[marker].storage == ParameterStorage::SD;
    // Guard the marker in its own medium before a possible cross-medium update.
    if (table[index].storage != table[marker].storage) {
      memcpy(guarded, candidate, count*sizeof(double));
      guarded[index] = oldValue;
      if (!persist(guarded, table[marker].storage == ParameterStorage::SD,
                   table[marker].storage == ParameterStorage::EEPROM)) return failed;
      table[marker].write(0);
      loadImuCalibration();
    }
  }
  if (sd && !storageReady) return failed;
  if (!persist(candidate, sd, ee)) {
    const size_t marker=parameterIndex(1113);
    double durable;
    if (marker<count && readEEPROMParameter(table[marker],durable) && durable==0) {
      table[marker].write(0); loadImuCalibration();
    }
    return failed;
  }
  applyCommittedParameters();
  if (table[index].id >= 1101 && table[index].id <= 1112) loadImuCalibration();
  return SaveResult::Saved;
}

void queueReply(uint32_t id, const char *format, ...) {
  int prefix = snprintf(txLine, sizeof(txLine), "@COFLY\t%lu\t",
                        static_cast<unsigned long>(id));
  va_list args;
  va_start(args, format);
  int body = vsnprintf(txLine + prefix, sizeof(txLine) - prefix - 2, format, args);
  va_end(args);
  if (body < 0 || static_cast<size_t>(body) >= sizeof(txLine) - prefix - 2) {
    body = snprintf(txLine + prefix, sizeof(txLine) - prefix - 2,
                    "ERROR\tINTERNAL\tResponse too long");
  }
  txLength = prefix + body;
  txLine[txLength++] = '\n';
  txLine[txLength] = '\0';
}

void error(uint32_t id, const char *code, const char *message) {
  queueReply(id, "ERROR\t%s\t%s", code, message);
}

void handleCommand(char *line) {
  if (!strncmp(line, "GCS ", 4) || !strncmp(line, "GCS\t", 4)) {
    handleGroundStationCommand(line, txLine, sizeof(txLine));
    txLength = strlen(txLine);
    return;
  }
  char *tokens[7];
  size_t tokenCount = 0;
  char *save;
  for (char *p = strtok_r(line, " \t", &save); p; p = strtok_r(nullptr, " \t", &save)) {
    if (tokenCount == 7) { error(0, "SYNTAX", "Too many arguments"); return; }
    tokens[tokenCount++] = p;
  }
  if (tokenCount == 0) return;
  if (strcmp(tokens[0], "PARAM") || tokenCount < 2) {
    error(0, "UNKNOWN_COMMAND", "Use PARAM HELP"); return;
  }
  uint32_t id = 0;
  if (tokenCount >= 3 && !parseUnsigned(tokens[2], id)) {
    error(0, "SYNTAX", "Request ID must be uint32"); return;
  }
  if (!strcmp(tokens[1], "HELP") && tokenCount <= 3) {
    queueReply(id, "HELP\tPARAM READ [id]; PARAM SET id name value; PARAM HELP [id]");
  } else if (!strcmp(tokens[1], "READ") && tokenCount <= 3) {
    for (size_t i = 0; i < count; ++i) snapshot[i] = table[i].read();
    listId = id;
    listIndex = 0;
    listing = true;
    queueReply(id, "BEGIN\t%s\t%u\t%s\t%s\t%u\t%u\t%u\t%lu", profile,
               static_cast<unsigned>(count), storageReady ? "SD_READY" : "NO_SD",
               loadState, static_cast<unsigned>(loadStats.matched),
               static_cast<unsigned>(loadStats.defaultsUsed),
               static_cast<unsigned>(loadStats.skipped), static_cast<unsigned long>(generation));
  } else if (!strcmp(tokens[1], "SET") && tokenCount == 5) {
    size_t index = 0;
    while (index < count && strcmp(tokens[3], table[index].name)) ++index;
    if (index == count) { error(id, "UNKNOWN_PARAMETER", "Parameter not registered"); return; }
    double value;
    if (!parseValue(index, tokens[4], value)) {
      error(id, "RANGE", "Value must be finite and within parameter bounds"); return;
    }
    if (table[index].readOnly) { error(id, "READ_ONLY", "Calibration metadata is managed by firmware"); return; }
    if ((table[index].storage == ParameterStorage::EEPROM || table[index].id >= 1000 ||
         table[index].effect == ParameterEffect::Reboot) && !isFlightLocked()) {
      error(id, "LOCK_REQUIRED", "Lock aircraft before changing reboot parameters or calibration"); return;
    }
    if (table[index].storage == ParameterStorage::SD && !storageReady && table[index].effect == ParameterEffect::Reboot) {
      error(id, "NO_SD", "SD card unavailable; RAM unchanged"); return;
    }
    const SaveResult result = saveOne(index, value);
    if (result == SaveResult::Rejected) {
      error(id, "SAVE_FAILED", "Configuration invalid or persistent write/verification failed"); return;
    }
    char formatted[VALUE_SIZE];
    formatValue(index, value, formatted, sizeof(formatted));
    queueReply(id, "OK\t%s\t%s\t%s\t%s\t%lu", table[index].name,
               table[index].type(), formatted, result == SaveResult::Saved ? "SAVED" : "RAM_ONLY",
               static_cast<unsigned long>(generation));
  } else {
    error(id, "SYNTAX", "Use PARAM READ [id] or PARAM SET id name value");
  }
}
} // namespace

void initializeParameterService(bool sdReady) {
  rebootRequired = false;
  table = controlParameterTable(count);
  storageReady = sdReady && count > 0 && count <= MAX_PARAMETERS;
  if (!count || count > MAX_PARAMETERS) { count=0; return; }
  for (size_t i=0; i<count; ++i) {
    if (!table[i].id || !validParameterName(table[i].name) || !validValue(i, table[i].defaultValue)) { count=0; return; }
    for (size_t j=0; j<i; ++j)
      if (table[j].id == table[i].id || !strcmp(table[j].name, table[i].name)) { count=0; return; }
  }
  if (parameterIndex(1001)==count || parameterIndex(1002)==count || parameterIndex(1113)==count) { count=0; return; }
  activePath=nullptr; generation=0; rxLength=txLength=listIndex=0;
  rxOverflow=listing=false; loadState="DEFAULTS"; loadStats={0,count,0};
  const bool haveEEPROM = initializeEEPROMParameters();
  for (size_t i=0; i<count; ++i) {
    defaults[i] = table[i].defaultValue;
    eepromLoaded[i] = readEEPROMParameter(table[i], defaults[i]);
    if (!validValue(i, defaults[i])) { defaults[i]=table[i].defaultValue; eepromLoaded[i]=false; }
  }
  for (size_t i=0; i<count; ++i) table[i].write(defaults[i]);
  snprintf(profile, sizeof(profile), "BOARD-%08lx%08lx-" PARAM_CONTROL "-" PARAM_INDI "-" PARAM_MODEL,
           static_cast<unsigned long>(HW_OCOTP_CFG0), static_cast<unsigned long>(HW_OCOTP_CFG1));
  bool needsMigration=false;
  if (storageReady) {
    if (readParameterFile(primaryPath, candidate, generation, &needsMigration, &loadStats)) activePath=primaryPath;
    else if (readParameterFile(backupPath, candidate, generation, &needsMigration, &loadStats)) activePath=backupPath;
    // Re-read selected file: parsing another candidate overwrites restoration flags.
    if (activePath && !readParameterFile(activePath,candidate,generation,&needsMigration,&loadStats)) activePath=nullptr;
  }
  if (!activePath) { memcpy(candidate,defaults,count*sizeof(double)); memset(fileSeen,0,sizeof(fileSeen)); }
  bool completeCalibration=true;
  for (uint16_t key=1101; key<=1115; ++key)
    if (parameterIndex(key)==count) completeCalibration=false;
  for (size_t i=0; i<count; ++i) {
    if (table[i].storage==ParameterStorage::EEPROM && eepromLoaded[i]) candidate[i]=defaults[i];
    if (table[i].id>=1101 && table[i].id<=1115 && !eepromLoaded[i] && !fileSeen[i]) completeCalibration=false;
    if (table[i].id>=1101 && table[i].id<=1115 && table[i].storage==ParameterStorage::SD &&
        !fileSeen[i] && !hasEEPROMMigrationSource(table[i])) completeCalibration=false;
  }
  const size_t calibrationMarker=parameterIndex(1113);
  if (completeCalibration) {
    float offsets[12];
    for (unsigned i=0; i<12; ++i) offsets[i]=candidate[parameterIndex(1101+i)];
    if (candidate[parameterIndex(1115)] != imuCalibrationFingerprint(offsets,candidate[parameterIndex(1114)])) completeCalibration=false;
  }
  if (!completeCalibration && calibrationMarker<count) candidate[calibrationMarker]=0;
  const size_t idIndex=parameterIndex(1001), countIndex=parameterIndex(1002);
  if ((table[idIndex].storage==ParameterStorage::SD && !fileSeen[idIndex] && !hasEEPROMMigrationSource(table[idIndex])) ||
      (table[countIndex].storage==ParameterStorage::SD && !fileSeen[countIndex] && !hasEEPROMMigrationSource(table[countIndex]))) candidate[idIndex]=0;
  if (!validAircraftConfiguration(candidate[idIndex],candidate[countIndex])) {
    candidate[idIndex]=0; candidate[countIndex]=3;
  }
  const bool eeSaved=saveEEPROMParameters(table,count,candidate);
  if (!eeSaved) { candidate[idIndex]=0; candidate[parameterIndex(1113)]=0; loadState="EEPROM_ERROR"; }
  for (size_t i=0; i<count; ++i) table[i].write(candidate[i]);
  if (eeSaved) {
    loadState=(activePath || haveEEPROM) ? "LOADED" : "DEFAULTS";
    // A storage move may need SD serialization even if the old schema otherwise matches.
    bool missingSD=false;
    for (size_t i=0; i<count; ++i) if (table[i].storage==ParameterStorage::SD && !fileSeen[i] && eepromLoaded[i]) missingSD=true;
    if (storageReady && (needsMigration || missingSD)) {
      if (!saveSnapshot(candidate,true)) loadState="MIGRATION_PENDING";
      else if (!retireMigratedSources(candidate)) loadState="MIGRATION_PENDING";
    }
  }
  activateAircraftConfiguration();
  USBSerial.printf("[PARAM] %s: %s; EEPROM %s; aircraft %s/%d\n", profile,loadState,
                   eepromParameterState(),aircraftName(),aircraftCount());
}

bool saveParameterValue(uint16_t id, double value) { return saveOne(parameterIndex(id),value) == SaveResult::Saved; }
bool parameterRebootRequired() { return rebootRequired; }

bool saveImuCalibrationParameters(const float *offsets) {
  if (!table || !count || !isFlightLocked()) return false;
  for (size_t i=0; i<count; ++i) candidate[i]=table[i].read();
  bool sd=false,ee=false;
  for (unsigned j=0; j<12; ++j) {
    const size_t i=parameterIndex(1101+j);
    if (i==count || !validValue(i,offsets[j])) return false;
    sd |= table[i].storage==ParameterStorage::SD; ee |= table[i].storage==ParameterStorage::EEPROM;
  }
  const size_t marker=parameterIndex(1113), model=parameterIndex(1114), fingerprint=parameterIndex(1115);
  if (marker==count || model==count || fingerprint==count || (sd && !storageReady)) return false;
  sd |= table[marker].storage==ParameterStorage::SD || table[model].storage==ParameterStorage::SD || table[fingerprint].storage==ParameterStorage::SD;
  ee |= table[marker].storage==ParameterStorage::EEPROM || table[model].storage==ParameterStorage::EEPROM || table[fingerprint].storage==ParameterStorage::EEPROM;
  if (sd && !storageReady) return false;
  candidate[marker]=0;
  // Invalidate before writing either medium; valid is the final commit of the calibration group.
  if (!persist(candidate,table[marker].storage==ParameterStorage::SD,table[marker].storage==ParameterStorage::EEPROM)) return false;
  table[marker].write(0);
  loadImuCalibration();
  for (unsigned j=0; j<12; ++j) candidate[parameterIndex(1101+j)]=offsets[j];
  candidate[model]=1;
  candidate[fingerprint]=imuCalibrationFingerprint(offsets,1);
  if (!persist(candidate,sd,ee)) return false;
  candidate[marker]=1;
  if (!persist(candidate,table[marker].storage==ParameterStorage::SD,table[marker].storage==ParameterStorage::EEPROM)) return false;
  applyCommittedParameters();
  loadImuCalibration();
  return true;
}

void pollParameterService() {
  pollDeviceReboot(txLength != 0);
  if (deviceRebootPending() && !txLength) return;
  if (!table || count == 0 || count > MAX_PARAMETERS) return;
  if (!USBSerial) {
    // A reconnect starts with no stale request/response bytes.
    rxLength = txLength = 0;
    rxOverflow = listing = false;
    return;
  }
  if (txLength) {
    // A single line, one write, only when the USB buffer has enough room.
    if (USBSerial.availableForWrite() >= static_cast<int>(txLength)) {
      USBSerial.write(reinterpret_cast<const uint8_t *>(txLine), txLength);
      txLength = 0;
    }
    return;
  }
  if (listing) {
    if (listIndex < count) {
      const auto &p = table[listIndex];
      char value[VALUE_SIZE], low[VALUE_SIZE], high[VALUE_SIZE];
      formatValue(listIndex, snapshot[listIndex], value, sizeof(value));
      formatValue(listIndex, p.minimum, low, sizeof(low));
      formatValue(listIndex, p.maximum, high, sizeof(high));
      queueReply(listId, "VALUE\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", p.name,
                 p.type(), value, low, high, p.group, p.description,
                 p.storage==ParameterStorage::EEPROM ? "EEPROM" : "SD",
                 p.effect==ParameterEffect::Reboot ? "REBOOT" : "IMMEDIATE",
                 p.readOnly ? "READ_ONLY" : "EDITABLE");
      ++listIndex;
    } else {
      queueReply(listId, "END\t%u", static_cast<unsigned>(count));
      listing = false;
    }
    return;
  }
  // No readStringUntil()/delay(): fragments arrive over successive cycles.
  for (unsigned budget = 0; budget < 64 && USBSerial.available(); ++budget) {
    int c = USBSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxOverflow) error(0, "LINE_TOO_LONG", "Command exceeds 511 bytes");
      else {
        rxLine[rxLength] = '\0';
        handleCommand(rxLine);
      }
      rxLength = 0;
      rxOverflow = false;
      return;
    }
    if ((c < 32 && c != '\t') || c > 126) { rxOverflow = true; continue; }
    if (!rxOverflow && rxLength < sizeof(rxLine) - 1) rxLine[rxLength++] = c;
    else rxOverflow = true;
  }
}
