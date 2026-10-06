#include "serial_ports.h"
#include "parameter_service.h"
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
constexpr size_t MAX_PARAMETERS = 64;
constexpr size_t LINE_SIZE = 256;
constexpr size_t FILE_SIZE = 8192;
const char *const paths[] = {"params0.cfg", "params1.cfg"};

#if defined APLANE
#define PARAM_AIRCRAFT "A"
#elif defined BPLANE
#define PARAM_AIRCRAFT "B"
#elif defined CPLANE
#define PARAM_AIRCRAFT "C"
#elif defined DPLANE
#define PARAM_AIRCRAFT "D"
#elif defined EPLANE
#define PARAM_AIRCRAFT "E"
#elif defined FPLANE
#define PARAM_AIRCRAFT "F"
#elif defined GPLANE
#define PARAM_AIRCRAFT "G"
#else
#error "Parameter storage requires an aircraft identity"
#endif
#if defined TEAM
#define PARAM_CONTROL "TEAM"
#else
#define PARAM_CONTROL "SINGLE"
#endif
#if defined SEVENPLANE
#define PARAM_SIZE "7"
#elif defined FIVEPLANE
#define PARAM_SIZE "5"
#elif defined FOURPLANE
#define PARAM_SIZE "4"
#elif defined THREEPLANE
#define PARAM_SIZE "3"
#else
#define PARAM_SIZE "1"
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
const char profile[] = PARAM_AIRCRAFT "-" PARAM_CONTROL "-" PARAM_SIZE "-"
                       PARAM_INDI "-" PARAM_MODEL;

const FlightParameter *table;
size_t count;
bool storageReady;
int activeSlot = -1;
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
const char *loadState = "DEFAULTS";

// Bit inspection is intentional: the project enables -ffast-math, which can
// optimize away ordinary isfinite() checks. Reject NaN/Inf before comparisons.
bool finiteFloat(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7f800000UL) != 0x7f800000UL;
}

bool parseFloat(const char *text, float &value) {
  char *end;
  errno = 0;
  value = strtof(text, &end);
  return end != text && *end == '\0' && errno != ERANGE && finiteFloat(value);
}

bool parseUnsigned(const char *text, uint32_t &value) {
  if (!*text) return false;
  uint64_t parsed = 0;
  for (const char *p = text; *p; ++p) {
    if (*p < '0' || *p > '9') return false;
    parsed = parsed * 10 + (*p - '0');
    if (parsed > UINT32_MAX) return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool validValue(size_t index, double value) {
  return (table[index].integer || finiteFloat(static_cast<float>(value))) && value >= table[index].minimum &&
         value <= table[index].maximum;
}

bool parseValue(size_t index, const char *text, double &value) {
  if (!table[index].integer) {
    float parsed;
    if (!parseFloat(text, parsed)) return false;
    value = parsed;
  } else {
    const char *digits = text;
    if (*digits == '+' || *digits == '-') ++digits;
    if (!*digits) return false;
    for (const char *p = digits; *p; ++p)
      if (*p < '0' || *p > '9') return false;
    char *end;
    errno = 0;
    const long long parsed = strtoll(text, &end, 10);
    if (errno == ERANGE || *end || parsed < INT32_MIN || parsed > INT32_MAX) return false;
    value = static_cast<double>(parsed);
  }
  return validValue(index, value);
}

void formatValue(size_t index, double value, char *text, size_t capacity) {
  if (table[index].integer) {
    snprintf(text, capacity, "%ld", static_cast<long>(value));
    return;
  }
  const float original = static_cast<float>(value);
  // Shortest decimal that recovers exactly the same float32 bits.
  for (int digits = 1; digits <= 9; ++digits) {
    snprintf(text, capacity, "%.*g", digits, value);
    float parsed;
    if (parseFloat(text, parsed) && !memcmp(&parsed, &original, sizeof(float))) return;
  }
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

// Exact names/order/count plus CRC prevent partial or mixed-profile loads.
bool readSlot(int slot, double *values, uint32_t &sequence) {
  char data[FILE_SIZE];
  File file = SD.open(paths[slot], FILE_READ);
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
  snprintf(header, sizeof(header), "HFAFC_PARAMS_V1 %s", profile);
  const bool legacy = line && !strcmp(line, header);
  snprintf(header, sizeof(header), "HFAFC_PARAMS_V2 %s", profile);
  if (!legacy && (!line || strcmp(line, header))) return false;
  line = strtok_r(nullptr, "\n", &save);
  if (!line || strncmp(line, "GEN=", 4) || !parseUnsigned(line + 4, sequence))
    return false;
  for (size_t i = 0; i < count; ++i) {
    line = strtok_r(nullptr, "\n", &save);
    if (!line && legacy && table[i].integer) {
      // V1 contained the original float prefix; new integer settings use defaults.
      for (size_t j = i; j < count; ++j) {
        if (!table[j].integer) return false;
        values[j] = defaults[j];
      }
      return true;
    }
    if (!line) return false;
    char *equals = strchr(line, '=');
    if (!equals) return false;
    *equals = '\0';
    char key[128];
    snprintf(key, sizeof(key), legacy ? "%s" : "%s:%s", table[i].name, table[i].type());
    if (strcmp(line, key) || !parseValue(i, equals + 1, values[i])) return false;
  }
  return strtok_r(nullptr, "\n", &save) == nullptr;
}

bool saveSnapshot(const double *values) {
  if (!storageReady) return false;
  char data[FILE_SIZE];
  const uint32_t nextGeneration = generation + 1;
  size_t used = snprintf(data, sizeof(data), "HFAFC_PARAMS_V2 %s\nGEN=%lu\n",
                         profile, static_cast<unsigned long>(nextGeneration));
  for (size_t i = 0; i < count; ++i) {
    char value[32];
    formatValue(i, values[i], value, sizeof(value));
    int n = snprintf(data + used, sizeof(data) - used, "%s:%s=%s\n",
                     table[i].name, table[i].type(), value);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(data) - used) return false;
    used += n;
  }
  const uint32_t crc = checksum(data, used);
  int n = snprintf(data + used, sizeof(data) - used, "CRC32=%08lx\n",
                   static_cast<unsigned long>(crc));
  if (n < 0 || static_cast<size_t>(n) >= sizeof(data) - used) return false;
  used += n;
  const int target = activeSlot == 0 ? 1 : 0;
  File file = SD.open(paths[target], FILE_WRITE_BEGIN);
  if (!file || !file.truncate(0)) return false;
  const size_t written = file.write(reinterpret_cast<const uint8_t *>(data), used);
  file.flush();
  file.close();
  double verified[MAX_PARAMETERS];
  uint32_t verifiedGeneration = 0;
  if (written != used || !readSlot(target, verified, verifiedGeneration) ||
      verifiedGeneration != nextGeneration ||
      memcmp(values, verified, count * sizeof(double))) {
    // Keep the previous slot intact. A torn new file fails CRC on next boot.
    SD.remove(paths[target]);
    return false;
  }
  activeSlot = target;
  generation = nextGeneration;
  loadState = "SAVED";
  return true;
}

void queueReply(uint32_t id, const char *format, ...) {
  int prefix = snprintf(txLine, sizeof(txLine), "@HFAFC\t%lu\t",
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
    queueReply(id, "BEGIN\t%s\t%u\t%s\t%s", profile,
               static_cast<unsigned>(count), storageReady ? "SD_READY" : "NO_SD",
               loadState);
  } else if (!strcmp(tokens[1], "SET") && tokenCount == 5) {
    size_t index = 0;
    while (index < count && strcmp(tokens[3], table[index].name)) ++index;
    if (index == count) { error(id, "UNKNOWN_PARAMETER", "Parameter not registered"); return; }
    double value;
    if (!parseValue(index, tokens[4], value)) {
      error(id, "RANGE", "Value must be finite and within parameter bounds"); return;
    }
    if (!storageReady) { error(id, "NO_SD", "SD card unavailable; RAM unchanged"); return; }
    double candidate[MAX_PARAMETERS];
    for (size_t i = 0; i < count; ++i) candidate[i] = table[i].read();
    candidate[index] = value;
    if (!saveSnapshot(candidate)) {
      error(id, "SD_WRITE", "SD save/verification failed; RAM unchanged"); return;
    }
    table[index].write(value);
    char formatted[32];
    formatValue(index, value, formatted, sizeof(formatted));
    queueReply(id, "OK\t%s\t%s\t%s\tSAVED", table[index].name,
               table[index].type(), formatted);
  } else {
    error(id, "SYNTAX", "Use PARAM READ [id] or PARAM SET id name value");
  }
}
} // namespace

void initializeParameterService(bool sdReady) {
  table = controlParameterTable(count);
  storageReady = sdReady && count > 0 && count <= MAX_PARAMETERS;
  activeSlot = -1;
  generation = 0;
  rxLength = txLength = listIndex = 0;
  rxOverflow = listing = false;
  loadState = "DEFAULTS";
  if (!storageReady) {
    USBSerial.println("[PARAM] SD unavailable; using compiled defaults"); return;
  }
  double first[MAX_PARAMETERS], second[MAX_PARAMETERS];
  for (size_t i = 0; i < count; ++i) defaults[i] = table[i].read();
  uint32_t sequence0 = 0, sequence1 = 0;
  bool valid0 = readSlot(0, first, sequence0);
  bool valid1 = readSlot(1, second, sequence1);
  // Sequence comparison handles uint32 wrap without signed overflow.
  const uint32_t difference = sequence1 - sequence0;
  if (valid1 && (!valid0 || (difference != 0 && difference < 0x80000000UL))) {
    activeSlot = 1;
    generation = sequence1;
    for (size_t i = 0; i < count; ++i) table[i].write(second[i]);
  } else if (valid0) {
    activeSlot = 0;
    generation = sequence0;
    for (size_t i = 0; i < count; ++i) table[i].write(first[i]);
  }
  if (activeSlot >= 0) loadState = "LOADED";
  USBSerial.printf("[PARAM] %s: %s, %u parameters\n", profile, loadState,
                static_cast<unsigned>(count));
}

void pollParameterService() {
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
      char value[32], low[32], high[32];
      formatValue(listIndex, snapshot[listIndex], value, sizeof(value));
      formatValue(listIndex, p.minimum, low, sizeof(low));
      formatValue(listIndex, p.maximum, high, sizeof(high));
      queueReply(listId, "VALUE\t%s\t%s\t%s\t%s\t%s\t%s\t%s", p.name,
                 p.type(), value, low, high, p.group, p.description);
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
      if (rxOverflow) error(0, "LINE_TOO_LONG", "Command exceeds 255 bytes");
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
