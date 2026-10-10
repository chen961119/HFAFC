#pragma once
#include "parameter_registry.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

// Shared ASCII record codec for SD and EEPROM. Values round-trip float32 exactly.
namespace ParameterRecord {
inline bool finiteFloat(float value) {
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return (bits & 0x7f800000UL) != 0x7f800000UL;
}

inline bool parseFloat(const char *text, float &value) {
  char *end;
  errno = 0;
  value = strtof(text, &end);
  // libc may set ERANGE for representable subnormal float32 values.
  uint32_t bits;memcpy(&bits,&value,sizeof(bits));
  return end != text && *end == '\0' && finiteFloat(value) &&
      (errno != ERANGE || (bits & 0x7fffffffu) != 0);
}

inline bool parseUnsigned(const char *text, uint32_t &value) {
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

inline bool parseValue(const FlightParameter &p, const char *text, double &value) {
  if (!p.integer) {
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
  return validParameterValue(p, value);
}

inline void formatValue(const FlightParameter &p, double value, char *text, size_t capacity) {
  if (p.integer) {
    snprintf(text, capacity, "%ld", static_cast<long>(value));
    return;
  }
  const float original = static_cast<float>(value);
  // Shortest decimal that recovers exactly the same float32 bits.
  char shortText[32];
  for (int digits = 1; digits <= 9; ++digits) {
    snprintf(shortText, sizeof(shortText), "%.*g", digits, value);
    float parsed;
    if (parseFloat(shortText, parsed) && !memcmp(&parsed, &original, sizeof(float))) break;
  }
  const char *exponent = strchr(shortText, 'e');
  if (!exponent) {
    snprintf(text, capacity, "%s", shortText);
    return;
  }
  // Expand the rounded decimal text, preserving its digits without rounding
  // the binary float again. No scientific notation in replies or SD files.
  const bool negative = shortText[0] == '-';
  const char *start = shortText + (negative ? 1 : 0);
  char digits[16];
  int length = 0, point = 0;
  bool afterPoint = false;
  for (const char *p = start; p < exponent; ++p) {
    if (*p == '.') { afterPoint = true; continue; }
    digits[length++] = *p;
    if (!afterPoint) ++point;
  }
  point += atoi(exponent + 1);
  size_t used = 0;
  auto append = [&](char c) { if (used + 1 < capacity) text[used++] = c; };
  if (negative) append('-');
  if (point <= 0) {
    append('0'); append('.');
    for (int i = 0; i < -point; ++i) append('0');
  }
  for (int i = 0; i < length; ++i) {
    if (i > 0 && i == point) append('.');
    append(digits[i]);
  }
  for (int i = length; i < point; ++i) append('0');
  text[used] = '\0';
}


inline bool split(char *line, char *&name, char *&type, char *&value) {
  char *colon=strchr(line, ':');char *equals=strchr(line, '=');
  if (!colon || !equals || colon>=equals || !equals[1]) return false;
  *colon=0;*equals=0;name=line;type=colon+1;value=equals+1;
  return validParameterName(name, PARAMETER_NAME_COMPAT_LIMIT) &&
         (!strcmp(type,"float") || !strcmp(type,"int"));
}
inline int encode(const FlightParameter &p,double value,char *out,size_t capacity) {
  if (!validParameterName(p.name) || !validParameterValue(p,value)) return -1;
  char text[PARAMETER_VALUE_SIZE];formatValue(p,value,text,sizeof(text));
  const int n=snprintf(out,capacity,"%s:%s=%s\n",p.name,p.type(),text);
  return n<0 || static_cast<size_t>(n)>=capacity ? -1 : n;
}
} // namespace ParameterRecord
