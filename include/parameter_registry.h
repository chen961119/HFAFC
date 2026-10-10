#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

enum class ParameterStorage { SD, EEPROM };
enum class ParameterEffect { Immediate, Reboot };
#include "parameter_storage_config.h"
inline bool validParameterName(const char *name, size_t limit = MAX_PARAMETER_NAME) {
  if (!name || !*name) return false;
  size_t n=0;
  for (; name[n]; ++n) {
    if (n == limit) return false;
    const char c=name[n];
    if (!((c>='a' && c<='z') || (c>='A' && c<='Z') ||
          (c>='0' && c<='9') || c=='_')) return false;
  }
  return true;
}

// Parameter descriptors bind directly to the controller's private variables.
// Bounds are input validation limits, not verified flight tuning ranges.
struct FlightParameter {
  uint16_t id; // Internal key for module lookup; persistent records store names.
  const char *name;
  float *value;
  double minimum;
  double maximum;
  const char *group;
  const char *description;
  int32_t *integer = nullptr;
  ParameterStorage storage;
  ParameterEffect effect;
  double defaultValue;
  bool readOnly;
  FlightParameter(uint16_t key, const char *n, float *v, double lo, double hi,
                  const char *g, const char *d, ParameterStorage s = ParameterStorage::SD,
                  ParameterEffect e = ParameterEffect::Immediate, bool ro = false)
      : id(key), name(n), value(v), minimum(lo), maximum(hi), group(g), description(d),
        storage(s), effect(e), defaultValue(*v), readOnly(ro) {}
  FlightParameter(uint16_t key, const char *n, int32_t *v, double lo, double hi,
                  const char *g, const char *d, ParameterStorage s = ParameterStorage::SD,
                  ParameterEffect e = ParameterEffect::Immediate, bool ro = false)
      : id(key), name(n), value(nullptr), minimum(lo), maximum(hi), group(g), description(d), integer(v),
        storage(s), effect(e), defaultValue(*v), readOnly(ro) {}
  const char *type() const { return integer ? "int" : "float"; }
  double read() const { return integer ? static_cast<double>(*integer) : *value; }
  void write(double v) const {
    if (integer) *integer = static_cast<int32_t>(v);
    else *value = static_cast<float>(v);
  }
};

inline bool validParameterValue(const FlightParameter &p, double value) {
  uint64_t raw; memcpy(&raw, &value, sizeof(raw));
  if ((raw & UINT64_C(0x7ff0000000000000)) == UINT64_C(0x7ff0000000000000)) return false;
  if (p.integer) {
    if (value < INT32_MIN || value > INT32_MAX || value != static_cast<int32_t>(value)) return false;
  } else {
    const float f = static_cast<float>(value);
    uint32_t bits; memcpy(&bits, &f, sizeof(bits));
    if ((bits & 0x7f800000u) == 0x7f800000u) return false;
  }
  const size_t length = strlen(p.name);
  if (length >= 4 && !strcmp(p.name + length - 4, "_rev") && value != -1 && value != 1) return false;
  return value >= p.minimum && value <= p.maximum;
}

const FlightParameter *controlParameterTable(size_t &count);
