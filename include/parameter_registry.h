#pragma once

#include <stddef.h>
#include <stdint.h>

// Parameter descriptors bind directly to the controller's private variables.
// Bounds are input validation limits, not verified flight tuning ranges.
struct FlightParameter {
  const char *name;
  float *value;
  double minimum;
  double maximum;
  const char *group;
  const char *description;
  int32_t *integer = nullptr;
  FlightParameter(const char *n, float *v, double lo, double hi,
                  const char *g, const char *d)
      : name(n), value(v), minimum(lo), maximum(hi), group(g), description(d) {}
  FlightParameter(const char *n, int32_t *v, double lo, double hi,
                  const char *g, const char *d)
      : name(n), value(nullptr), minimum(lo), maximum(hi), group(g), description(d), integer(v) {}
  const char *type() const { return integer ? "int" : "float"; }
  double read() const { return integer ? static_cast<double>(*integer) : *value; }
  void write(double v) const {
    if (integer) *integer = static_cast<int32_t>(v);
    else *value = static_cast<float>(v);
  }
};

const FlightParameter *controlParameterTable(size_t &count);
