#pragma once
#include <stddef.h>

// Change this default, or override with -DCOFLY_PARAMETER_NAME_LIMIT=48.
// Names are ASCII bytes; changing this limit does not change the record layout.
#ifndef COFLY_PARAMETER_NAME_LIMIT
#define COFLY_PARAMETER_NAME_LIMIT 32
#endif
constexpr size_t MAX_PARAMETER_NAME = COFLY_PARAMETER_NAME_LIMIT;
constexpr size_t PARAMETER_NAME_COMPAT_LIMIT = 128;
static_assert(MAX_PARAMETER_NAME > 0 && MAX_PARAMETER_NAME <= PARAMETER_NAME_COMPAT_LIMIT,
              "Parameter name limit must be between 1 and 128");
constexpr size_t PARAMETER_VALUE_SIZE = 64;
constexpr size_t PARAMETER_RECORD_SIZE = PARAMETER_NAME_COMPAT_LIMIT + PARAMETER_VALUE_SIZE + 9;
