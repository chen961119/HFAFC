#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <deque>
#include <string>

#ifdef _WIN32
inline char *strtok_r(char *text, const char *delimiters, char **state) {
  char *start = text ? text : *state;
  start += strspn(start, delimiters);
  if (!*start) { *state = start; return nullptr; }
  char *end = start + strcspn(start, delimiters);
  if (*end) *end++ = '\0';
  *state = end;
  return start;
}
#endif

struct MockSerial {
  bool connected = true;
  int capacity = 512;
  std::deque<unsigned char> input;
  std::string output;
  explicit operator bool() const { return connected; }
  int available() const { return static_cast<int>(input.size()); }
  int availableForWrite() const { return capacity; }
  int read() { int result = input.front(); input.pop_front(); return result; }
  size_t write(const uint8_t *bytes, size_t count) {
    output.append(reinterpret_cast<const char *>(bytes), count);
    return count;
  }
  void println(const char *text) { output += text; output += '\n'; }
  void printf(const char *format, ...) {
    char buffer[512];
    va_list args;
    va_start(args, format);
    int count = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    output.append(buffer, count);
  }
  void feed(const std::string &text) { input.insert(input.end(), text.begin(), text.end()); }
};
extern MockSerial Serial;
