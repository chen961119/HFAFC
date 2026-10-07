#pragma once
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>

constexpr int FILE_READ = 0;
constexpr int FILE_WRITE_BEGIN = 2;

struct MockDisk {
  std::map<std::string, std::string> files;
  bool failOpen = false;
  int writesBeforeFailure = -1;
  bool shortWrite = false;
  bool shortRead = false;
  bool corruptRead = false;
  bool failTruncate = false;
};
extern MockDisk disk;

struct File {
  std::string *data = nullptr;
  bool writing = false;
  explicit operator bool() const { return data != nullptr; }
  size_t size() const { return data->size(); }
  size_t read(uint8_t *destination, size_t length) {
    length = std::min(length, data->size());
    if (disk.shortRead && length) --length;
    memcpy(destination, data->data(), length);
    if (disk.corruptRead && length) destination[0] ^= 1;
    return length;
  }
  size_t write(const uint8_t *source, size_t length) {
    if (disk.shortWrite || disk.writesBeforeFailure == 0) length /= 2;
    if (disk.writesBeforeFailure > 0) --disk.writesBeforeFailure;
    data->assign(reinterpret_cast<const char *>(source), length);
    return length;
  }
  bool truncate(size_t = 0) {
    if (disk.failTruncate) return false;
    data->clear();
    return true;
  }
  void flush() {}
  void close() {}
};

struct MockSD {
  File open(const char *path, int mode) {
    if (mode == FILE_WRITE_BEGIN) {
      if (disk.failOpen) return {};
      return {&disk.files[path], true};
    }
    auto found = disk.files.find(path);
    if (found == disk.files.end()) return {};
    return {&found->second, false};
  }
  bool remove(const char *path) { return disk.files.erase(path) != 0; }
};
extern MockSD SD;
