#pragma once
#include <Arduino.h>
#include <deque>
#include "Logging.h"

class RingLogger {
public:
  struct Entry {
    String line;
    LogLevel level;
  };

  explicit RingLogger(size_t maxBytes = 6 * 1024) : _maxBytes(maxBytes) {}

  void append(const String& line, LogLevel level) {
    String s = line;
    if (!s.endsWith("\n")) s += "\n";
    const size_t len = s.length();
    while ((_totalBytes + len) > _maxBytes && !_lines.empty()) {
      _totalBytes -= _lines.front().line.length();
      _lines.pop_front();
    }
    _totalBytes += len;
    _lines.push_back(Entry{std::move(s), level});
    ++_revision;
  }

  String blob(LogLevel minLevel = LogLevel::DEBUG) const {
    String out;
    out.reserve(_totalBytes + 16);
    for (const auto& entry : _lines) {
      if (entry.level >= minLevel) {
        out += entry.line;
      }
    }
    return out;
  }

  size_t sizeBytes() const { return _totalBytes; }
  size_t lineCount() const { return _lines.size(); }
  uint32_t revision() const { return _revision; }

  void clear() {
    _lines.clear();
    _totalBytes = 0;
    _revision = 0;
  }

private:
  std::deque<Entry> _lines;
  size_t _maxBytes;
  size_t _totalBytes{0};
  uint32_t _revision{0};
};
