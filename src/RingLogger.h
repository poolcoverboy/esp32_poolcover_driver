#pragma once
#include <Arduino.h>
#include <deque>

class RingLogger {
public:
  explicit RingLogger(size_t maxBytes = 6 * 1024) : _maxBytes(maxBytes) {}

  // Append one full line (adds newline if missing)
  void append(const String& line) {
    String s = line;
    if (!s.endsWith("\n")) s += "\n";
    const size_t len = s.length();
    // Evict oldest until it fits
    while ((_totalBytes + len) > _maxBytes && !_lines.empty()) {
      _totalBytes -= _lines.front().length();
      _lines.pop_front();
    }
    _lines.push_back(std::move(s));
    _totalBytes += len;
    _dirty = true;
  }

  // Compose a single multi-line blob; cheap if unchanged
  String blob() {
    if (!_dirty && _cachedValid) return _cached;
    String out; out.reserve(_totalBytes + 16);
    for (auto &l : _lines) out += l;
    _cached = std::move(out);
    _cachedValid = true;
    _dirty = false;
    return _cached;
  }

  void clear() {
    _lines.clear(); _totalBytes = 0;
    _dirty = true; _cachedValid = false; _cached = "";
  }

  size_t sizeBytes() const { return _totalBytes; }

private:
  std::deque<String> _lines;
  size_t _maxBytes;
  size_t _totalBytes{0};
  bool _dirty{true};
  bool _cachedValid{false};
  String _cached;
};
