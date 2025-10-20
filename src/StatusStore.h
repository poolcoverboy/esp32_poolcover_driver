#pragma once
#include <Arduino.h>

class StatusStore {
public:
  struct Entry {
    String label;
    String value;
  };

  static constexpr uint8_t MAX_ITEMS = 10;

  void configure(const char* labels[], uint8_t count) {
    _count = min<uint8_t>(count, MAX_ITEMS);
    for (uint8_t i = 0; i < _count; ++i) {
      _entries[i].label = labels[i];
      _entries[i].value = "";
    }
    _dirty = true;
  }

  bool setStatus(const String& label, const String& value) {
    for (uint8_t i = 0; i < _count; ++i) {
      if (_entries[i].label.equalsIgnoreCase(label)) {
        if (_entries[i].value == value) return false;
        _entries[i].value = value;
        _dirty = true;
        return true;
      }
    }
    if (_count < MAX_ITEMS) {
      _entries[_count].label = label;
      _entries[_count].value = value;
      ++_count;
      _dirty = true;
      return true;
    }
    return false;
  }

  bool setStatus(const char* label, const String& value) {
    return setStatus(String(label), value);
  }

  bool takeDirty() {
    bool wasDirty = _dirty;
    _dirty = false;
    return wasDirty;
  }

  bool dirty() const { return _dirty; }

  uint8_t count() const { return _count; }

  const Entry& entry(uint8_t index) const { return _entries[index]; }

private:
  Entry _entries[MAX_ITEMS];
  uint8_t _count = 0;
  bool _dirty = false;
};
