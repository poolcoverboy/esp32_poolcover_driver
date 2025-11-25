#pragma once
#include <Arduino.h>

enum class LogLevel : uint8_t {
  DEBUG = 0,
  INFO  = 1,
  WARN  = 2,
  ERROR = 3,
};

inline const char* logLevelName(LogLevel level) {
  switch (level) {
    case LogLevel::DEBUG: return "debug";
    case LogLevel::INFO:  return "info";
    case LogLevel::WARN:  return "warn";
    case LogLevel::ERROR: return "error";
    default: return "unknown";
  }
}

inline char logLevelCode(LogLevel level) {
  switch (level) {
    case LogLevel::DEBUG: return 'D';
    case LogLevel::INFO:  return 'I';
    case LogLevel::WARN:  return 'W';
    case LogLevel::ERROR: return 'E';
    default: return '?';
  }
}

inline LogLevel maxLogLevel(LogLevel a, LogLevel b) {
  return (static_cast<uint8_t>(a) >= static_cast<uint8_t>(b)) ? a : b;
}
