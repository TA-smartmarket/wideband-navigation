// Tiny compile-time-filtered logger.
//
// The firmware build defines NAV_LOG_LEVEL; the simulator (host build) leaves
// the default.  Logging is deliberately allocation-free: messages are built in
// a fixed 160-byte buffer and handed to a sink callback, so nothing here
// depends on Arduino's Serial or on iostreams.
#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>

namespace nav {

enum class LogLevel : uint8_t {
    ERROR = 0,
    WARN = 1,
    INFO = 2,
    DEBUG = 3,
    TRACE = 4,
};

/// Callback invoked for every emitted line (without trailing newline).
using LogSink = void (*)(LogLevel level, const char* tag, const char* message);

/// Install the sink; nullptr disables output.
void setLogSink(LogSink sink);

/// Runtime level filter (a message is emitted when level <= currentLevel()).
void setLogLevel(LogLevel level);
LogLevel logLevel();

/// Tagged, printf-style logging.  Truncates to the internal buffer.
void logMessage(LogLevel level, const char* tag, const char* format, ...);

namespace detail {
void logV(LogLevel level, const char* tag, const char* format, va_list args);
}

#define NAV_LOG_ERROR(tag, ...) ::nav::logMessage(::nav::LogLevel::ERROR, tag, __VA_ARGS__)
#define NAV_LOG_WARN(tag, ...) ::nav::logMessage(::nav::LogLevel::WARN, tag, __VA_ARGS__)
#define NAV_LOG_INFO(tag, ...) ::nav::logMessage(::nav::LogLevel::INFO, tag, __VA_ARGS__)
#define NAV_LOG_DEBUG(tag, ...) ::nav::logMessage(::nav::LogLevel::DEBUG, tag, __VA_ARGS__)
#define NAV_LOG_TRACE(tag, ...) ::nav::logMessage(::nav::LogLevel::TRACE, tag, __VA_ARGS__)

}  // namespace nav
