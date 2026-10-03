#include "navigation/logging.hpp"

namespace nav {

namespace {
LogSink g_sink = nullptr;
LogLevel g_level = LogLevel::INFO;
}  // namespace

void setLogSink(LogSink sink) { g_sink = sink; }

void setLogLevel(LogLevel level) { g_level = level; }

LogLevel logLevel() { return g_level; }

namespace detail {
void logV(LogLevel level, const char* tag, const char* format, va_list args) {
    if (g_sink == nullptr || level > g_level) {
        return;
    }
    char message[160];
    // vsnprintf never writes more than the buffer size and always terminates.
    std::vsnprintf(message, sizeof(message), format, args);
    g_sink(level, tag == nullptr ? "" : tag, message);
}
}  // namespace detail

void logMessage(LogLevel level, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    detail::logV(level, tag, format, args);
    va_end(args);
}

}  // namespace nav
