// Serial output arbitration for newline-delimited dashboard traffic.
#pragma once

#include <Arduino.h>
#include <freertos/semphr.h>

namespace firmware {

void initialiseSerialOutput();
SemaphoreHandle_t serialOutputMutex();

class SerialOutputGuard {
public:
    SerialOutputGuard() : mutex_(serialOutputMutex()) {
        locked_ = mutex_ != nullptr && xSemaphoreTake(mutex_, portMAX_DELAY) == pdTRUE;
    }
    ~SerialOutputGuard() {
        if (locked_) xSemaphoreGive(mutex_);
    }
    SerialOutputGuard(const SerialOutputGuard&) = delete;
    SerialOutputGuard& operator=(const SerialOutputGuard&) = delete;

private:
    SemaphoreHandle_t mutex_{nullptr};
    bool locked_{false};
};

inline void serialPrintLocked(const char* text) {
    SerialOutputGuard guard;
    Serial.print(text);
}

inline void serialPrintlnLocked(const char* text = nullptr) {
    SerialOutputGuard guard;
    if (text == nullptr) Serial.println(); else Serial.println(text);
}

template <typename... Args>
inline void serialPrintfLocked(const char* format, Args... args) {
    SerialOutputGuard guard;
    Serial.printf(format, args...);
}

}  // namespace firmware
