#include "serial_output.hpp"

namespace firmware {
namespace {
SemaphoreHandle_t g_serial_output_mutex = nullptr;
}

void initialiseSerialOutput() {
    if (g_serial_output_mutex == nullptr) {
        g_serial_output_mutex = xSemaphoreCreateMutex();
    }
}

SemaphoreHandle_t serialOutputMutex() { return g_serial_output_mutex; }

}  // namespace firmware
