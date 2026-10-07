// Compile-time firmware configuration.
//
// Everything here is a build-time choice (pins, task placement, feature flags).
// Tunable *navigation* parameters live in config/navigation.json and are applied
// through nav::NavigationConfig, so the firmware and the simulator share them.
#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Feature flags
// ---------------------------------------------------------------------------

/// Position source selection.
///   1 = SerialPositionProvider (mock UWB lines over USB serial) - default for
///       development before the real positioning subsystem exists
///   0 = the real integration adapter (see RealPositionProvider in
///       position_provider.cpp)
#ifndef NAVIGATION_USE_MOCK_POSITION
#define NAVIGATION_USE_MOCK_POSITION 1
#endif

/// Position transport selection (used when the real positioning server exists).
/// Takes precedence over NAVIGATION_USE_MOCK_POSITION.
///   0 = serial/UART (legacy NAVIGATION_USE_MOCK_POSITION behaviour)
///   1 = HTTP pull from the positioning server (GET /api/v1/navigation/position)
///   2 = MQTT subscribe to the positioning server push
///       (topic `<base>/navigation/position`)
#ifndef NAVIGATION_POSITION_TRANSPORT
#define NAVIGATION_POSITION_TRANSPORT 0
#endif

// ---------------------------------------------------------------------------
// Network settings (only used when NAVIGATION_POSITION_TRANSPORT is 1 or 2)
// ---------------------------------------------------------------------------

/// Positioning server base URL (no trailing slash).
#ifndef NAVIGATION_SERVER_URL
#define NAVIGATION_SERVER_URL "http://192.168.1.10:8080"
#endif

/// Wi-Fi credentials for the positioning network.
#ifndef NAVIGATION_WIFI_SSID
#define NAVIGATION_WIFI_SSID ""
#endif
#ifndef NAVIGATION_WIFI_PASSWORD
#define NAVIGATION_WIFI_PASSWORD ""
#endif

/// MQTT broker (positioning server's broker) and topic.
#ifndef NAVIGATION_MQTT_HOST
#define NAVIGATION_MQTT_HOST "192.168.1.10"
#endif
#ifndef NAVIGATION_MQTT_PORT
#define NAVIGATION_MQTT_PORT 1883
#endif
#ifndef NAVIGATION_MQTT_TOPIC
#define NAVIGATION_MQTT_TOPIC "uwb/home/navigation/position"
#endif

/// Motor output mode.
///   0 = NullMotorDriver: no hardware, state machine still fully exercised
///   1 = H-bridge driver on LEDC PWM
#define NAVIGATION_ENABLE_MOTOR_OUTPUT 1

/// Publish a telemetry line every NAVIGATION_TELEMETRY_PERIOD_MS.
#ifndef NAVIGATION_ENABLE_TELEMETRY
#define NAVIGATION_ENABLE_TELEMETRY 1
#endif

/// Accept debug commands (CMD: lines) on the console UART.
#define NAVIGATION_ENABLE_SERIAL_COMMANDS 1

/// Build a JSON configuration blob into the firmware image so the device does
/// not depend on a filesystem or a server to start navigating.
#define NAVIGATION_BUILTIN_CONFIG 1

// ---------------------------------------------------------------------------
// Task and timing configuration
// ---------------------------------------------------------------------------

/// Navigation control rate.  Matches control.navigation_rate_hz in
/// config/navigation.json; the control loop measures the real dt anyway.
constexpr uint32_t NAVIGATION_PERIOD_MS = 50;   // 20 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;   // 5 Hz
constexpr uint32_t POSITION_PERIOD_MS = 20;     // 50 Hz poll of the RX queue

/// Core pinning (ESP32-S3 is dual core).  Core 1 runs the time-critical
/// navigation loop, core 0 handles I/O and telemetry, so a slow serial write
/// can never delay motor control.
constexpr int NAVIGATION_TASK_CORE = 1;
constexpr int COMMUNICATION_TASK_CORE = 0;

constexpr uint32_t NAVIGATION_TASK_STACK = 8192;
constexpr uint32_t COMMUNICATION_TASK_STACK = 4096;
constexpr uint32_t TELEMETRY_TASK_STACK = 6144;

constexpr UBaseType_t NAVIGATION_TASK_PRIORITY = 5;   // highest: control loop
constexpr UBaseType_t POSITION_TASK_PRIORITY = 4;
constexpr UBaseType_t TELEMETRY_TASK_PRIORITY = 2;
constexpr UBaseType_t COMMAND_TASK_PRIORITY = 3;

/// Safety watchdog: if the navigation loop stalls for longer than this, the
/// watchdog fires and the device reboots rather than driving blind.
constexpr uint32_t NAVIGATION_WATCHDOG_TIMEOUT_S = 5;

/// Maximum number of position samples buffered between the reader and the
/// navigation loop.  Bounded so a stalled consumer cannot grow the queue.
constexpr uint32_t POSITION_QUEUE_LENGTH = 8;

/// Serial line buffer for incoming JSON / commands (bounded, no reallocation).
constexpr uint32_t SERIAL_LINE_BUFFER = 320;

/// Telemetry output buffer (single line of JSON).
constexpr uint32_t TELEMETRY_BUFFER = 1200;

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

inline constexpr const char* TROLLEY_ID = "TROLLEY_01";
inline constexpr const char* FIRMWARE_VERSION = "1.0.0";
