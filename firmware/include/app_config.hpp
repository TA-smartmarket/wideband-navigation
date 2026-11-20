// Compile-time firmware configuration.
//
// Everything here is a build-time choice (pins, task placement, feature flags).
// Tunable *navigation* parameters live in config/navigation.json and are applied
// through nav::NavigationConfig, so the firmware and the simulator share them.
#pragma once

#include <cstdint>

// Optional per-machine credentials and broker address.  This file is ignored
// by Git; copy local_secrets.example.hpp to local_secrets.hpp before a local
// MQTT build.
#if __has_include("local_secrets.hpp")
#include "local_secrets.hpp"
#endif

// ---------------------------------------------------------------------------
// Feature flags
// ---------------------------------------------------------------------------

/// Position source selection.
///   1 = SerialPositionProvider (mock UWB lines over USB serial) - default for
///       development before the real positioning subsystem exists
///   0 = the real integration adapter (see RealPositionProvider in
///       position_provider.cpp)
#ifndef NAVIGATION_USE_MOCK_POSITION
#define NAVIGATION_USE_MOCK_POSITION 0
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

/// Publish existing NDJSON telemetry to a local MQTT broker.
#ifndef NAVIGATION_ENABLE_MQTT_TELEMETRY
#define NAVIGATION_ENABLE_MQTT_TELEMETRY 0
#endif

/// Accept a small, safety-bounded subset of dashboard commands from the
/// trolley's MQTT command topic. Keep this disabled unless the broker is on
/// an isolated, trusted LAN (or protected with credentials and ACLs).
#ifndef NAVIGATION_ENABLE_MQTT_CONTROL
#define NAVIGATION_ENABLE_MQTT_CONTROL 0
#endif

/// Runs the MQTT client's network state machine. This is separate from
/// telemetry formatting so a bench build can diagnose Wi-Fi independently.
#ifndef NAVIGATION_ENABLE_MQTT_SERVICE
#define NAVIGATION_ENABLE_MQTT_SERVICE 1
#endif

/// Set to 0 only for an explicitly monitor-only bench build. It keeps TB6600
/// outputs inhibited while serial/MQTT telemetry can be inspected.
#ifndef NAVIGATION_ENABLE_STEPPER_TIMER
#define NAVIGATION_ENABLE_STEPPER_TIMER 1
#endif

/// Keep enabled for deployed motion firmware. It may be disabled temporarily
/// while isolating a board-level boot/reset fault.
#ifndef NAVIGATION_ENABLE_TASK_WATCHDOG
#define NAVIGATION_ENABLE_TASK_WATCHDOG 1
#endif

/// Enables the AS5600 acquisition and wheel-velocity PID task when the
/// navigation configuration requests it. Disable only for an explicit
/// open-loop bench profile where encoder hardware is not present.
#ifndef NAVIGATION_ENABLE_WHEEL_CLOSED_LOOP
#define NAVIGATION_ENABLE_WHEEL_CLOSED_LOOP 1
#endif

/// Maximum accepted gap between encoder odometry and a fresh UWB fix.
/// Calibrate against measured UWB noise and wheel slip on the actual trolley.
#ifndef NAVIGATION_MAX_UWB_ODOMETRY_GAP_M
#define NAVIGATION_MAX_UWB_ODOMETRY_GAP_M 0.50f
#endif

/// Allows a diagnostic build to isolate NVS/Preferences from network startup.
#ifndef NAVIGATION_ENABLE_PERSISTED_CONFIG
#define NAVIGATION_ENABLE_PERSISTED_CONFIG 1
#endif

#ifndef NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC
#define NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC "smart_trolley"
#endif

/// Optional broker credentials. Leave both empty only on an isolated trusted
/// development LAN.
#ifndef NAVIGATION_MQTT_USERNAME
#define NAVIGATION_MQTT_USERNAME ""
#endif
#ifndef NAVIGATION_MQTT_PASSWORD
#define NAVIGATION_MQTT_PASSWORD ""
#endif

/// Motor output mode.
///   0 = NullMotorDriver: no hardware, state machine still fully exercised
///   1 = H-bridge driver on LEDC PWM
#ifndef NAVIGATION_ENABLE_MOTOR_OUTPUT
#define NAVIGATION_ENABLE_MOTOR_OUTPUT 1
#endif

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
constexpr uint32_t WHEEL_CONTROL_PERIOD_MS = 10;  // 100 Hz
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;   // 5 Hz
constexpr uint32_t POSITION_PERIOD_MS = 20;     // 50 Hz poll of the RX queue

/// Core pinning (ESP32-S3 is dual core).  Core 1 runs the time-critical
/// navigation loop, core 0 handles I/O and telemetry, so a slow serial write
/// can never delay motor control.
constexpr int NAVIGATION_TASK_CORE = 1;
constexpr int COMMUNICATION_TASK_CORE = 0;

constexpr uint32_t NAVIGATION_TASK_STACK = 8192;
constexpr uint32_t WHEEL_CONTROL_TASK_STACK = 6144;
constexpr uint32_t COMMUNICATION_TASK_STACK = 4096;
// JSON parsing plus configuration replies exceeded the generic communication
// stack on the physical ESP32-S3 (TaskCommands stack-canary reset). Keep the
// command parser on its own measured budget instead of sharing the lightweight
// position-task allocation.
constexpr uint32_t COMMAND_TASK_STACK = 8192;
constexpr uint32_t TELEMETRY_TASK_STACK = 6144;
/// MQTT packet assembly deliberately lives outside the command parser task;
/// its bounded receive/publish buffers require a larger I/O stack.
constexpr uint32_t MQTT_TASK_STACK = 8192;

constexpr UBaseType_t NAVIGATION_TASK_PRIORITY = 5;   // highest: control loop
constexpr UBaseType_t WHEEL_CONTROL_TASK_PRIORITY = 6;
constexpr UBaseType_t POSITION_TASK_PRIORITY = 4;
constexpr UBaseType_t TELEMETRY_TASK_PRIORITY = 2;
constexpr UBaseType_t COMMAND_TASK_PRIORITY = 3;
constexpr UBaseType_t MQTT_TASK_PRIORITY = 1;

/// Safety watchdog: if the navigation loop stalls for longer than this, the
/// watchdog fires and the device reboots rather than driving blind.
constexpr uint32_t NAVIGATION_WATCHDOG_TIMEOUT_S = 5;

/// Maximum number of position samples buffered between the reader and the
/// navigation loop.  Bounded so a stalled consumer cannot grow the queue.
constexpr uint32_t POSITION_QUEUE_LENGTH = 8;

/// Serial line buffer for incoming JSON / commands (bounded, no reallocation).
constexpr uint32_t SERIAL_LINE_BUFFER = 320;

/// Manual motion commands are valid only while the dashboard continues sending
/// its heartbeat.  USB loss therefore stops a manually driven trolley.
constexpr uint32_t MANUAL_COMMAND_TIMEOUT_MS = 350;

/// Telemetry output buffer (single line of JSON).
constexpr uint32_t TELEMETRY_BUFFER = 1200;

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

inline constexpr const char* TROLLEY_ID = "TROLLEY_01";
inline constexpr const char* FIRMWARE_VERSION = "1.0.0";
