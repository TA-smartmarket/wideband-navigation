// ESP32-S3 Smart Trolley navigation firmware.
//
// Task layout (see docs/esp32_firmware.md):
//
//   TaskPosition    (core 0) - reads position JSON, validates, publishes the
//                              latest sample
//   TaskCommands    (core 0) - parses console commands and flags them for the
//                              navigation task
//   TaskTelemetry   (core 0) - formats and publishes telemetry at 5 Hz
//   TaskNavigation  (core 1) - runs the FSM / path follower at 20 Hz and drives
//                              the motors
//
// Safety invariants enforced here:
//   * motors are stopped before anything else runs and whenever the navigation
//     core reports a non-moving state;
//   * the emergency stop is latched in hardware-facing code, not only in the FSM;
//   * a stalled navigation loop trips the task watchdog instead of driving on.
#include <Arduino.h>

#include <cstring>

#include "app_config.hpp"
#include "app_context.hpp"
#include "config_store.hpp"
#include "motor_driver.hpp"
#include "navigation/config_loader.hpp"
#include "navigation/logging.hpp"
#include "navigation/time_utils.hpp"
#include "network_position_provider.hpp"
#include "pin_config.hpp"
#include "position_provider.hpp"
#include "serial_protocol.hpp"
#include "stepper_driver.hpp"
#include "telemetry.hpp"
#include "wheel_control.hpp"

#if CONFIG_FREERTOS_USE_TASK_WATCHDOG
#include <esp_task_wdt.h>
#endif

namespace {

using namespace firmware;

// --- logging ---------------------------------------------------------------

void serialLogSink(nav::LogLevel level, const char* tag, const char* message) {
    const char* name = "INFO";
    switch (level) {
        case nav::LogLevel::ERROR: name = "ERROR"; break;
        case nav::LogLevel::WARN: name = "WARN"; break;
        case nav::LogLevel::INFO: name = "INFO"; break;
        case nav::LogLevel::DEBUG: name = "DEBUG"; break;
        case nav::LogLevel::TRACE: name = "TRACE"; break;
    }
    Serial.printf("[%s][%s] %s\n", name, tag, message);
}

// --- tasks -----------------------------------------------------------------

/// Reads position lines and publishes the newest accepted sample.
void taskPosition(void*) {
    FirmwareContext& ctx = context();

#if NAVIGATION_POSITION_TRANSPORT == 1
    // HTTP pull from the positioning server (GET /api/v1/navigation/position).
    static firmware::HttpPositionProvider http_provider(NAVIGATION_SERVER_URL, TROLLEY_ID);
    nav::IPositionProvider& real_provider = http_provider;
    NAV_LOG_INFO("POS", "real position adapter: %s @ %s", real_provider.name(), NAVIGATION_SERVER_URL);
#elif NAVIGATION_POSITION_TRANSPORT == 2
    // MQTT push from the positioning server (topic <base>/navigation/position).
    static firmware::MqttPositionProvider mqtt_provider(
        NAVIGATION_MQTT_HOST, NAVIGATION_MQTT_PORT, NAVIGATION_MQTT_TOPIC, TROLLEY_ID);
    nav::IPositionProvider& real_provider = mqtt_provider;
    NAV_LOG_INFO("POS", "real position adapter: %s @ %s:%u topic '%s'",
                 real_provider.name(), NAVIGATION_MQTT_HOST, NAVIGATION_MQTT_PORT, NAVIGATION_MQTT_TOPIC);
#else
    // Integration mode: the real UWB/EKF subsystem on UART1, same JSON contract.
    static HardwareSerial uwb_serial(1);
    uwb_serial.begin(kUwbUartBaud, SERIAL_8N1, kUwbUartRxPin, kUwbUartTxPin);
    static RealPositionProvider uart_provider(uwb_serial, TROLLEY_ID);
    nav::IPositionProvider& real_provider = uart_provider;
    NAV_LOG_INFO("POS", "real position adapter: %s on UART1 rx=%d tx=%d @ %u baud",
                 real_provider.name(), kUwbUartRxPin, kUwbUartTxPin, kUwbUartBaud);
#endif

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        nav::PositionMeasurement measurement;
        const uint64_t now_ms = nav::monotonicMillis();
        bool has_measurement = false;
        if (ctx.position_source == nav::PositionSourceMode::ENCODER_ODOMETRY) {
            has_measurement = wheelControl().odometryMeasurement(measurement, now_ms);
        } else if (ctx.position_source == nav::PositionSourceMode::REAL_UWB) {
            has_measurement = real_provider.poll(measurement, now_ms);
        }
        if (has_measurement) {
            // The FSM performs full validation; the task only forwards the
            // sample together with the local monotonic receive time.
            ctx.latest_position.publish(measurement, now_ms);
            ++ctx.position_samples;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POSITION_PERIOD_MS));
    }
}

/// Runs AS5600 acquisition, wheel velocity PID, odometry and STEP-rate output.
/// Navigation only publishes bounded wheel targets; this task owns feedback
/// control at 100 Hz and stops the actuator on any encoder fault.
void taskWheelControl(void*) {
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        wheelControl().update(nav::monotonicNanos() / 1000ull);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(WHEEL_CONTROL_PERIOD_MS));
    }
}

/// Parses console commands and raises the corresponding flags.
void taskCommands(void*) {
    FirmwareContext& ctx = context();
    char line[SERIAL_LINE_BUFFER];
    std::size_t length = 0;
    nav::SerialPositionProvider mock_line_parser(nullptr, TROLLEY_ID);

    for (;;) {
        while (Serial.available() > 0) {
            const char c = static_cast<char>(Serial.read());
            if (c == '\n' || c == '\r') {
                if (length == 0) {
                    continue;
                }
                line[length] = '\0';
                nav::ParsedCommand command;
                const bool looks_like_position =
                    std::strncmp(line, "POS:", 4) == 0 ||
                    (line[0] == '{' && std::strstr(line, "\"schema_version\"") != nullptr &&
                     std::strstr(line, "\"position\"") != nullptr);
                if (ctx.position_source == nav::PositionSourceMode::EXTERNAL_MOCK_UWB &&
                    looks_like_position) {
                    nav::PositionMeasurement measurement;
                    if (mock_line_parser.parseLine(line, length, measurement)) {
                        ctx.latest_position.publish(measurement, nav::monotonicMillis());
                        ++ctx.position_samples;
                    } else {
                        ++g_malformed_position_lines;
                        Serial.println("{\"type\":\"error\",\"protocol_version\":1,\"message\":\"invalid mock position\"}");
                    }
                } else if (line[0] == '{' && std::strstr(line, "\"type\"") != nullptr) {
                    DashboardCommand dashboard_command;
                    char dashboard_error[96] = {};
                    if (parseDashboardCommand(line, dashboard_command, dashboard_error, sizeof(dashboard_error))) {
                        switch (dashboard_command.kind) {
                            case DashboardCommandKind::HELLO:
                                Serial.printf("{\"type\":\"ack\",\"protocol_version\":%u,\"config_revision\":%lu,\"message\":\"Smart Trolley firmware %s\"}\n",
                                              kDashboardProtocolVersion,
                                              static_cast<unsigned long>(configStore().revision()),
                                              FIRMWARE_VERSION);
                                break;
                            case DashboardCommandKind::MANUAL:
                                ctx.manual_linear_mps = dashboard_command.linear_mps;
                                ctx.manual_angular_radps = dashboard_command.angular_radps;
                                ctx.manual_heartbeat_ms = millis();
                                ctx.command_manual_update = true;
                                break;
                            case DashboardCommandKind::ESTOP: ctx.command_estop = true; break;
                            case DashboardCommandKind::RESET_ESTOP: ctx.command_clear_estop = true; break;
                            case DashboardCommandKind::NAVIGATION_STOP: ctx.command_stop = true; break;
                            case DashboardCommandKind::NAVIGATION_DESTINATION:
                                ctx.manual_active = false;
                                ctx.command_destination = dashboard_command.node_id;
                                break;
                            case DashboardCommandKind::CONFIG_GET:
                            {
                                char config_json[640] = {};
                                nav::NavigationConfig active = wheelControl().enabled()
                                    ? wheelControl().configSnapshot() : motorDriverConfig();
                                active.position_source = ctx.position_source;
                                configStore().formatJson(active, config_json, sizeof(config_json));
                                Serial.print(config_json);
                                break;
                            }
                            case DashboardCommandKind::CONFIG_SAVE:
                            {
                                nav::NavigationConfig active = wheelControl().enabled()
                                    ? wheelControl().configSnapshot() : motorDriverConfig();
                                active.position_source = ctx.position_source;
                                const bool ok = configStore().save(active);
                                Serial.printf("{\"type\":\"%s\",\"protocol_version\":1,\"config_revision\":%lu,\"message\":\"%s\"}\n",
                                              ok ? "ack" : "error",
                                              static_cast<unsigned long>(configStore().revision()),
                                              ok ? "configuration persisted" : "configuration save failed");
                                break;
                            }
                            case DashboardCommandKind::CONFIG_DEFAULTS:
                            {
                                ctx.manual_active = false;
                                ctx.command_stop = true;
                                const bool ok = configStore().restoreDefaults();
                                Serial.printf("{\"type\":\"%s\",\"protocol_version\":1,\"config_revision\":0,\"message\":\"%s\"}\n",
                                              ok ? "ack" : "error",
                                              ok ? "factory defaults selected; reboot required" : "failed to restore defaults");
                                break;
                            }
                            case DashboardCommandKind::PID_SET:
                            {
                                bool ok = true;
                                if (dashboard_command.heading_pid) {
                                    ok = false;
                                    if (xSemaphoreTake(ctx.status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                                        ok = ctx.fsm.setHeadingPidGains(
                                            dashboard_command.kp, dashboard_command.ki,
                                            dashboard_command.kd);
                                        xSemaphoreGive(ctx.status_mutex);
                                    }
                                    ok = ok && wheelControl().recordHeadingPid(
                                        dashboard_command.kp, dashboard_command.ki,
                                        dashboard_command.kd);
                                } else {
                                    ok = wheelControl().requestPidUpdate(
                                        dashboard_command.right_wheel, dashboard_command.kp,
                                        dashboard_command.ki, dashboard_command.kd);
                                }
                                Serial.printf("{\"type\":\"%s\",\"protocol_version\":1,\"message\":\"%s PID %s\"}\n",
                                              ok ? "ack" : "error",
                                              dashboard_command.heading_pid ? "heading" :
                                              dashboard_command.right_wheel ? "right wheel" : "left wheel",
                                              ok ? "applied (not yet persisted)" : "rejected");
                                break;
                            }
                            case DashboardCommandKind::POSITION_SOURCE:
                                ctx.command_position_source = dashboard_command.position_source;
                                Serial.println("{\"type\":\"ack\",\"protocol_version\":1,\"message\":\"position source change queued; navigation will stop\"}");
                                break;
                            case DashboardCommandKind::ENCODER_ZERO:
                                ctx.command_encoder_zero = true;
                                Serial.println("{\"type\":\"ack\",\"protocol_version\":1,\"message\":\"encoder zero queued; navigation will stop\"}");
                                break;
                            case DashboardCommandKind::ODOMETRY_SET_POSE:
                                ctx.command_pose_x_m = dashboard_command.pose_x_m;
                                ctx.command_pose_y_m = dashboard_command.pose_y_m;
                                ctx.command_pose_heading_rad = dashboard_command.pose_heading_rad;
                                ctx.command_set_pose = true;
                                Serial.println("{\"type\":\"ack\",\"protocol_version\":1,\"message\":\"initial pose queued; navigation will stop\"}");
                                break;
                            default: break;
                        }
                    } else {
                        Serial.printf("{\"type\":\"error\",\"protocol_version\":1,\"message\":\"%s\"}\n", dashboard_error);
                    }
                } else if (nav::parseCommand(line, command)) {
                    switch (command.kind) {
                        case nav::CommandKind::DESTINATION:
                            if (command.argument > 0) {
                                ctx.command_destination = command.argument;
                                Serial.printf("CMD: destination queued -> node %d\n",
                                              command.argument);
                            } else {
                                Serial.println("CMD: usage: destination <node_id>");
                            }
                            break;
                        case nav::CommandKind::START:
                            ctx.command_destination = -2;  // sentinel: "start"
                            break;
                        case nav::CommandKind::STOP:
                            ctx.command_stop = true;
                            break;
                        case nav::CommandKind::CANCEL:
                            ctx.command_cancel = true;
                            break;
                        case nav::CommandKind::ESTOP:
                            ctx.command_estop = true;
                            break;
                        case nav::CommandKind::CLEAR_ESTOP:
                            ctx.command_clear_estop = true;
                            break;
                        case nav::CommandKind::REPLAN:
                            ctx.command_replan = true;
                            break;
                        case nav::CommandKind::DEBUG_ON:
                            nav::setLogLevel(nav::LogLevel::DEBUG);
                            Serial.println("CMD: debug logging on");
                            break;
                        case nav::CommandKind::DEBUG_OFF:
                            nav::setLogLevel(nav::LogLevel::INFO);
                            Serial.println("CMD: debug logging off");
                            break;
                        case nav::CommandKind::HELP:
                            Serial.print(nav::commandHelpText());
                            break;
                        default:
                            // status/graph/route/position are answered directly
                            // by the command task because they only read state.
                            break;
                    }
                    if (command.kind == nav::CommandKind::UNKNOWN) {
                        Serial.printf("CMD: unknown command '%s' (try 'help')\n", command.raw);
                    }
                }
                length = 0;
                continue;
            }
            if (length < SERIAL_LINE_BUFFER - 1) {
                line[length++] = c;
            } else {
                // Overflow: drop the line rather than corrupting the parser.
                length = 0;
                ++ctx.position_rejected;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/// Publishes telemetry at the configured rate.
void taskTelemetry(void*) {
    FirmwareContext& ctx = context();
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (ctx.telemetry != nullptr && xSemaphoreTake(ctx.status_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            const nav::NavigationStatus status = ctx.fsm.status();
            xSemaphoreGive(ctx.status_mutex);
            ctx.telemetry->publishJson(status, nav::monotonicMillis());
            if (wheelControl().enabled()) {
                ctx.telemetry->publishWheelJson(wheelControl().snapshot(),
                                                nav::monotonicMillis());
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}

/// Runs the navigation control loop at 20 Hz and actuates the motors.
void taskNavigation(void*) {
    FirmwareContext& ctx = context();
    IMotorDriver& motors = motorDriver();
    nav::LogLevel last_logged_state = nav::LogLevel::INFO;
    (void)last_logged_state;

    // Motors must be stopped before the first control cycle, whatever the FSM
    // believes its state to be.
    motors.stop();

#if CONFIG_FREERTOS_USE_TASK_WATCHDOG
    // Subscribe only this task: the watchdog exists to catch a stalled *control
    // loop*, not to police the I/O tasks.
    esp_task_wdt_add(nullptr);
#endif

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_loop_us = static_cast<uint32_t>(nav::monotonicNanos() / 1000ull);

    for (;;) {
        const uint64_t now_ms = nav::monotonicMillis();

        // --- apply queued commands ---------------------------------------
        if (ctx.command_estop) {
            ctx.command_estop = false;
            // Cut STEP generation/driver enable before the FSM state change.
            // A normal stop is ramped; E-stop must be immediate.
            if (StepperMotorDriver* stepper = stepperDriver()) {
                if (wheelControl().enabled()) wheelControl().emergencyStop();
                else stepper->emergencyStop();
            } else {
                motors.stop();
            }
            ctx.fsm.emergencyStop();
            ctx.manual_active = false;
            ctx.telemetry->publishEvent("ESTOP", "emergency stop engaged");
        }
        if (ctx.command_clear_estop) {
            ctx.command_clear_estop = false;
            ctx.fsm.clearEmergencyStop();
            if (wheelControl().enabled()) wheelControl().clearEmergencyStop(); else motors.stop();
            ctx.telemetry->publishEvent("ESTOP", "emergency stop cleared (start required)");
        }
        if (ctx.command_cancel) {
            ctx.command_cancel = false;
            ctx.fsm.cancelNavigation();
            ctx.manual_active = false;
            if (wheelControl().enabled()) wheelControl().stop(); else motors.stop();
        }
        if (ctx.command_stop) {
            ctx.command_stop = false;
            ctx.fsm.stopNavigation();
            ctx.manual_active = false;
            if (wheelControl().enabled()) wheelControl().stop(); else motors.stop();
        }
        if (ctx.command_replan) {
            ctx.command_replan = false;
            ctx.fsm.requestReplan();
        }
        if (ctx.command_manual_update) {
            ctx.command_manual_update = false;
            // Transfer actuator ownership only after autonomous motion stops.
            // A heartbeat timeout can therefore never resume the old route.
            ctx.fsm.stopNavigation();
            ctx.manual_active = true;
        }
        if (ctx.command_position_source >= 0) {
            const int requested = ctx.command_position_source;
            ctx.command_position_source = -1;
            ctx.manual_active = false;
            ctx.fsm.stopNavigation();
            wheelControl().stop();
            ctx.latest_position.clear();
            ctx.fsm.notifyPositionStreamLost();
            ctx.position_source = static_cast<nav::PositionSourceMode>(requested);
            if (wheelControl().enabled()) wheelControl().setPositionSource(ctx.position_source);
            ctx.telemetry->publishEvent("POSITION_SOURCE",
                requested == 0 ? "encoder odometry selected; explicit start required" :
                requested == 1 ? "external mock UWB selected; explicit start required" :
                                 "real UWB selected; explicit start required");
        }
        if (ctx.command_encoder_zero) {
            ctx.command_encoder_zero = false;
            ctx.manual_active = false;
            ctx.fsm.stopNavigation();
            wheelControl().stop();
            const nav::OdometryPose pose = wheelControl().snapshot().odometry.pose;
            const bool ok = wheelControl().requestOdometryReset(pose, true);
            ctx.telemetry->publishEvent("ENCODER_ZERO",
                ok ? "encoders zeroed; explicit start required" : "encoder zero rejected");
        }
        if (ctx.command_set_pose) {
            ctx.command_set_pose = false;
            ctx.manual_active = false;
            ctx.fsm.stopNavigation();
            wheelControl().stop();
            nav::OdometryPose pose;
            pose.x_m = ctx.command_pose_x_m;
            pose.y_m = ctx.command_pose_y_m;
            pose.heading_rad = ctx.command_pose_heading_rad;
            const bool ok = wheelControl().requestOdometryReset(pose, false);
            ctx.latest_position.clear();
            ctx.telemetry->publishEvent("ODOMETRY_POSE",
                ok ? "initial pose applied; explicit start required" : "initial pose rejected");
        }
        if (ctx.command_destination != -1) {
            const int requested = ctx.command_destination;
            ctx.command_destination = -1;
            if (requested == -2) {
                // "start" command: resume the destination held across an E-stop.
                const bool ok = ctx.fsm.startNavigation();
                ctx.telemetry->publishEvent("NAV", ok ? "start accepted" : "start rejected");
            } else {
                const bool ok = ctx.fsm.requestDestination(requested);
                ctx.telemetry->publishEvent("NAV", ok ? "destination accepted" : "destination rejected");
            }
        }

        // --- feed the newest position ------------------------------------
        nav::PositionMeasurement measurement;
        uint64_t received_at_ms = 0;
        uint32_t sequence = 0;
        if (ctx.latest_position.read(measurement, received_at_ms, sequence)) {
            const nav::MeasurementStatus status =
                ctx.fsm.submitPosition(measurement, received_at_ms);
            if (status != nav::MeasurementStatus::OK) {
                ++ctx.position_rejected;
            }
        }

        // --- one control cycle -------------------------------------------
        if (xSemaphoreTake(ctx.status_mutex, 0) == pdTRUE) {
            ctx.fsm.update(now_ms);
            xSemaphoreGive(ctx.status_mutex);
        }
        const nav::NavigationStatus& status = ctx.fsm.status();

        // Safety gate: the FSM is the single authority on whether motion is
        // allowed, but the motor layer re-checks the emergency stop so a latched
        // E-stop can never be overridden by a stale status field.
        const bool manual_timed_out = ctx.manual_active &&
            static_cast<uint32_t>(now_ms - ctx.manual_heartbeat_ms) > MANUAL_COMMAND_TIMEOUT_MS;
        if (manual_timed_out) {
            ctx.manual_active = false;
            if (wheelControl().enabled()) wheelControl().stop(); else motors.stop();
            ctx.telemetry->publishEvent("WATCHDOG", "manual command heartbeat timed out");
        }
        if (ctx.manual_active && !ctx.fsm.emergencyStopActive()) {
            const nav::Twist manual_twist{ctx.manual_linear_mps, ctx.manual_angular_radps};
            const nav::MotorCommand manual_motor =
                nav::twistToMotorCommand(manual_twist, motorDriverConfig().drive);
            if (wheelControl().enabled()) {
                wheelControl().setNormalizedTarget(manual_motor.left, manual_motor.right);
            } else {
                motors.setBoth(manual_motor.left, manual_motor.right);
            }
        } else if (status.state == nav::NavState::NAVIGATING && !ctx.fsm.emergencyStopActive()) {
            if (wheelControl().enabled()) {
                wheelControl().setNormalizedTarget(status.motor.left, status.motor.right);
            } else {
                motors.setBoth(status.motor.left, status.motor.right);
            }
        } else {
            if (wheelControl().enabled()) wheelControl().stop(); else motors.stop();
        }

        // --- loop timing diagnostics -------------------------------------
        const uint32_t now_us = static_cast<uint32_t>(nav::monotonicNanos() / 1000ull);
        const uint32_t elapsed_us = now_us - last_loop_us;
        last_loop_us = now_us;
        if (elapsed_us > NAVIGATION_PERIOD_MS * 1000u * 2u) {
            ++ctx.loop_overruns;
            NAV_LOG_WARN("NAV", "control loop overrun: %lu us (target %lu us)",
                         static_cast<unsigned long>(elapsed_us),
                         static_cast<unsigned long>(NAVIGATION_PERIOD_MS * 1000u));
        }

#if CONFIG_FREERTOS_USE_TASK_WATCHDOG
        esp_task_wdt_reset();
#endif
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(NAVIGATION_PERIOD_MS));
    }
}

}  // namespace

void setup() {
    // 1. Serial first: the very first thing a developer needs is the log.
    Serial.begin(115200);
    const uint32_t serial_start = millis();
    while (!Serial && (millis() - serial_start) < 1500) {
        delay(10);  // bounded wait: never block start-up indefinitely
    }
    delay(50);

    nav::setLogSink(serialLogSink);
#if NAV_LOG_LEVEL >= 3
    nav::setLogLevel(nav::LogLevel::DEBUG);
#else
    nav::setLogLevel(nav::LogLevel::INFO);
#endif

    Serial.println();
    NAV_LOG_INFO("BOOT", "Smart Trolley navigation firmware v%s (ESP32-S3)",
                 FIRMWARE_VERSION);

    // 2. Load the configuration BEFORE building the actuator, because
    //    `drive_kind` selects which driver to instantiate.
    static nav::Graph graph;
    char error[192] = {};
    if (!nav::loadGraph(builtinGraphJson(), graph, error, sizeof(error))) {
        NAV_LOG_ERROR("BOOT", "graph load failed: %s", error);
        for (;;) {
            delay(1000);
        }
    }
    nav::NavigationConfig config;
    if (!nav::loadNavigationConfig(builtinNavigationJson(), config, error, sizeof(error))) {
        NAV_LOG_ERROR("BOOT", "navigation config load failed: %s", error);
        for (;;) {
            delay(1000);
        }
    }
    const bool loaded_persisted_config = configStore().begin(config);
    if (!nav::validateConfig(config).valid) {
        NAV_LOG_ERROR("BOOT", "persisted configuration invalid; halting safely");
        for (;;) delay(1000);
    }
    NAV_LOG_INFO("CFG", "%s configuration revision %lu",
                 loaded_persisted_config ? "loaded persisted" : "using repository default",
                 static_cast<unsigned long>(configStore().revision()));
#if NAVIGATION_USE_MOCK_POSITION
    config.position_source = nav::PositionSourceMode::EXTERNAL_MOCK_UWB;
#endif
    setMotorDriverConfig(config);

    // 3. Safety: stop the motors before anything else can command them.
    initialiseMotorDriver();
    motorDriver().stop();
    if (config.drive_kind == nav::DriveKind::STEPPER_STEP_DIR &&
        config.wheel_closed_loop_enabled) {
        if (!wheelControl().begin(config, stepperDriver())) {
            NAV_LOG_ERROR("WHEEL", "closed-loop wheel control initialization failed; motors remain stopped");
            motorDriver().stop();
            for (;;) {
                delay(1000);
            }
        } else {
            NAV_LOG_INFO("WHEEL", "dual AS5600 + wheel PID initialized at %.0f Hz",
                         static_cast<double>(config.control.wheel_rate_hz));
        }
    }

    FirmwareContext& ctx = context();
    ctx.position_source = config.position_source;
    ctx.status_mutex = xSemaphoreCreateMutex();
    if (ctx.status_mutex == nullptr) {
        NAV_LOG_ERROR("BOOT", "cannot create status mutex; halting with motors stopped");
        motorDriver().stop();
        for (;;) {
            delay(1000);
        }
    }

    static SerialTelemetrySink serial_sink(Serial);
    static TelemetryPublisher telemetry(&serial_sink);
    ctx.telemetry = &telemetry;

    // 4. Initialise the navigation core with the same graph and configuration.
    const nav::ConfigValidation validation = ctx.fsm.begin(&graph, config);
    if (!validation.valid) {
        NAV_LOG_ERROR("BOOT", "navigation init failed: %s", validation.message);
        motorDriver().stop();
        for (;;) {
            delay(1000);
        }
    }
    NAV_LOG_INFO("BOOT", "graph: %d nodes, %d edges", graph.nodeCount(), graph.edgeCount());
    NAV_LOG_INFO("BOOT", "control loop %lu Hz, telemetry %lu Hz",
                 static_cast<unsigned long>(1000u / NAVIGATION_PERIOD_MS),
                 static_cast<unsigned long>(1000u / TELEMETRY_PERIOD_MS));

    // 4. Start the tasks.
    const BaseType_t navigation_ok = xTaskCreatePinnedToCore(
        taskNavigation, "TaskNavigation", NAVIGATION_TASK_STACK, nullptr,
        NAVIGATION_TASK_PRIORITY, nullptr, NAVIGATION_TASK_CORE);
    BaseType_t wheel_control_ok = pdPASS;
    if (wheelControl().enabled()) {
        wheel_control_ok = xTaskCreatePinnedToCore(
            taskWheelControl, "TaskWheelControl", WHEEL_CONTROL_TASK_STACK, nullptr,
            WHEEL_CONTROL_TASK_PRIORITY, nullptr, NAVIGATION_TASK_CORE);
    }
    const BaseType_t position_ok = xTaskCreatePinnedToCore(
        taskPosition, "TaskPosition", COMMUNICATION_TASK_STACK, nullptr,
        POSITION_TASK_PRIORITY, nullptr, COMMUNICATION_TASK_CORE);
    const BaseType_t command_ok = xTaskCreatePinnedToCore(
        taskCommands, "TaskCommands", COMMUNICATION_TASK_STACK, nullptr,
        COMMAND_TASK_PRIORITY, nullptr, COMMUNICATION_TASK_CORE);
#if NAVIGATION_ENABLE_TELEMETRY
    const BaseType_t telemetry_ok = xTaskCreatePinnedToCore(
        taskTelemetry, "TaskTelemetry", TELEMETRY_TASK_STACK, nullptr,
        TELEMETRY_TASK_PRIORITY, nullptr, COMMUNICATION_TASK_CORE);
#endif

    if (navigation_ok != pdPASS || wheel_control_ok != pdPASS ||
        position_ok != pdPASS || command_ok != pdPASS
#if NAVIGATION_ENABLE_TELEMETRY
        || telemetry_ok != pdPASS
#endif
    ) {
        NAV_LOG_ERROR("BOOT", "task creation failed; motors remain stopped");
        motorDriver().stop();
        for (;;) {
            delay(1000);
        }
    }

    // 5. Watchdog: only enabled once the control loop exists, so the WDT can
    //    actually catch a stall in it.
#if CONFIG_FREERTOS_USE_TASK_WATCHDOG
    esp_task_wdt_init(NAVIGATION_WATCHDOG_TIMEOUT_S, true);
#endif

    ctx.navigation_ready = true;
    NAV_LOG_INFO("BOOT", "ready: motors stopped, waiting for a destination and a valid position");
    Serial.println("Send: CMD:destination 11   then   CMD:start");
    Serial.println("Or:   {\"schema_version\":1,\"trolley_id\":\"TROLLEY_01\","
                   "\"frame_id\":\"smart_market_map\",\"timestamp_ms\":1000,"
                   "\"position\":{\"x_m\":0.8,\"y_m\":0.9},\"quality\":0.95,\"valid\":true}");
}

void loop() {
    // All work happens in the pinned FreeRTOS tasks.  The Arduino loop task only
    // keeps the scheduler's default task alive and reports periodic health.
    static uint32_t last_report_ms = 0;
    const uint32_t now = millis();
    if (now - last_report_ms >= 10000) {
        last_report_ms = now;
        FirmwareContext& ctx = context();
        NAV_LOG_DEBUG("SYS", "samples=%lu rejected=%lu overruns=%lu heap=%lu",
                      static_cast<unsigned long>(ctx.position_samples),
                      static_cast<unsigned long>(ctx.position_rejected),
                      static_cast<unsigned long>(ctx.loop_overruns),
                      static_cast<unsigned long>(ESP.getFreeHeap()));
    }
    delay(50);
}
