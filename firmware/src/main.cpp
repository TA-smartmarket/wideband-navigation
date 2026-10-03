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

#include "app_config.hpp"
#include "app_context.hpp"
#include "motor_driver.hpp"
#include "navigation/config_loader.hpp"
#include "navigation/logging.hpp"
#include "navigation/time_utils.hpp"
#include "pin_config.hpp"
#include "position_provider.hpp"
#include "stepper_driver.hpp"
#include "telemetry.hpp"

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

    ArduinoStreamAdapter console_stream(Serial);
    nav::SerialPositionProvider serial_provider(&console_stream, TROLLEY_ID);

#if NAVIGATION_USE_MOCK_POSITION
    // Development mode: mock UWB JSON lines arrive on the console UART.
    nav::IPositionProvider& provider = serial_provider;
    NAV_LOG_INFO("POS", "position source: %s (mock/development mode)", provider.name());
#else
    // Integration mode: the real UWB/EKF subsystem on UART1, same JSON contract.
    static HardwareSerial uwb_serial(1);
    uwb_serial.begin(kUwbUartBaud, SERIAL_8N1, kUwbUartRxPin, kUwbUartTxPin);
    static RealPositionProvider real_provider(uwb_serial, TROLLEY_ID);
    nav::IPositionProvider& provider = real_provider;
    NAV_LOG_INFO("POS", "position source: %s on UART1 rx=%d tx=%d @ %u baud",
                 provider.name(), kUwbUartRxPin, kUwbUartTxPin, kUwbUartBaud);
#endif

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        nav::PositionMeasurement measurement;
        const uint64_t now_ms = nav::monotonicMillis();
        if (provider.poll(measurement, now_ms)) {
            // The FSM performs full validation; the task only forwards the
            // sample together with the local monotonic receive time.
            ctx.latest_position.publish(measurement, now_ms);
            ++ctx.position_samples;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POSITION_PERIOD_MS));
    }
}

/// Parses console commands and raises the corresponding flags.
void taskCommands(void*) {
    FirmwareContext& ctx = context();
    char line[SERIAL_LINE_BUFFER];
    std::size_t length = 0;

    for (;;) {
        while (Serial.available() > 0) {
            const char c = static_cast<char>(Serial.read());
            if (c == '\n' || c == '\r') {
                if (length == 0) {
                    continue;
                }
                line[length] = '\0';
                nav::ParsedCommand command;
                if (nav::parseCommand(line, command)) {
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
                } else if (length > 0 && line[0] == '{') {
                    // A raw position JSON line: forward it to the position
                    // parser by re-injecting it is unnecessary (the position task
                    // reads the same UART); count it as malformed only if the
                    // position task never sees it.  Nothing to do here.
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
            motors.stop();  // cut power first, then tell the FSM
            ctx.fsm.emergencyStop();
            ctx.telemetry->publishEvent("ESTOP", "emergency stop engaged");
        }
        if (ctx.command_clear_estop) {
            ctx.command_clear_estop = false;
            ctx.fsm.clearEmergencyStop();
            motors.stop();
            ctx.telemetry->publishEvent("ESTOP", "emergency stop cleared (start required)");
        }
        if (ctx.command_cancel) {
            ctx.command_cancel = false;
            ctx.fsm.cancelNavigation();
            motors.stop();
        }
        if (ctx.command_stop) {
            ctx.command_stop = false;
            ctx.fsm.stopNavigation();
            motors.stop();
        }
        if (ctx.command_replan) {
            ctx.command_replan = false;
            ctx.fsm.requestReplan();
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
        if (status.state == nav::NavState::NAVIGATING && !ctx.fsm.emergencyStopActive()) {
            motors.setBoth(status.motor.left, status.motor.right);
        } else {
            motors.stop();
        }

        // A step/dir driver needs its pulse train independent of the control
        // rate, so the generator is serviced in short sub-slices until the next
        // navigation cycle is due.  This keeps the emitted STEP frequency close
        // to the commanded one instead of bursting every 50 ms.
        if (StepperMotorDriver* stepper = stepperDriver()) {
            const TickType_t cycle_end = last_wake + pdMS_TO_TICKS(NAVIGATION_PERIOD_MS);
            while (xTaskGetTickCount() < cycle_end) {
                stepper->servicePulseGenerator();
            }
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
    setMotorDriverConfig(config);

    // 3. Safety: stop the motors before anything else can command them.
    initialiseMotorDriver();
    motorDriver().stop();

    FirmwareContext& ctx = context();
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

    if (navigation_ok != pdPASS || position_ok != pdPASS || command_ok != pdPASS
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
