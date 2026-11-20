// Read-only MQTT mirror for firmware telemetry.  Network work happens only
// when service() is called by TaskTelemetry; write() only stores a bounded
// latest line, so navigation and wheel-control tasks never wait on Wi-Fi.
#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "telemetry.hpp"

namespace firmware {

class MqttTelemetrySink final : public ITelemetrySink {
public:
    MqttTelemetrySink();
    void write(const char* text) override;
    const char* name() const override { return "mqtt"; }
    /// Begin association early in setup(). This is non-blocking, so it cannot
    /// delay safety-critical control or hold the motors enabled.
    void begin();
    void service();

    /// Copy the newest accepted MQTT command. Parsing and every
    /// motor-affecting action remain in TaskCommands.
    bool takeCommand(char* destination, std::size_t destination_size);
    /// Queue a bounded JSON response produced by TaskCommands.
    void queueResponse(const char* json_line);

private:
    enum class PendingKind : uint8_t { NONE = 0, TELEMETRY = 1, WHEEL = 2, VERIFICATION = 3, EVENT = 4, RESPONSE = 5 };
    enum class ConnectionState : uint8_t { DISCONNECTED, WAITING_CONNACK, CONNECTED };

    bool ensureConnected();
    void startWifi(bool reconnect);
    bool publish(PendingKind kind, const char* payload);
    bool publishStatus(const char* payload, bool retained);
    bool sendConnect();
    bool sendSubscribe();
    void readPackets();
    bool processPacket(uint8_t header, const uint8_t* payload, std::size_t length);
    void disconnect();
    static PendingKind classify(const char* line);
    char* pendingBuffer(PendingKind kind);
    void queueCommand(const char* topic, const uint8_t* payload, unsigned int length);

    WiFiClient transport_{};
    portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
    char assembling_[TELEMETRY_BUFFER]{};
    char pending_telemetry_[TELEMETRY_BUFFER]{};
    char pending_wheel_[TELEMETRY_BUFFER]{};
    char pending_verification_[TELEMETRY_BUFFER]{};
    char pending_event_[TELEMETRY_BUFFER]{};
    char pending_response_[TELEMETRY_BUFFER]{};
    char pending_command_[SERIAL_LINE_BUFFER]{};
    std::size_t assembling_length_{0};
    uint8_t pending_mask_{0};
    bool command_pending_{false};
    wl_status_t last_wifi_status_{WL_NO_SHIELD};
    uint32_t last_wifi_attempt_ms_{0};
    uint32_t last_mqtt_attempt_ms_{0};
    uint32_t last_io_ms_{0};
    ConnectionState connection_state_{ConnectionState::DISCONNECTED};
    uint8_t receive_buffer_[SERIAL_LINE_BUFFER + 128]{};
    std::size_t receive_length_{0};
    uint16_t next_packet_id_{1};
    // Edge-triggered state only: serial output must report a connection
    // change, never continuously print network status.
    bool wifi_was_connected_{false};
    char client_id_[48]{};
    char telemetry_topic_[96]{};
    char wheel_topic_[96]{};
    char verification_topic_[96]{};
    char event_topic_[96]{};
    char availability_topic_[96]{};
    char command_topic_[96]{};
    char response_topic_[96]{};
};

MqttTelemetrySink& mqttTelemetry();

}  // namespace firmware
