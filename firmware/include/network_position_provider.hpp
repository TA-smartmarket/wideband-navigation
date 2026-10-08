// Network position providers for the firmware.
//
// When the real positioning subsystem (wideband-positioning) is available, the
// firmware can consume the processed position over the network instead of a UART:
//
//   * HttpPositionProvider  - polls GET /api/v1/navigation/position (pull)
//   * MqttPositionProvider  - subscribes to `<base>/navigation/position` (push)
//
// Both reuse `nav::RemotePositionProvider` (in the navigation core), so the JSON
// contract and validation are identical across serial, HTTP and MQTT.  Only the
// transport glue differs, exactly like `RealPositionProvider` for UART.
#pragma once

#include <Arduino.h>

#include "app_config.hpp"
#include "navigation/position_provider.hpp"
#include "navigation/remote_position_provider.hpp"

namespace firmware {

/// HTTP pull provider: GET the navigation position document at ~10 Hz.
class HttpPositionProvider : public nav::IPositionProvider {
public:
    explicit HttpPositionProvider(const char* server_url, const char* expected_trolley_id);

    bool poll(nav::PositionMeasurement& out, uint64_t now_ms) override;
    const char* name() const override { return "http-json"; }

    bool wifiConnected() const { return wifi_connected_; }

private:
    bool ensureWifi();
    bool fetchOnce();

    char server_url_[96]{};
    nav::RemotePositionProvider parser_;
    uint32_t last_attempt_ms_{0};
    bool wifi_connected_{false};
};

/// MQTT subscribe provider: consume the pushed position document.
class MqttPositionProvider : public nav::IPositionProvider {
public:
    MqttPositionProvider(const char* host, uint16_t port, const char* topic,
                         const char* expected_trolley_id);
    ~MqttPositionProvider() override;

    bool poll(nav::PositionMeasurement& out, uint64_t now_ms) override;
    const char* name() const override { return "mqtt-json"; }

    bool wifiConnected() const { return wifi_connected_; }

    /// PubSubClient callback entry point (dispatches to the single instance).
    void onMessage(const char* payload, unsigned int length);

private:
    bool ensureWifi();
    void ensureConnected();

    char host_[64]{};
    uint16_t port_{1883};
    char topic_[96]{};
    nav::RemotePositionProvider parser_;
    bool wifi_connected_{false};
    void* client_ = nullptr;   // PubSubClient* (opaque here to keep the header light)
};

}  // namespace firmware
