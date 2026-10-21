#include "network_position_provider.hpp"

#include <cstdio>
#include <cstring>
#include <new>

#include <HTTPClient.h>
#include <WiFi.h>

#if NAVIGATION_POSITION_TRANSPORT == 2
#include <PubSubClient.h>
#endif

namespace firmware {

namespace {

constexpr uint32_t kHttpPollIntervalMs = 100;   // 10 Hz pull target
constexpr uint32_t kWiFiConnectTimeoutMs = 8000;

}  // namespace

// ---------------------------------------------------------------------------
// HTTP pull provider
// ---------------------------------------------------------------------------

HttpPositionProvider::HttpPositionProvider(const char* server_url,
                                           const char* expected_trolley_id)
    : parser_(expected_trolley_id) {
    nav::copyBounded(server_url_, sizeof(server_url_), server_url);
}

bool HttpPositionProvider::ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected_ = true;
        return true;
    }
    wifi_connected_ = false;
    if (NAVIGATION_WIFI_SSID[0] == '\0') {
        return false;  // no credentials configured
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(NAVIGATION_WIFI_SSID, NAVIGATION_WIFI_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < kWiFiConnectTimeoutMs) {
        delay(50);
    }
    wifi_connected_ = (WiFi.status() == WL_CONNECTED);
    return wifi_connected_;
}

bool HttpPositionProvider::fetchOnce() {
    if (!ensureWifi()) {
        return false;
    }
    // Compose the endpoint: server_url + "/api/v1/navigation/position".
    char url[128] = {};
    std::snprintf(url, sizeof(url), "%s/api/v1/navigation/position", server_url_);

    HTTPClient http;
    http.setTimeout(500);
    if (!http.begin(url)) {
        return false;
    }
    const int code = http.GET();
    if (code != HTTP_CODE_OK) {
        http.end();
        return false;
    }
    const String body = http.getString();
    http.end();
    if (body.isEmpty()) {
        return false;
    }
    return parser_.feed(body.c_str(), body.length());
}

bool HttpPositionProvider::poll(nav::PositionMeasurement& out, uint64_t now_ms) {
    // Drain a previously fetched sample first.
    if (parser_.poll(out, now_ms)) {
        return true;
    }
    const uint32_t now = millis();
    if (now - last_attempt_ms_ < kHttpPollIntervalMs) {
        return false;
    }
    last_attempt_ms_ = now;
    if (!fetchOnce()) {
        return false;
    }
    return parser_.poll(out, now_ms);
}

// ---------------------------------------------------------------------------
// MQTT subscribe provider
// ---------------------------------------------------------------------------

#if NAVIGATION_POSITION_TRANSPORT == 2

namespace {

/// Single global instance so the PubSubClient callback (a plain function) can
/// reach the provider that owns it.
MqttPositionProvider* g_mqtt_provider = nullptr;

void mqttMessageCallback(char* topic, byte* payload, unsigned int length) {
    (void)topic;
    if (g_mqtt_provider != nullptr) {
        g_mqtt_provider->onMessage(reinterpret_cast<const char*>(payload), length);
    }
}

}  // namespace

MqttPositionProvider::MqttPositionProvider(const char* host, uint16_t port,
                                           const char* topic,
                                           const char* expected_trolley_id)
    : parser_(expected_trolley_id) {
    nav::copyBounded(host_, sizeof(host_), host);
    port_ = port;
    nav::copyBounded(topic_, sizeof(topic_), topic);
    auto* client = new (std::nothrow) PubSubClient();
    if (client != nullptr) {
        client->setCallback(mqttMessageCallback);
        client_ = client;
        g_mqtt_provider = this;
    }
}

MqttPositionProvider::~MqttPositionProvider() {
    if (g_mqtt_provider == this) {
        g_mqtt_provider = nullptr;
    }
    auto* client = static_cast<PubSubClient*>(client_);
    delete client;
    client_ = nullptr;
}

bool MqttPositionProvider::ensureWifi() {
    if (WiFi.status() == WL_CONNECTED) {
        wifi_connected_ = true;
        return true;
    }
    wifi_connected_ = false;
    if (NAVIGATION_WIFI_SSID[0] == '\0') {
        return false;
    }
    WiFi.mode(WIFI_STA);
    WiFi.begin(NAVIGATION_WIFI_SSID, NAVIGATION_WIFI_PASSWORD);
    const uint32_t started = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - started < kWiFiConnectTimeoutMs) {
        delay(50);
    }
    wifi_connected_ = (WiFi.status() == WL_CONNECTED);
    return wifi_connected_;
}

void MqttPositionProvider::ensureConnected() {
    auto* client = static_cast<PubSubClient*>(client_);
    if (client == nullptr || !ensureWifi()) {
        return;
    }
    if (client->connected()) {
        return;
    }
    client->setServer(host_, port_);
    if (client->connect(TROLLEY_ID)) {
        client->subscribe(topic_, 0);
    }
}

void MqttPositionProvider::onMessage(const char* payload, unsigned int length) {
    parser_.feed(payload, static_cast<std::size_t>(length));
}

bool MqttPositionProvider::poll(nav::PositionMeasurement& out, uint64_t now_ms) {
    auto* client = static_cast<PubSubClient*>(client_);
    if (client == nullptr) {
        return false;
    }
    ensureConnected();
    if (client->connected()) {
        client->loop();  // dispatch any queued callback(s)
    }
    return parser_.poll(out, now_ms);
}

#endif  // NAVIGATION_POSITION_TRANSPORT == 2

}  // namespace firmware
