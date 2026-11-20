#include "mqtt_telemetry.hpp"

#include <cstdio>
#include <cstring>

#include "app_config.hpp"
#include "navigation/logging.hpp"

namespace firmware {
namespace {
constexpr uint32_t kReconnectIntervalMs = 5000;
constexpr uint32_t kKeepAliveIntervalMs = 10000;
constexpr int32_t kTcpConnectTimeoutMs = 100;
constexpr uint16_t kMqttKeepAliveSeconds = 15;

bool appendByte(uint8_t* b, std::size_t cap, std::size_t& n, uint8_t v) {
    if (n >= cap) return false;
    b[n++] = v;
    return true;
}
bool appendString(uint8_t* b, std::size_t cap, std::size_t& n, const char* text) {
    if (text == nullptr) return false;
    const std::size_t size = std::strlen(text);
    if (size > 0xffffu || n + 2u + size > cap) return false;
    b[n++] = static_cast<uint8_t>(size >> 8u);
    b[n++] = static_cast<uint8_t>(size & 0xffu);
    std::memcpy(b + n, text, size);
    n += size;
    return true;
}
bool encodePacket(WiFiClient& transport, uint8_t header, const uint8_t* data, std::size_t size) {
    uint8_t prefix[5]{};
    std::size_t prefix_size = 1;
    prefix[0] = header;
    std::size_t remaining = size;
    do {
        uint8_t byte = static_cast<uint8_t>(remaining % 128u);
        remaining /= 128u;
        if (remaining > 0) byte |= 0x80u;
        prefix[prefix_size++] = byte;
    } while (remaining > 0 && prefix_size < sizeof(prefix));
    return transport.write(prefix, prefix_size) == prefix_size &&
           (size == 0 || transport.write(data, size) == size);
}
bool decodeRemainingLength(const uint8_t* b, std::size_t available,
                           std::size_t& value, std::size_t& encoded) {
    value = 0;
    encoded = 0;
    std::size_t multiplier = 1;
    while (encoded < available && encoded < 4u) {
        const uint8_t byte = b[encoded++];
        value += static_cast<std::size_t>(byte & 0x7fu) * multiplier;
        if ((byte & 0x80u) == 0) return true;
        multiplier *= 128u;
    }
    return false;
}
}  // namespace

MqttTelemetrySink::MqttTelemetrySink() {
    std::snprintf(client_id_, sizeof(client_id_), "%s-telemetry", TROLLEY_ID);
    std::snprintf(telemetry_topic_, sizeof(telemetry_topic_), "%s/%s/telemetry", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
    std::snprintf(wheel_topic_, sizeof(wheel_topic_), "%s/%s/wheel_telemetry", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
    std::snprintf(verification_topic_, sizeof(verification_topic_), "%s/%s/position_verification", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
    std::snprintf(event_topic_, sizeof(event_topic_), "%s/%s/event", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
    std::snprintf(availability_topic_, sizeof(availability_topic_), "%s/%s/status", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
    std::snprintf(command_topic_, sizeof(command_topic_), "%s/%s/command", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
    std::snprintf(response_topic_, sizeof(response_topic_), "%s/%s/response", NAVIGATION_MQTT_TELEMETRY_BASE_TOPIC, TROLLEY_ID);
}

MqttTelemetrySink::PendingKind MqttTelemetrySink::classify(const char* line) {
    if (std::strstr(line, "\"type\":\"wheel_telemetry\"") != nullptr) return PendingKind::WHEEL;
    if (std::strstr(line, "\"type\":\"position_verification\"") != nullptr) return PendingKind::VERIFICATION;
    if (std::strstr(line, "\"type\":\"telemetry\"") != nullptr) return PendingKind::TELEMETRY;
    if (std::strncmp(line, "[EVENT]", 7) == 0) return PendingKind::EVENT;
    return PendingKind::NONE;
}

char* MqttTelemetrySink::pendingBuffer(PendingKind kind) {
    switch (kind) {
        case PendingKind::TELEMETRY: return pending_telemetry_;
        case PendingKind::WHEEL: return pending_wheel_;
        case PendingKind::VERIFICATION: return pending_verification_;
        case PendingKind::EVENT: return pending_event_;
        case PendingKind::RESPONSE: return pending_response_;
        default: return nullptr;
    }
}

void MqttTelemetrySink::startWifi(bool reconnect) {
    if (NAVIGATION_WIFI_SSID[0] == '\0') {
        NAV_LOG_ERROR("MQTT", "Wi-Fi SSID is empty; telemetry is disabled");
        return;
    }
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(NAVIGATION_WIFI_SSID, NAVIGATION_WIFI_PASSWORD);
    last_wifi_attempt_ms_ = millis();
    (void)reconnect;
}
void MqttTelemetrySink::begin() { startWifi(false); }

void MqttTelemetrySink::write(const char* text) {
    if (text == nullptr) return;
    portENTER_CRITICAL(&mux_);
    for (const char* p = text; *p != '\0'; ++p) {
        if (*p == '\n' || *p == '\r') {
            if (assembling_length_ > 0) {
                assembling_[assembling_length_] = '\0';
                const PendingKind kind = classify(assembling_);
                if (kind != PendingKind::NONE) {
                    char* pending = pendingBuffer(kind);
                    if (pending != nullptr) {
                        std::strncpy(pending, assembling_, TELEMETRY_BUFFER - 1);
                        pending[TELEMETRY_BUFFER - 1] = '\0';
                        pending_mask_ |= static_cast<uint8_t>(1u << static_cast<uint8_t>(kind));
                    }
                }
                assembling_length_ = 0;
            }
        } else if (assembling_length_ + 1 < sizeof(assembling_)) {
            assembling_[assembling_length_++] = *p;
        } else {
            assembling_length_ = 0;
        }
    }
    portEXIT_CRITICAL(&mux_);
}

void MqttTelemetrySink::disconnect() {
    transport_.stop();
    connection_state_ = ConnectionState::DISCONNECTED;
    receive_length_ = 0;
}

bool MqttTelemetrySink::sendConnect() {
    uint8_t packet[384]{};
    std::size_t n = 0;
    const bool credentials = NAVIGATION_MQTT_USERNAME[0] != '\0';
    const uint8_t flags = static_cast<uint8_t>(0x02u | 0x04u | 0x08u | 0x20u |
        (credentials ? 0x80u : 0u) | (credentials && NAVIGATION_MQTT_PASSWORD[0] != '\0' ? 0x40u : 0u));
    if (!appendByte(packet, sizeof(packet), n, 0) || !appendByte(packet, sizeof(packet), n, 4) ||
        !appendByte(packet, sizeof(packet), n, 'M') || !appendByte(packet, sizeof(packet), n, 'Q') ||
        !appendByte(packet, sizeof(packet), n, 'T') || !appendByte(packet, sizeof(packet), n, 'T') ||
        !appendByte(packet, sizeof(packet), n, 4) || !appendByte(packet, sizeof(packet), n, flags) ||
        !appendByte(packet, sizeof(packet), n, 0) || !appendByte(packet, sizeof(packet), n, kMqttKeepAliveSeconds) ||
        !appendString(packet, sizeof(packet), n, client_id_) ||
        !appendString(packet, sizeof(packet), n, availability_topic_) ||
        !appendString(packet, sizeof(packet), n, "offline") ||
        (credentials && !appendString(packet, sizeof(packet), n, NAVIGATION_MQTT_USERNAME)) ||
        (credentials && NAVIGATION_MQTT_PASSWORD[0] != '\0' && !appendString(packet, sizeof(packet), n, NAVIGATION_MQTT_PASSWORD))) return false;
    if (!encodePacket(transport_, 0x10, packet, n)) return false;
    last_io_ms_ = millis();
    connection_state_ = ConnectionState::WAITING_CONNACK;
    return true;
}

bool MqttTelemetrySink::sendSubscribe() {
    uint8_t packet[128]{};
    std::size_t n = 0;
    const uint16_t id = next_packet_id_++;
    if (!appendByte(packet, sizeof(packet), n, static_cast<uint8_t>(id >> 8u)) ||
        !appendByte(packet, sizeof(packet), n, static_cast<uint8_t>(id)) ||
        !appendString(packet, sizeof(packet), n, command_topic_) || !appendByte(packet, sizeof(packet), n, 0)) return false;
    if (!encodePacket(transport_, 0x82, packet, n)) return false;
    last_io_ms_ = millis();
    return true;
}

bool MqttTelemetrySink::ensureConnected() {
    const wl_status_t wifi_status = WiFi.status();
    if (wifi_status != last_wifi_status_) {
        last_wifi_status_ = wifi_status;
        NAV_LOG_INFO("MQTT", "Wi-Fi association state %d", static_cast<int>(wifi_status));
    }
    if (wifi_status != WL_CONNECTED) {
        if (wifi_was_connected_) {
            wifi_was_connected_ = false;
            disconnect();
            NAV_LOG_WARN("MQTT", "Wi-Fi disconnected from SSID '%s'", NAVIGATION_WIFI_SSID);
        }
        if (millis() - last_wifi_attempt_ms_ >= kReconnectIntervalMs) startWifi(true);
        return false;
    }
    if (!wifi_was_connected_) {
        wifi_was_connected_ = true;
        NAV_LOG_INFO("MQTT", "Wi-Fi connected: SSID '%s', IP %s", NAVIGATION_WIFI_SSID, WiFi.localIP().toString().c_str());
    }
    if (connection_state_ == ConnectionState::CONNECTED) {
        if (transport_.connected()) return true;
        disconnect();
    }
    if (connection_state_ == ConnectionState::WAITING_CONNACK) {
        if (millis() - last_io_ms_ >= 1000u) disconnect();
        return false;
    }
    if (millis() - last_mqtt_attempt_ms_ < kReconnectIntervalMs) return false;
    last_mqtt_attempt_ms_ = millis();
    IPAddress ip;
    if (!ip.fromString(NAVIGATION_MQTT_HOST)) {
        NAV_LOG_ERROR("MQTT", "broker host must be a numeric LAN IP: %s", NAVIGATION_MQTT_HOST);
        return false;
    }
    if (!transport_.connect(ip, NAVIGATION_MQTT_PORT, kTcpConnectTimeoutMs) || !sendConnect()) {
        disconnect();
        return false;
    }
    return false;
}

bool MqttTelemetrySink::processPacket(uint8_t header, const uint8_t* data, std::size_t size) {
    const uint8_t type = static_cast<uint8_t>(header >> 4u);
    if (type == 2u) {
        if (connection_state_ != ConnectionState::WAITING_CONNACK || size != 2u || data[1] != 0u) return false;
        connection_state_ = ConnectionState::CONNECTED;
        if (!sendSubscribe() || !publishStatus("online", true)) return false;
        NAV_LOG_INFO("MQTT", "broker connected at %s:%u", NAVIGATION_MQTT_HOST, static_cast<unsigned>(NAVIGATION_MQTT_PORT));
        return true;
    }
    if (type != 3u) return true;
    if (size < 2u) return false;
    const std::size_t topic_size = (static_cast<std::size_t>(data[0]) << 8u) | data[1];
    if (topic_size == 0u || topic_size + 2u > size || topic_size >= 128u) return false;
    char topic[128]{};
    std::memcpy(topic, data + 2u, topic_size);
    std::size_t offset = topic_size + 2u;
    const uint8_t qos = static_cast<uint8_t>((header >> 1u) & 0x03u);
    if (qos > 1u || (qos == 1u && offset + 2u > size)) return false;
    if (qos == 1u) offset += 2u;
    queueCommand(topic, data + offset, static_cast<unsigned int>(size - offset));
    return true;
}

void MqttTelemetrySink::readPackets() {
    while (transport_.available() > 0 && receive_length_ < sizeof(receive_buffer_)) {
        const int value = transport_.read();
        if (value < 0) break;
        receive_buffer_[receive_length_++] = static_cast<uint8_t>(value);
        last_io_ms_ = millis();
    }
    if (receive_length_ == sizeof(receive_buffer_) && transport_.available() > 0) {
        disconnect();
        return;
    }
    while (receive_length_ >= 2u) {
        std::size_t remaining = 0, encoded = 0;
        if (!decodeRemainingLength(receive_buffer_ + 1u, receive_length_ - 1u, remaining, encoded)) return;
        const std::size_t total = 1u + encoded + remaining;
        if (total > sizeof(receive_buffer_)) { disconnect(); return; }
        if (receive_length_ < total) return;
        if (!processPacket(receive_buffer_[0], receive_buffer_ + 1u + encoded, remaining)) { disconnect(); return; }
        const std::size_t tail = receive_length_ - total;
        if (tail > 0) std::memmove(receive_buffer_, receive_buffer_ + total, tail);
        receive_length_ = tail;
    }
}

void MqttTelemetrySink::queueCommand(const char* topic, const uint8_t* payload, unsigned int length) {
#if NAVIGATION_ENABLE_MQTT_CONTROL
    if (topic == nullptr || payload == nullptr || std::strcmp(topic, command_topic_) != 0 || length == 0 || length >= sizeof(pending_command_)) return;
    portENTER_CRITICAL(&mux_);
    std::memcpy(pending_command_, payload, length);
    pending_command_[length] = '\0';
    command_pending_ = true;
    portEXIT_CRITICAL(&mux_);
#else
    (void)topic; (void)payload; (void)length;
#endif
}

bool MqttTelemetrySink::takeCommand(char* destination, std::size_t destination_size) {
    if (destination == nullptr || destination_size == 0) return false;
    bool found = false;
    portENTER_CRITICAL(&mux_);
    if (command_pending_) {
        std::strncpy(destination, pending_command_, destination_size - 1);
        destination[destination_size - 1] = '\0';
        command_pending_ = false;
        found = true;
    }
    portEXIT_CRITICAL(&mux_);
    return found;
}

void MqttTelemetrySink::queueResponse(const char* json_line) {
    if (json_line == nullptr || json_line[0] == '\0') return;
    portENTER_CRITICAL(&mux_);
    std::strncpy(pending_response_, json_line, sizeof(pending_response_) - 1);
    pending_response_[sizeof(pending_response_) - 1] = '\0';
    pending_mask_ |= static_cast<uint8_t>(1u << static_cast<uint8_t>(PendingKind::RESPONSE));
    portEXIT_CRITICAL(&mux_);
}

bool MqttTelemetrySink::publish(PendingKind kind, const char* payload) {
    if (connection_state_ != ConnectionState::CONNECTED || payload == nullptr) return false;
    const char* topic = telemetry_topic_;
    if (kind == PendingKind::WHEEL) topic = wheel_topic_;
    else if (kind == PendingKind::VERIFICATION) topic = verification_topic_;
    else if (kind == PendingKind::EVENT) topic = event_topic_;
    else if (kind == PendingKind::RESPONSE) topic = response_topic_;
    uint8_t packet[TELEMETRY_BUFFER + 96]{};
    std::size_t n = 0;
    const std::size_t payload_size = std::strlen(payload);
    if (!appendString(packet, sizeof(packet), n, topic) || n + payload_size > sizeof(packet)) return false;
    std::memcpy(packet + n, payload, payload_size);
    n += payload_size;
    if (!encodePacket(transport_, 0x30u, packet, n)) return false;
    last_io_ms_ = millis();
    return true;
}

bool MqttTelemetrySink::publishStatus(const char* payload, bool retained) {
    if (connection_state_ != ConnectionState::CONNECTED || payload == nullptr) return false;
    uint8_t packet[160]{};
    std::size_t n = 0;
    const std::size_t payload_size = std::strlen(payload);
    if (!appendString(packet, sizeof(packet), n, availability_topic_) ||
        n + payload_size > sizeof(packet)) return false;
    std::memcpy(packet + n, payload, payload_size);
    n += payload_size;
    if (!encodePacket(transport_, retained ? 0x31u : 0x30u, packet, n)) return false;
    last_io_ms_ = millis();
    return true;
}

void MqttTelemetrySink::service() {
    const bool connected = ensureConnected();
    if (connection_state_ == ConnectionState::WAITING_CONNACK) readPackets();
    if (!connected || connection_state_ != ConnectionState::CONNECTED) return;
    readPackets();
    if (connection_state_ != ConnectionState::CONNECTED) return;
    if (millis() - last_io_ms_ >= kKeepAliveIntervalMs) {
        const uint8_t ping[] = {0xc0, 0x00};
        if (transport_.write(ping, sizeof(ping)) != sizeof(ping)) { disconnect(); return; }
        last_io_ms_ = millis();
    }
    constexpr PendingKind kinds[] = {PendingKind::RESPONSE, PendingKind::EVENT,
                                     PendingKind::TELEMETRY, PendingKind::WHEEL,
                                     PendingKind::VERIFICATION};
    for (const PendingKind kind : kinds) {
        char line[TELEMETRY_BUFFER]{};
        const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(kind));
        bool found = false;
        portENTER_CRITICAL(&mux_);
        if ((pending_mask_ & bit) != 0u) {
            char* pending = pendingBuffer(kind);
            if (pending != nullptr) {
                std::strncpy(line, pending, sizeof(line) - 1);
                pending_mask_ &= static_cast<uint8_t>(~bit);
                found = true;
            }
        }
        portEXIT_CRITICAL(&mux_);
        if (!found) continue;
        if (publish(kind, line)) continue;
        portENTER_CRITICAL(&mux_);
        char* pending = pendingBuffer(kind);
        if (pending != nullptr) {
            std::strncpy(pending, line, TELEMETRY_BUFFER - 1);
            pending[TELEMETRY_BUFFER - 1] = '\0';
            pending_mask_ |= bit;
        }
        portEXIT_CRITICAL(&mux_);
        disconnect();
        return;
    }
}

MqttTelemetrySink& mqttTelemetry() {
    static MqttTelemetrySink instance;
    return instance;
}
}  // namespace firmware
