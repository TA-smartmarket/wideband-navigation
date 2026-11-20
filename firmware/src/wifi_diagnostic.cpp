// Motor-safe ESP32-S3 Wi-Fi / USB-CDC diagnostic image.
// This file is built only by the `esp32-s3-wifi-diagnostic` PlatformIO env.
#include <Arduino.h>
#include <WiFi.h>

#include "app_config.hpp"

namespace {
uint32_t last_attempt_ms = 0;
bool was_connected = false;

void startAssociation() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(NAVIGATION_WIFI_SSID, NAVIGATION_WIFI_PASSWORD);
    last_attempt_ms = millis();
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1200);  // lets the native USB CDC port enumerate after reset
    Serial.println();
    Serial.println("[WIFI-DIAG] BOOT: minimal diagnostic image running");
    Serial.printf("[WIFI-DIAG] MAC: %s\n", WiFi.macAddress().c_str());
    if (NAVIGATION_WIFI_SSID[0] == '\0') {
        Serial.println("[WIFI-DIAG] ERROR: Wi-Fi SSID is empty");
        return;
    }
    startAssociation();
}

void loop() {
    const uint32_t now = millis();
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED && !was_connected) {
        was_connected = true;
        Serial.printf("[WIFI-DIAG] Connected: SSID '%s', IP %s\n",
                      NAVIGATION_WIFI_SSID, WiFi.localIP().toString().c_str());
    } else if (status != WL_CONNECTED && was_connected) {
        was_connected = false;
        Serial.printf("[WIFI-DIAG] Disconnected from SSID '%s'\n", NAVIGATION_WIFI_SSID);
    }
    if (status != WL_CONNECTED && now - last_attempt_ms >= 10000) {
        startAssociation();
    }
    delay(20);
}
