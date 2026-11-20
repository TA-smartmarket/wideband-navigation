// Copy this file to local_secrets.hpp, then set values for this computer.
// local_secrets.hpp is ignored by Git.
#pragma once

#define NAVIGATION_WIFI_SSID "YOUR_WIFI_SSID"
#define NAVIGATION_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#define NAVIGATION_MQTT_HOST "192.168.0.100"
#define NAVIGATION_MQTT_PORT 1883
#define NAVIGATION_ENABLE_MQTT_TELEMETRY 1
// Enables only motor.manual, navigation.stop, estop and estop.reset from the
// trolley MQTT command topic. Use only on a protected laboratory LAN.
#define NAVIGATION_ENABLE_MQTT_CONTROL 0
// Set to 0 for a telemetry-only bench image with motor outputs inhibited.
// Production STEP generation must use an ISR that contains no float math.
#define NAVIGATION_ENABLE_STEPPER_TIMER 1

// Leave empty only on an isolated development LAN. If Mosquitto authentication
// is enabled, set both fields.
#define NAVIGATION_MQTT_USERNAME ""
#define NAVIGATION_MQTT_PASSWORD ""
