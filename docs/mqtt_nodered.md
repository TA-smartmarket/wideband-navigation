# MQTT monitoring with Node-RED

This integration mirrors telemetry and can optionally accept a tightly bounded
manual-control channel from the included Node-RED web page.

## Local network topology

The deployment configured in the untracked `firmware/include/local_secrets.hpp`
uses Wi-Fi `TP-LINK_7030` and expects the laptop broker at `192.168.0.100:1883`.
`127.0.0.1:1880` is Node-RED's local editor, not a broker address usable by the
ESP32.

The main FastAPI dashboard starts an embedded MQTT broker on port 1883 and
connects directly to it. Mosquitto and Node-RED are optional. The supplied
Node-RED flow remains usable as a secondary monitor through `127.0.0.1:1883`.

Keep the laptop's `192.168.0.100` address reserved in the router, or update
`NAVIGATION_MQTT_HOST` in the local secrets file before building.

## Topics

| Topic | Rate / retention | Purpose |
| --- | --- | --- |
| `smart_trolley/TROLLEY_01/telemetry` | 5 Hz, not retained | navigation telemetry JSON |
| `smart_trolley/TROLLEY_01/wheel_telemetry` | 5 Hz, not retained | wheel PID, encoder, odometry JSON |
| `smart_trolley/TROLLEY_01/position_verification` | 5 Hz, not retained | UWB/odometry agreement |
| `smart_trolley/TROLLEY_01/event` | event-driven, not retained | firmware event text |
| `smart_trolley/TROLLEY_01/status` | retained | `online` or broker LWT `offline` |
| `smart_trolley/TROLLEY_01/command` | not retained | bounded dashboard protocol JSON |
| `smart_trolley/TROLLEY_01/response` | not retained | command acknowledgement/configuration response |

Telemetry uses QoS 0 and keeps only the latest unsent packet. It will not queue
old samples during a broker/network outage.

## Web motor test

Set `NAVIGATION_ENABLE_MQTT_CONTROL` to `1` in the ignored
`firmware/include/local_secrets.hpp`, build, and upload. Open the normal
dashboard at `http://<laptop-LAN-IP>:8000`. The optional Node-RED test page is
available at `http://<laptop-LAN-IP>:1880/smart-trolley-control` after its flow
is imported and deployed.

Hold **Forward** or **Reverse**: the page publishes `motor.manual` every 100 ms;
releasing publishes zero speed. Firmware independently stops movement after
350 ms without a valid heartbeat. **STOP** and **E-STOP** are immediate command
paths. The same bounded parser also validates destination, PID, configuration,
position-source and odometry-pose commands before TaskNavigation consumes them.

Use the normal firmware image, not the Wi-Fi diagnostic image. Before a
wheels-raised test, verify TB6600 wiring, enable polarity, and E-stop. The STEP
timer uses the integer DDS ISR path; keep the manual heartbeat watchdog enabled.

## Security

`local_secrets.hpp` is ignored by Git. The example file contains placeholders.
The included Windows setup limits ports 1883 and 8000 to `LocalSubnet`, but the
embedded development broker allows anonymous clients. Use this only on a
trusted laboratory LAN. A deployed trolley should use authenticated MQTT/TLS
and a physical E-stop.
