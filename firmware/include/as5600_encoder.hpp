#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "navigation/wheel_encoder.hpp"
#include "pin_config.hpp"

namespace firmware {

class As5600Device {
public:
    explicit As5600Device(TwoWire& bus) : bus_(bus) {}
    bool read(uint16_t& raw_angle, bool& magnet_detected);

private:
    bool readRegister(uint8_t reg, uint8_t* data, size_t length);
    TwoWire& bus_;
};

class DualAs5600 {
public:
    bool begin(const nav::WheelEncoderConfig& left_config,
               const nav::WheelEncoderConfig& right_config);
    bool sample(uint64_t now_us);
    void resetTrackers();

    const nav::WheelEncoderTracker& left() const { return left_tracker_; }
    const nav::WheelEncoderTracker& right() const { return right_tracker_; }
    bool leftMagnetDetected() const { return left_magnet_; }
    bool rightMagnetDetected() const { return right_magnet_; }

private:
    TwoWire left_bus_{0};
    TwoWire right_bus_{1};
    As5600Device left_device_{left_bus_};
    As5600Device right_device_{right_bus_};
    nav::WheelEncoderTracker left_tracker_{};
    nav::WheelEncoderTracker right_tracker_{};
    bool left_magnet_{false};
    bool right_magnet_{false};
};

}  // namespace firmware
