#include "as5600_encoder.hpp"

namespace firmware {

namespace {
constexpr uint8_t kStatusRegister = 0x0B;
constexpr uint8_t kRawAngleRegister = 0x0C;
constexpr uint8_t kMagnetDetectedMask = 0x20;
}

bool As5600Device::readRegister(uint8_t reg, uint8_t* data, size_t length) {
    bus_.beginTransmission(kAs5600Address);
    bus_.write(reg);
    if (bus_.endTransmission(false) != 0) return false;
    const size_t received = bus_.requestFrom(kAs5600Address, length, true);
    if (received != length) return false;
    for (size_t i = 0; i < length; ++i) data[i] = static_cast<uint8_t>(bus_.read());
    return true;
}

bool As5600Device::read(uint16_t& raw_angle, bool& magnet_detected) {
    uint8_t status = 0;
    uint8_t angle[2] = {};
    if (!readRegister(kStatusRegister, &status, 1) ||
        !readRegister(kRawAngleRegister, angle, 2)) {
        return false;
    }
    magnet_detected = (status & kMagnetDetectedMask) != 0;
    raw_angle = static_cast<uint16_t>(((angle[0] & 0x0Fu) << 8u) | angle[1]);
    return magnet_detected;
}

bool DualAs5600::begin(const nav::WheelEncoderConfig& left_config,
                       const nav::WheelEncoderConfig& right_config) {
    if (!left_tracker_.configure(left_config) || !right_tracker_.configure(right_config)) {
        return false;
    }
    const bool left_ok = left_bus_.begin(kLeftEncoderSda, kLeftEncoderScl,
                                         kEncoderI2cFrequencyHz);
    const bool right_ok = right_bus_.begin(kRightEncoderSda, kRightEncoderScl,
                                           kEncoderI2cFrequencyHz);
    return left_ok && right_ok;
}

bool DualAs5600::sample(uint64_t now_us) {
    uint16_t left_raw = 0;
    uint16_t right_raw = 0;
    const bool left_read = left_device_.read(left_raw, left_magnet_);
    const bool right_read = right_device_.read(right_raw, right_magnet_);
    const bool left_ok = left_read && left_tracker_.update(left_raw, now_us);
    const bool right_ok = right_read && right_tracker_.update(right_raw, now_us);
    if (!left_ok) left_tracker_.markCommunicationFailure();
    if (!right_ok) right_tracker_.markCommunicationFailure();
    return left_ok && right_ok;
}

void DualAs5600::resetTrackers() {
    left_tracker_.reset();
    right_tracker_.reset();
}

}  // namespace firmware
