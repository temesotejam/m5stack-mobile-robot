#include "GoPlus2Driver.h"

namespace {
constexpr uint8_t kServoAngleRegister = 0x00;
constexpr uint8_t kServoPulseWidthRegister = 0x10;
constexpr uint16_t kMinServoPulseWidthUs = 500;
constexpr uint16_t kMaxServoPulseWidthUs = 2500;
}  // namespace

GoPlus2Driver::GoPlus2Driver(TwoWire& wire) : _wire(wire) {}

bool GoPlus2Driver::begin() {
    _wire.beginTransmission(kI2cAddress);
    _ready = (_wire.endTransmission() == 0);
    return _ready;
}

bool GoPlus2Driver::setServoAngle(uint8_t channel, uint8_t angle) {
    if (!_ready || channel >= kServoChannels || angle > 180) {
        return false;
    }

    const uint8_t value = angle;
    return writeRegister(
        static_cast<uint8_t>(kServoAngleRegister | channel), &value, 1);
}

bool GoPlus2Driver::setServoPulseWidth(uint8_t channel, uint16_t pulseWidthUs) {
    if (!_ready || channel >= kServoChannels ||
        pulseWidthUs < kMinServoPulseWidthUs ||
        pulseWidthUs > kMaxServoPulseWidthUs) {
        return false;
    }

    const uint8_t data[2] = {
        static_cast<uint8_t>((pulseWidthUs >> 8) & 0xFF),
        static_cast<uint8_t>(pulseWidthUs & 0xFF),
    };
    const uint8_t reg =
        static_cast<uint8_t>(kServoPulseWidthRegister | (channel * 2));

    return writeRegister(reg, data, sizeof(data));
}

bool GoPlus2Driver::isReady() const {
    return _ready;
}

bool GoPlus2Driver::writeRegister(
    uint8_t reg, const uint8_t* data, size_t length) {
    _wire.beginTransmission(kI2cAddress);
    _wire.write(reg);
    _wire.write(data, length);
    return _wire.endTransmission() == 0;
}
