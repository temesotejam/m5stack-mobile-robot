#pragma once

#include <Arduino.h>
#include <Wire.h>

class GoPlus2Driver {
public:
    static constexpr uint8_t kI2cAddress = 0x38;
    static constexpr uint8_t kServoChannels = 4;

    explicit GoPlus2Driver(TwoWire& wire = Wire);

    // Checks whether a GoPlus2 is responding. Does not move any actuator.
    bool begin();

    // Sends a servo-angle command (0..180 degrees) to channel 0..3.
    bool setServoAngle(uint8_t channel, uint8_t angle);

    // Sends a pulse-width command in microseconds to channel 0..3.
    bool setServoPulseWidth(uint8_t channel, uint16_t pulseWidthUs);

    bool isReady() const;

private:
    bool writeRegister(uint8_t reg, const uint8_t* data, size_t length);

    TwoWire& _wire;
    bool _ready = false;
};
