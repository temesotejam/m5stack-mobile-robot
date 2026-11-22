#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <M5Module_GNSS.h>
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <string.h>

#include "drivers/GoPlus2Driver.h"

namespace {

constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;

constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;
constexpr uint32_t kGnssBaudRate = 38400;

constexpr uint8_t kBmi270Addresses[] = {0x69, 0x68};
constexpr uint8_t kBmp280Addresses[] = {0x76, 0x77};

constexpr uint32_t kSensorPeriodMs = 20;       // 50 Hz
constexpr uint32_t kBarometerPeriodMs = 500;   // 2 Hz
constexpr uint32_t kDisplayPeriodMs = 250;     // 4 Hz
constexpr uint32_t kStatusPeriodMs = 1000;     // 1 Hz
constexpr uint32_t kGoPlusRetryMs = 2000;

HardwareSerial GnssSerial(2);
BMI270::BMI270 imu;
Adafruit_BMP280 barometer(&Wire);
GoPlus2Driver goPlus2(Wire);

bool goPlus2Ready = false;
bool imuReady = false;
bool magnetometerReady = false;
bool barometerReady = false;

uint8_t bmi270Address = 0;
uint8_t bmp280Address = 0;

float accelX = 0.0F;
float accelY = 0.0F;
float accelZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

int16_t magX = 0;
int16_t magY = 0;
int16_t magZ = 0;
bool magneticFieldValid = false;

float temperatureC = 0.0F;
float pressureHpa = 0.0F;

uint8_t selectedServoChannel = 0;
uint8_t servoAngle = 90;

uint32_t gnssRxBytes = 0;
uint32_t lastSensorMs = 0;
uint32_t lastBarometerMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastGoPlusRetryMs = 0;

char serialLine[48];
size_t serialLineLength = 0;

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

void initializeI2cDevices() {
    Wire.begin(kI2cSdaPin, kI2cSclPin, 100000);

    goPlus2Ready = goPlus2.begin();
    lastGoPlusRetryMs = millis();
    Serial.printf("[GoPlus2] %s at 0x38\n",
                  goPlus2Ready ? "ready" : "not found");

    for (const uint8_t address : kBmi270Addresses) {
        if (!i2cDevicePresent(address)) {
            continue;
        }

        if (imu.init(Wire, address)) {
            bmi270Address = address;
            imuReady = true;
            magnetometerReady = imu.bmm150isEnabled();
            break;
        }
    }

    Serial.printf("[BMI270] %s",
                  imuReady ? "ready" : "not found");
    if (imuReady) {
        Serial.printf(" at 0x%02X", bmi270Address);
    }
    Serial.println();

    Serial.printf("[BMM150] %s\n",
                  magnetometerReady ? "ready" : "not available");

    for (const uint8_t address : kBmp280Addresses) {
        if (barometer.begin(address)) {
            bmp280Address = address;
            barometerReady = true;
            break;
        }
    }

    Serial.printf("[BMP280] %s",
                  barometerReady ? "ready" : "not found");
    if (barometerReady) {
        Serial.printf(" at 0x%02X", bmp280Address);
    }
    Serial.println();
}

void initializeGnssUart() {
    GnssSerial.begin(
        kGnssBaudRate, SERIAL_8N1, kGnssRxPin, kGnssTxPin);
    Serial.printf("[GNSS] UART2 active: %lu baud, RX=%d, TX=%d\n",
                  static_cast<unsigned long>(kGnssBaudRate),
                  kGnssRxPin,
                  kGnssTxPin);
}

void pollGnssUart() {
    // Keep the UART link active without decoding GNSS position messages.
    while (GnssSerial.available() > 0) {
        (void)GnssSerial.read();
        ++gnssRxBytes;
    }
}

void pollSensors() {
    if (!imuReady) {
        return;
    }

    if (imu.accelerationAvailable() > 0) {
        (void)imu.readAcceleration(accelX, accelY, accelZ);
    }

    if (imu.gyroscopeAvailable() > 0) {
        (void)imu.readGyroscope(gyroX, gyroY, gyroZ);
    }

    if (magnetometerReady && imu.magneticFieldAvailable() > 0) {
        magneticFieldValid =
            (imu.readMagneticField(magX, magY, magZ) != 0);
    }
}

void pollBarometer() {
    if (!barometerReady) {
        return;
    }

    temperatureC = barometer.readTemperature();
    pressureHpa = barometer.readPressure() / 100.0F;
}

bool commandServoAngle(uint8_t channel, uint8_t angle) {
    if (!goPlus2Ready) {
        Serial.println("[SERVO] GoPlus2 is not connected");
        return false;
    }

    if (!goPlus2.setServoAngle(channel, angle)) {
        Serial.println("[SERVO] command failed");
        return false;
    }

    selectedServoChannel = channel;
    servoAngle = angle;
    Serial.printf("[SERVO] channel=%u angle=%u deg\n", channel, angle);
    return true;
}

bool commandServoPulse(uint8_t channel, uint16_t pulseWidthUs) {
    if (!goPlus2Ready) {
        Serial.println("[SERVO] GoPlus2 is not connected");
        return false;
    }

    if (pulseWidthUs < 500 || pulseWidthUs > 2500) {
        Serial.println("[SERVO] pulse width must be 500..2500 us");
        return false;
    }

    if (!goPlus2.setServoPulseWidth(channel, pulseWidthUs)) {
        Serial.println("[SERVO] command failed");
        return false;
    }

    selectedServoChannel = channel;
    Serial.printf("[SERVO] channel=%u pulse=%u us\n",
                  channel, pulseWidthUs);
    return true;
}

void printHelp() {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  s <channel 0-3> <angle 0-180>");
    Serial.println("  p <channel 0-3> <pulse_us 500-2500>");
    Serial.println("  status");
    Serial.println("  ?");
    Serial.println();
    Serial.println("Basic buttons: A=next channel, B=angle -5, C=angle +5");
}

void reportStatus() {
    Serial.printf("[STATUS] GoPlus2=%s BMI270=%s BMM150=%s BMP280=%s GNSS_UART=ON RX=%lu\n",
                  goPlus2Ready ? "OK" : "NO",
                  imuReady ? "OK" : "NO",
                  magnetometerReady ? "OK" : "NO",
                  barometerReady ? "OK" : "NO",
                  static_cast<unsigned long>(gnssRxBytes));

    if (imuReady) {
        Serial.printf("[ACC g] %.3f %.3f %.3f  [GYRO dps] %.2f %.2f %.2f\n",
                      accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
    }

    if (magnetometerReady && magneticFieldValid) {
        Serial.printf("[MAG uT] %d %d %d\n", magX, magY, magZ);
    }

    if (barometerReady) {
        Serial.printf("[BARO] %.2f C  %.2f hPa\n",
                      temperatureC, pressureHpa);
    }
}

void processSerialLine(char* line) {
    if (line[0] == '\0') {
        return;
    }

    if (strcmp(line, "?") == 0 || strcmp(line, "help") == 0) {
        printHelp();
        return;
    }

    if (strcmp(line, "status") == 0) {
        reportStatus();
        return;
    }

    unsigned int channel = 0;
    unsigned int value = 0;

    if ((line[0] == 's' || line[0] == 'S') &&
        sscanf(line + 1, "%u %u", &channel, &value) == 2) {
        if (channel > 3 || value > 180) {
            Serial.println("[SERVO] channel must be 0..3 and angle 0..180");
            return;
        }
        (void)commandServoAngle(
            static_cast<uint8_t>(channel),
            static_cast<uint8_t>(value));
        return;
    }

    if ((line[0] == 'p' || line[0] == 'P') &&
        sscanf(line + 1, "%u %u", &channel, &value) == 2) {
        if (channel > 3 || value > 2500) {
            Serial.println("[SERVO] invalid channel or pulse width");
            return;
        }
        (void)commandServoPulse(
            static_cast<uint8_t>(channel),
            static_cast<uint16_t>(value));
        return;
    }

    Serial.println("[SERIAL] unknown command. Type '?' for help.");
}

void pollSerialConsole() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            serialLine[serialLineLength] = '\0';
            processSerialLine(serialLine);
            serialLineLength = 0;
            continue;
        }

        if (serialLineLength < sizeof(serialLine) - 1) {
            serialLine[serialLineLength++] = c;
        } else {
            serialLineLength = 0;
            Serial.println("[SERIAL] command too long");
        }
    }
}

void pollButtons() {
    if (M5.BtnA.wasPressed()) {
        selectedServoChannel =
            static_cast<uint8_t>((selectedServoChannel + 1) % 4);
        Serial.printf("[SERVO] selected channel %u\n",
                      selectedServoChannel);
    }

    if (M5.BtnB.wasPressed()) {
        const uint8_t nextAngle =
            servoAngle >= 5 ? static_cast<uint8_t>(servoAngle - 5) : 0;
        (void)commandServoAngle(selectedServoChannel, nextAngle);
    }

    if (M5.BtnC.wasPressed()) {
        const uint8_t nextAngle =
            servoAngle <= 175 ? static_cast<uint8_t>(servoAngle + 5) : 180;
        (void)commandServoAngle(selectedServoChannel, nextAngle);
    }
}

void renderDisplay() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(6, 5);

    M5.Display.println("M5STACK MOBILE ROBOT");
    M5.Display.printf("GoPlus2:%s  BMI270:%s  MAG:%s\n",
                      goPlus2Ready ? "OK" : "--",
                      imuReady ? "OK" : "--",
                      magnetometerReady ? "OK" : "--");
    M5.Display.printf("BMP280:%s  GNSS UART:ON  RX:%lu\n",
                      barometerReady ? "OK" : "--",
                      static_cast<unsigned long>(gnssRxBytes));

    if (imuReady) {
        M5.Display.printf("ACC[g] %.2f %.2f %.2f\n",
                          accelX, accelY, accelZ);
        M5.Display.printf("GYR[dps] %.1f %.1f %.1f\n",
                          gyroX, gyroY, gyroZ);
    } else {
        M5.Display.println("IMU: not available");
        M5.Display.println();
    }

    if (magnetometerReady && magneticFieldValid) {
        M5.Display.printf("MAG[uT] %d %d %d\n", magX, magY, magZ);
    } else {
        M5.Display.println("MAG: not available");
    }

    if (barometerReady) {
        M5.Display.printf("TEMP: %.1f C  PRESS: %.1f hPa\n",
                          temperatureC, pressureHpa);
    } else {
        M5.Display.println("BARO: not available");
    }

    M5.Display.drawFastHLine(6, 151, 308, TFT_DARKGREY);
    M5.Display.printf("SERVO CH:%u  ANGLE:%u deg\n",
                      selectedServoChannel, servoAngle);
    M5.Display.println("A: next CH   B: -5 deg   C: +5 deg");
    M5.Display.println("Serial: s <ch> <deg> / p <ch> <us>");
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.println("Servo moves only after a command.");
}

}  // namespace

void setup() {
    M5.begin();
    Serial.begin(115200);

    M5.Display.setRotation(1);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(8, 8);
    M5.Display.println("M5Stack Mobile Robot");
    M5.Display.println("Initializing...");

    Serial.println();
    Serial.println("M5Stack Mobile Robot");
    Serial.println("Starting hardware...");

    initializeI2cDevices();
    initializeGnssUart();
    printHelp();

    Serial.println("[INFO] No servo or motor command is sent at startup.");
}

void loop() {
    M5.update();

    pollGnssUart();
    pollSerialConsole();
    pollButtons();

    const uint32_t now = millis();

    if (!goPlus2Ready && now - lastGoPlusRetryMs >= kGoPlusRetryMs) {
        lastGoPlusRetryMs = now;
        goPlus2Ready = goPlus2.begin();
    }

    if (now - lastSensorMs >= kSensorPeriodMs) {
        lastSensorMs = now;
        pollSensors();
    }

    if (now - lastBarometerMs >= kBarometerPeriodMs) {
        lastBarometerMs = now;
        pollBarometer();
    }

    if (now - lastDisplayMs >= kDisplayPeriodMs) {
        lastDisplayMs = now;
        renderDisplay();
    }

    if (now - lastStatusMs >= kStatusPeriodMs) {
        lastStatusMs = now;
        reportStatus();
    }

    delay(1);
}
