#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <M5Module_GNSS.h>
#include <Adafruit_BMP280.h>

#include "drivers/GoPlus2Driver.h"

namespace {

constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;

constexpr int kGnssRxPin = 16;  // ESP32 RX <- GNSS NEO-M9N TX
constexpr int kGnssTxPin = 17;  // ESP32 TX -> GNSS NEO-M9N RX
constexpr uint32_t kGnssBaudRate = 38400;

constexpr uint8_t kBmi270AddressCandidates[] = {0x69, 0x68};
constexpr uint8_t kBmp280AddressCandidates[] = {0x76, 0x77};

constexpr uint32_t kDisplayPeriodMs = 250;
constexpr uint32_t kSerialReportPeriodMs = 1000;

HardwareSerial GnssSerial(2);
TinyGPSPlus gps;
BMI270::BMI270 imu;
Adafruit_BMP280 barometer(&Wire);
GoPlus2Driver goPlus2(Wire);

bool goPlus2Ready = false;
bool imuReady = false;
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

uint32_t lastDisplayMs = 0;
uint32_t lastSerialReportMs = 0;

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

void scanI2cBus() {
    Serial.println("[I2C] scan start");

    for (uint8_t address = 1; address < 127; ++address) {
        if (i2cDevicePresent(address)) {
            Serial.printf("[I2C] found 0x%02X\n", address);
        }
    }

    Serial.println("[I2C] scan complete");
}

void initializeGnss() {
    GnssSerial.begin(
        kGnssBaudRate, SERIAL_8N1, kGnssRxPin, kGnssTxPin);
    Serial.printf(
        "[GNSS] Serial2 started: baud=%lu RX=%d TX=%d\n",
        static_cast<unsigned long>(kGnssBaudRate),
        kGnssRxPin,
        kGnssTxPin);
}

void initializeI2cDevices() {
    Wire.begin(kI2cSdaPin, kI2cSclPin, 100000);
    scanI2cBus();

    goPlus2Ready = goPlus2.begin();
    Serial.printf("[GoPlus2] %s (I2C 0x38)\n",
                  goPlus2Ready ? "detected" : "not detected");

    for (const uint8_t address : kBmi270AddressCandidates) {
        if (!i2cDevicePresent(address)) {
            continue;
        }

        Serial.printf("[BMI270] trying address 0x%02X\n", address);
        if (imu.init(Wire, address)) {
            bmi270Address = address;
            imuReady = true;
            break;
        }
    }

    Serial.printf("[BMI270] %s",
                  imuReady ? "initialized" : "not detected");
    if (imuReady) {
        Serial.printf(" at 0x%02X", bmi270Address);
    }
    Serial.println();

    for (const uint8_t address : kBmp280AddressCandidates) {
        if (barometer.begin(address)) {
            bmp280Address = address;
            barometerReady = true;
            break;
        }
    }

    Serial.printf("[BMP280] %s",
                  barometerReady ? "initialized" : "not detected");
    if (barometerReady) {
        Serial.printf(" at 0x%02X", bmp280Address);
    }
    Serial.println();
}

void pollGnss() {
    while (GnssSerial.available() > 0) {
        gps.encode(static_cast<char>(GnssSerial.read()));
    }
}

void pollImu() {
    if (!imuReady) {
        return;
    }

    if (imu.accelerationAvailable() > 0) {
        imu.readAcceleration(accelX, accelY, accelZ);
    }

    if (imu.gyroscopeAvailable() > 0) {
        imu.readGyroscope(gyroX, gyroY, gyroZ);
    }

    if (imu.magneticFieldAvailable() > 0) {
        magneticFieldValid =
            (imu.readMagneticField(magX, magY, magZ) == 0);
    }
}

bool gnssHasFreshFix() {
    return gps.location.isValid() && gps.location.age() < 2000;
}

void renderDisplay() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(6, 5);

    M5.Display.println("M5STACK MOBILE ROBOT");
    M5.Display.drawFastHLine(6, 20, 308, TFT_DARKGREY);

    M5.Display.printf("GoPlus2 : %s\n",
                      goPlus2Ready ? "READY" : "NOT FOUND");
    M5.Display.printf("BMI270  : %s",
                      imuReady ? "READY" : "NOT FOUND");
    if (imuReady) {
        M5.Display.printf(" (0x%02X)", bmi270Address);
    }
    M5.Display.println();

    M5.Display.printf("BMP280  : %s\n",
                      barometerReady ? "READY" : "NOT FOUND");

    M5.Display.drawFastHLine(6, 62, 308, TFT_DARKGREY);
    M5.Display.printf("GNSS: %s  SAT: %lu\n",
                      gnssHasFreshFix() ? "FIX" : "SEARCH",
                      static_cast<unsigned long>(
                          gps.satellites.isValid()
                              ? gps.satellites.value()
                              : 0));

    if (gps.location.isValid()) {
        M5.Display.printf("LAT: %.6f\n", gps.location.lat());
        M5.Display.printf("LON: %.6f\n", gps.location.lng());
    } else {
        M5.Display.println("LAT: --");
        M5.Display.println("LON: --");
    }

    M5.Display.drawFastHLine(6, 111, 308, TFT_DARKGREY);

    if (imuReady) {
        M5.Display.printf("ACC[g] %.2f  %.2f  %.2f\n",
                          accelX, accelY, accelZ);
        M5.Display.printf("GYR[dps] %.1f  %.1f  %.1f\n",
                          gyroX, gyroY, gyroZ);

        if (magneticFieldValid) {
            M5.Display.printf("MAG[uT] %d  %d  %d\n",
                              magX, magY, magZ);
        } else {
            M5.Display.println("MAG: waiting");
        }
    } else {
        M5.Display.println("IMU data: unavailable");
    }

    if (barometerReady) {
        M5.Display.printf("PRESS: %.1f hPa\n",
                          barometer.readPressure() / 100.0F);
    } else {
        M5.Display.println("PRESS: unavailable");
    }

    M5.Display.drawFastHLine(6, 194, 308, TFT_DARKGREY);
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.println("Actuators are idle at startup.");
}

void reportSerial() {
    const bool fix = gnssHasFreshFix();

    Serial.printf(
        "[STATUS] goplus2=%s imu=%s bmp=%s gps_fix=%s sats=%lu",
        goPlus2Ready ? "ok" : "missing",
        imuReady ? "ok" : "missing",
        barometerReady ? "ok" : "missing",
        fix ? "yes" : "no",
        static_cast<unsigned long>(
            gps.satellites.isValid() ? gps.satellites.value() : 0));

    if (gps.location.isValid()) {
        Serial.printf(" lat=%.7f lon=%.7f",
                      gps.location.lat(), gps.location.lng());
    }

    if (barometerReady) {
        Serial.printf(" pressure_hpa=%.2f",
                      barometer.readPressure() / 100.0F);
    }

    Serial.println();
}

}  // namespace

void setup() {
    M5.begin();
    Serial.begin(115200);

    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(8, 8);
    M5.Display.println("M5Stack Mobile Robot");
    M5.Display.println("Starting hardware check...");

    Serial.println();
    Serial.println("M5Stack Mobile Robot");
    Serial.println("Firmware: bring-up / sensor diagnostics");

    initializeI2cDevices();
    initializeGnss();

    Serial.println(
        "[SAFETY] No servo or motor command is sent during startup.");
}

void loop() {
    M5.update();

    pollGnss();
    pollImu();

    const uint32_t now = millis();

    if (now - lastDisplayMs >= kDisplayPeriodMs) {
        lastDisplayMs = now;
        renderDisplay();
    }

    if (now - lastSerialReportMs >= kSerialReportPeriodMs) {
        lastSerialReportMs = now;
        reportSerial();
    }

    delay(1);
}
