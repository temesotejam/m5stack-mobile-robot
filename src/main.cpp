#include <Arduino.h>
#include <M5Unified.h>
#include <Wire.h>
#include <M5Module_GNSS.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <stdio.h>
#include <string.h>

#include "drivers/GoPlus2Driver.h"
#include "estimation/AttitudeEkf.h"

namespace {

constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;

constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;
constexpr uint32_t kGnssBaudRate = 38400;

constexpr uint8_t kBmi270Addresses[] = {0x69, 0x68};
constexpr uint8_t kBmp280Addresses[] = {0x76, 0x77};

constexpr uint32_t kSensorPeriodMs = 20;       // BMI270 ODR: 50 Hz
constexpr uint32_t kBarometerPeriodMs = 500;   // 2 Hz
constexpr uint32_t kDisplayPeriodMs = 250;     // 4 Hz
constexpr uint32_t kStatusPeriodMs = 1000;     // 1 Hz
constexpr uint32_t kGoPlusRetryMs = 2000;

HardwareSerial GnssSerial(2);
TinyGPSPlus gnss;
BMI270::BMI270 imu;
Adafruit_BMP280 barometer(&Wire);
GoPlus2Driver goPlus2(Wire);
AttitudeEkf attitudeEkf;

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
uint32_t lastGnssByteMs = 0;
uint32_t lastGnssValidNmeaMs = 0;
uint32_t lastSensorMs = 0;
uint32_t lastBarometerMs = 0;
uint32_t lastDisplayMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastGoPlusRetryMs = 0;
uint32_t lastEkfGyroUs = 0;

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
        Serial.printf(" accel=%.1fHz gyro=%.1fHz",
                      imu.accelerationSampleRate(),
                      imu.gyroscopeSampleRate());
    }
    Serial.println();

    Serial.printf("[BMM150] %s (not used by attitude EKF)\n",
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
    GnssSerial.setRxBufferSize(1024);
    GnssSerial.begin(
        kGnssBaudRate, SERIAL_8N1, kGnssRxPin, kGnssTxPin);
    Serial.printf("[GNSS] UART2 active: %lu baud, RX=%d, TX=%d, NMEA parser ready\n",
                  static_cast<unsigned long>(kGnssBaudRate),
                  kGnssRxPin,
                  kGnssTxPin);
}

bool gnssUartRecent() {
    return gnssRxBytes > 0 && (millis() - lastGnssByteMs) < 3000;
}

bool gnssNmeaRecent() {
    return gnss.passedChecksum() > 0 &&
           (millis() - lastGnssValidNmeaMs) < 3000;
}

bool gnssHasFreshFix() {
    return gnssNmeaRecent() &&
           gnss.location.isValid() &&
           gnss.location.age() < 3000 &&
           gnss.satellites.isValid() &&
           gnss.satellites.age() < 3000 &&
           gnss.satellites.value() > 0;
}

const char* gnssStatusText() {
    if (!gnssUartRecent()) {
        return "NO UART";
    }
    if (!gnssNmeaRecent()) {
        return "RAW DATA";
    }
    return gnssHasFreshFix() ? "FIX" : "NMEA / NO FIX";
}

void pollGnssUart() {
    while (GnssSerial.available() > 0) {
        const char c = static_cast<char>(GnssSerial.read());
        ++gnssRxBytes;
        lastGnssByteMs = millis();

        // TinyGPSPlus returns true when a complete NMEA sentence has a
        // valid checksum. Unknown NMEA sentence types are still recognized.
        if (gnss.encode(c)) {
            lastGnssValidNmeaMs = millis();
        }
    }
}

void updateAttitudeEkf(bool accelUpdated, bool gyroUpdated) {
    if (!attitudeEkf.isInitialized() && accelUpdated) {
        if (attitudeEkf.initializeFromAcceleration(accelX, accelY, accelZ)) {
            lastEkfGyroUs = micros();
            Serial.println("[EKF] initialized from BMI270 acceleration; yaw=0");
        }
    }

    if (!attitudeEkf.isInitialized()) {
        return;
    }

    if (gyroUpdated) {
        const uint32_t nowUs = micros();
        if (lastEkfGyroUs != 0) {
            const float dtSeconds =
                static_cast<uint32_t>(nowUs - lastEkfGyroUs) * 1.0e-6F;
            attitudeEkf.predict(gyroX, gyroY, gyroZ, dtSeconds);
        }
        lastEkfGyroUs = nowUs;
    }

    if (accelUpdated) {
        (void)attitudeEkf.correctFromAcceleration(accelX, accelY, accelZ);
    }
}

void pollSensors() {
    if (!imuReady) {
        return;
    }

    bool accelUpdated = false;
    bool gyroUpdated = false;

    if (imu.accelerationAvailable() > 0) {
        accelUpdated = (imu.readAcceleration(accelX, accelY, accelZ) != 0);
    }

    if (imu.gyroscopeAvailable() > 0) {
        gyroUpdated = (imu.readGyroscope(gyroX, gyroY, gyroZ) != 0);
    }

    // BMM150 remains a separate hardware check; the attitude EKF does not use it.
    if (magnetometerReady && imu.magneticFieldAvailable() > 0) {
        magneticFieldValid =
            (imu.readMagneticField(magX, magY, magZ) != 0);
    }

    updateAttitudeEkf(accelUpdated, gyroUpdated);
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

void reportGnss() {
    Serial.printf("[GNSS] status=%s RX=%lu bytes NMEA_OK=%lu checksum_error=%lu\n",
                  gnssStatusText(),
                  static_cast<unsigned long>(gnssRxBytes),
                  static_cast<unsigned long>(gnss.passedChecksum()),
                  static_cast<unsigned long>(gnss.failedChecksum()));

    if (gnss.satellites.isValid() && gnss.satellites.age() < 3000) {
        Serial.printf("[GNSS] satellites_used=%lu",
                      static_cast<unsigned long>(gnss.satellites.value()));
        if (gnss.hdop.isValid() && gnss.hdop.age() < 3000) {
            Serial.printf(" HDOP=%.2f", gnss.hdop.hdop());
        }
        Serial.println();
    } else {
        Serial.println("[GNSS] satellite data waiting");
    }

    if (gnssHasFreshFix()) {
        Serial.printf("[GNSS FIX] lat=%.7f lon=%.7f",
                      gnss.location.lat(), gnss.location.lng());
        if (gnss.altitude.isValid() && gnss.altitude.age() < 3000) {
            Serial.printf(" altitude=%.1fm", gnss.altitude.meters());
        }
        if (gnss.speed.isValid() && gnss.speed.age() < 3000) {
            Serial.printf(" speed=%.2fkm/h", gnss.speed.kmph());
        }
        Serial.println();
    } else {
        Serial.println("[GNSS] waiting for a fresh position fix");
    }
}

void reportAttitude() {
    if (!attitudeEkf.isInitialized()) {
        Serial.println("[ATTITUDE] EKF waiting for a valid 1g accelerometer sample");
        return;
    }

    float roll = 0.0F, pitch = 0.0F, yaw = 0.0F;
    attitudeEkf.getEulerDegrees(roll, pitch, yaw);
    Serial.printf("[ATTITUDE deg] roll=%.2f pitch=%.2f yaw=%.2f (relative; yaw drifts)\n",
                  roll, pitch, yaw);
    Serial.printf("[EKF] accel correction: %s\n",
                  attitudeEkf.lastAccelCorrectionAccepted()
                      ? "accepted"
                      : "rejected");
}

void printHelp() {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  s <channel 0-3> <angle 0-180>");
    Serial.println("  p <channel 0-3> <pulse_us 500-2500>");
    Serial.println("  status");
    Serial.println("  gnss");
    Serial.println("  att");
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

    reportGnss();

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

    reportAttitude();
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

    if (strcmp(line, "gnss") == 0) {
        reportGnss();
        return;
    }

    if (strcmp(line, "att") == 0) {
        reportAttitude();
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

void drawDisplayRow(int y, uint16_t color, const char* text) {
    // Paint a fixed-width text row with its background color. This overwrites
    // old characters without clearing the whole LCD.
    char line[53];
    snprintf(line, sizeof(line), "%s", text);

    size_t length = strlen(line);
    while (length < 52) {
        line[length++] = ' ';
    }
    line[52] = '\0';

    M5.Display.setTextColor(color, TFT_BLACK);
    M5.Display.setCursor(6, y);
    M5.Display.print(line);
}

void drawStaticDisplay() {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setTextSize(1);
    drawDisplayRow(5, TFT_WHITE, "M5STACK MOBILE ROBOT");
    M5.Display.drawFastHLine(6, 158, 308, TFT_DARKGREY);
    drawDisplayRow(180, TFT_WHITE, "A: next CH   B: -5 deg   C: +5 deg");
    drawDisplayRow(195, TFT_WHITE, "Serial: s <ch> <deg> / p <ch> <us>");
    drawDisplayRow(210, TFT_YELLOW, "Servo moves only after a command.");
}

void renderDisplay() {
    char row[64];

    snprintf(row, sizeof(row), "GoPlus2:%s BMI270:%s BMP280:%s BMM:%s",
             goPlus2Ready ? "OK" : "--",
             imuReady ? "OK" : "--",
             barometerReady ? "OK" : "--",
             magnetometerReady ? "OK" : "--");
    drawDisplayRow(20, TFT_WHITE, row);

    snprintf(row, sizeof(row), "GNSS:%s RX:%lu",
             gnssStatusText(),
             static_cast<unsigned long>(gnssRxBytes));
    drawDisplayRow(35, TFT_WHITE, row);

    if (gnss.satellites.isValid() && gnss.satellites.age() < 3000) {
        if (gnss.hdop.isValid() && gnss.hdop.age() < 3000) {
            snprintf(row, sizeof(row), "SAT used:%lu  HDOP:%.2f",
                     static_cast<unsigned long>(gnss.satellites.value()),
                     gnss.hdop.hdop());
        } else {
            snprintf(row, sizeof(row), "SAT used:%lu  HDOP:--",
                     static_cast<unsigned long>(gnss.satellites.value()));
        }
    } else {
        snprintf(row, sizeof(row), "SAT used:--  HDOP:--");
    }
    drawDisplayRow(50, TFT_WHITE, row);

    if (gnssHasFreshFix()) {
        snprintf(row, sizeof(row), "LAT: %.6f", gnss.location.lat());
    } else {
        snprintf(row, sizeof(row), "LAT: waiting for fix");
    }
    drawDisplayRow(65, TFT_WHITE, row);

    if (gnssHasFreshFix()) {
        snprintf(row, sizeof(row), "LON: %.6f", gnss.location.lng());
    } else {
        snprintf(row, sizeof(row), "LON: waiting for fix");
    }
    drawDisplayRow(80, TFT_WHITE, row);

    if (imuReady) {
        snprintf(row, sizeof(row), "ACC[g] %.2f %.2f %.2f",
                 accelX, accelY, accelZ);
    } else {
        snprintf(row, sizeof(row), "ACC: not available");
    }
    drawDisplayRow(95, TFT_WHITE, row);

    if (imuReady) {
        snprintf(row, sizeof(row), "GYR[dps] %.1f %.1f %.1f",
                 gyroX, gyroY, gyroZ);
    } else {
        snprintf(row, sizeof(row), "GYR: not available");
    }
    drawDisplayRow(110, TFT_WHITE, row);

    if (attitudeEkf.isInitialized()) {
        float roll = 0.0F, pitch = 0.0F, yaw = 0.0F;
        attitudeEkf.getEulerDegrees(roll, pitch, yaw);
        snprintf(row, sizeof(row), "EKF R:%6.1f P:%6.1f deg", roll, pitch);
    } else {
        snprintf(row, sizeof(row), "EKF: waiting for stable acceleration");
    }
    drawDisplayRow(125, TFT_WHITE, row);

    if (attitudeEkf.isInitialized()) {
        float roll = 0.0F, pitch = 0.0F, yaw = 0.0F;
        attitudeEkf.getEulerDegrees(roll, pitch, yaw);
        snprintf(row, sizeof(row), "EKF Y:%6.1f deg (relative)", yaw);
    } else {
        snprintf(row, sizeof(row), "EKF Y: --");
    }
    drawDisplayRow(140, TFT_WHITE, row);

    snprintf(row, sizeof(row), "SERVO CH:%u  ANGLE:%u deg",
             selectedServoChannel, servoAngle);
    drawDisplayRow(165, TFT_WHITE, row);
}

}  // namespace

void setup() {
    auto config = M5.config();
    config.internal_imu = false;
    config.internal_rtc = false;
    config.internal_mic = false;
    config.internal_spk = false;
    M5.begin(config);
    Serial.begin(115200);

    M5.Display.setRotation(1);
    drawStaticDisplay();
    renderDisplay();
    lastDisplayMs = millis();

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
