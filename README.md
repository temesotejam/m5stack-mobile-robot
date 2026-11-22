# M5Stack Mobile Robot

A compact M5Stack Basic firmware for the mobile robot hardware stack.

## Hardware

- M5Stack Basic
- BASE AAA
- Module13.2 GoPlus2
- M5Stack GNSS Module (NEO-M9N)

## Firmware functions

- M5Stack display and A/B/C buttons
- GoPlus2 four-channel servo control
- BMI270 accelerometer and gyroscope
- 6-axis attitude EKF using BMI270 acceleration and gyroscope only
- BMM150 magnetometer hardware check (not used by the attitude EKF)
- BMP280 temperature and pressure
- GNSS NMEA parsing at 38400 baud: UART reception, valid sentence/checksum count, fix status, satellites used, HDOP, latitude, longitude, altitude and speed

The BASE AAA is a power module and does not need firmware code.

## GNSS status

The GNSS UART is configured for 38400 baud, 8-N-1 on RX=GPIO16 and TX=GPIO17. The firmware parses incoming NMEA data with TinyGPSPlus.

The display distinguishes:
- NO UART: no recent serial bytes
- RAW DATA: bytes arrive but no valid NMEA checksum has been received recently
- NMEA / NO FIX: valid NMEA sentences arrive, but there is no fresh position fix
- FIX: a fresh position and satellite-use report are available

The display shows satellites used, HDOP, latitude and longitude. The USB serial console command gnss prints RX bytes, valid and failed checksums, satellite count, HDOP, position, altitude and speed.

A GNSS fix is not expected indoors. For reception testing, place the supplied active antenna outdoors with a clear view of the sky. UART reception and valid NMEA sentences can be checked before a position fix is obtained.

## Attitude EKF

The EKF estimates a body-to-world quaternion and outputs roll, pitch, and yaw in degrees.

- Gyroscope: quaternion prediction at the BMI270 sample rate (50 Hz)
- Accelerometer: gravity-direction correction when acceleration magnitude is between 0.75 g and 1.25 g
- Initialization: roll/pitch from acceleration in a local +Z-down frame, yaw set to zero
- Acceleration outliers: rejected by magnitude and innovation gates
- Yaw: relative to startup and expected to drift because no magnetometer or other heading reference is used

The EKF uses only the BMI270 accelerometer and gyroscope. It does not use the GNSS module's BMM150, GNSS position, BMP280, or wheel data.

Serial commands:
- att: print roll, pitch, yaw, and accelerometer correction status
- gnss: print GNSS UART/NMEA/fix status and measurements
- status: print device and sensor status

See docs/ATTITUDE_EKF.md for the state model and limitations.

## Servo operation

The Basic buttons provide a small manual control:

- A: select the next servo channel (0 to 3)
- B: decrease the selected channel angle by 5 degrees
- C: increase the selected channel angle by 5 degrees

The serial console runs at 115200 baud and accepts:

    s 0 90       Set channel 0 to 90 degrees
    p 0 1500     Set channel 0 pulse width to 1500 us
    status       Print sensor and connection status
    gnss         Print GNSS status and measurements
    att          Print EKF attitude
    ?            Show command help

Angle commands accept 0..180 degrees. Pulse-width commands accept 500..2500 us. No servo or motor command is sent automatically at startup. GoPlus2 DC motor control is not included in this firmware.

## Web flasher

Open the browser flasher:

https://temesotejam.github.io/m5stack-mobile-robot/

Use desktop Chrome or Edge over HTTPS, connect the M5Stack Basic by USB, and select its serial port. GitHub Actions builds the firmware and publishes the ESP Web Tools manifest and merged binary.

## Build locally

    pio run -e m5stack-basic

## Hardware details

See docs/HARDWARE.md for the I2C bus, GNSS UART, and power notes.
