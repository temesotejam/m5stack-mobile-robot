# M5Stack Mobile Robot

A compact M5Stack Basic firmware for the mobile robot hardware stack.

## Hardware

- M5Stack Basic
- BASE AAA
- Module13.2 GoPlus2
- M5Stack GNSS Module

## Firmware functions

- M5Stack display and A/B/C buttons
- GoPlus2 four-channel servo control
- BMI270 accelerometer and gyroscope
- BMM150 magnetometer through the BMI270 auxiliary interface
- BMP280 temperature and pressure
- GNSS module UART kept active at 38400 baud; incoming bytes are discarded without position parsing

The BASE AAA is a power module and does not need firmware code.

## Servo operation

The Basic buttons provide a small manual control:

- A: select the next servo channel (0 to 3)
- B: decrease the selected channel angle by 5 degrees
- C: increase the selected channel angle by 5 degrees

The serial console runs at 115200 baud and accepts:

\`\`\`text
s 0 90       Set channel 0 to 90 degrees
p 0 1500     Set channel 0 pulse width to 1500 us
status       Print sensor and connection status
?            Show command help
\`\`\`

Angle commands accept 0..180 degrees. Pulse-width commands accept 500..2500 us. No servo or motor command is sent automatically at startup. GoPlus2 DC motor control is not included in this firmware.

## Web flasher

Open the browser flasher:

https://temesotejam.github.io/m5stack-mobile-robot/

Use desktop Chrome or Edge over HTTPS, connect the M5Stack Basic by USB, and select its serial port. GitHub Actions builds the firmware and publishes the ESP Web Tools manifest and merged binary.

## Build locally

\`\`\`sh
pio run -e m5stack-basic
\`\`\`

## Hardware details

See [docs/HARDWARE.md](docs/HARDWARE.md) for the I2C bus, GNSS UART, and power notes.
