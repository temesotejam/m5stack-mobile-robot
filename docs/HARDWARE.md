# Hardware configuration

## Target stack

- M5Stack Basic
- BASE AAA
- Module13.2 GoPlus2
- M5Stack GNSS Module

The firmware starts the sensors and communication interfaces, but does not move a servo or motor at startup.

## I2C bus

| Signal | Basic GPIO |
|---|---:|
| SDA | 21 |
| SCL | 22 |

The GoPlus2 and the GNSS module sensors share this bus.

| Device | Address / connection | Function |
|---|---|---|
| GoPlus2 | 0x38 | Servo controller |
| BMI270 | 0x69 or 0x68 | Accelerometer and gyroscope |
| BMM150 | BMI270 AUX | Magnetometer; not used by the attitude EKF |
| BMP280 | 0x76 or 0x77 | Temperature and pressure |

The firmware checks the known sensor addresses directly. It does not run a full I2C address scan.

## GNSS UART

UART2 is initialized at 38400 baud, 8-N-1:

| Signal | Basic GPIO |
|---|---:|
| GNSS TX | 16 (ESP32 RX) |
| GNSS RX | 17 (ESP32 TX) |

Set the GNSS module UART DIP switches to route the NEO-M9N UART to these pins.

Position messages are not decoded in this firmware. Incoming UART bytes are read and discarded so the serial receive buffer does not fill. The UART remains initialized for later GNSS use.

## BMI270 attitude EKF

The attitude EKF uses only the BMI270 accelerometer and gyroscope:

- BMI270 acceleration output is in G.
- BMI270 gyroscope output is in degrees/second.
- The M5Stack GNSS module library configures the accelerometer and gyroscope for 50 Hz.
- The EKF integrates gyro samples using measured elapsed time and corrects attitude from new accelerometer samples.
- Accelerometer correction is skipped when the measured magnitude is outside 0.75 g to 1.25 g or when the direction innovation is too large.

The EKF initializes roll and pitch from the first suitable acceleration sample. Yaw starts at zero. Since there is no heading reference in a 6-axis IMU, yaw is relative and drifts over time. Roll and pitch are gravity-referenced only when linear acceleration is small enough for the accelerometer to represent gravity.

The output axes follow the physical BMI270 module axes. No installation-angle or axis-alignment calibration is applied.

## GoPlus2 servo control

GoPlus2 is controlled over I2C at address 0x38. The GoPlus2 controller generates the servo signals; the Basic does not generate servo PWM directly.

Four servo channels are available. This firmware uses the servo functions only; the GoPlus2 DC motor functions are intentionally not included.

### Basic buttons

- A: select the next channel
- B: decrease the selected angle by 5 degrees
- C: increase the selected angle by 5 degrees

The angle is limited to 0..180 degrees. Selecting a channel does not move the servo. A servo command is sent only after pressing B/C or sending a serial command.

### Serial commands

Open the USB serial console at 115200 baud:

| Command | Action |
|---|---|
| s 0 90 | Set servo channel 0 to 90 degrees |
| p 0 1500 | Set channel 0 pulse width to 1500 us |
| status | Print sensor and connection status |
| att | Print EKF attitude |
| ? | Show help |

Pulse-width commands are limited to 500..2500 us. Confirm the connected servo's allowed pulse range and mechanical travel before using pulse-width control.

## BASE AAA and power

BASE AAA is a power module for the Basic; it has no software interface.

Do not assume the AAA cells can supply the peak current required by a servo. Confirm the GoPlus2 and servo supply voltage, current capacity, and ground connection before connecting a load. Keep motor/servo power wiring away from the GNSS magnetometer where practical.

## Initial check

1. Power the Basic and confirm the display starts.
2. Confirm GoPlus2, BMI270/BMM150, and BMP280 status on the display or serial console.
3. Keep the robot still after startup until the EKF initializes from a valid acceleration sample.
4. Check roll/pitch by slowly tilting the Basic; check yaw by rotating it around its vertical axis.
5. Check that the GNSS UART RX counter changes when the module sends data. No GNSS fix is required.
6. With the servo mechanism unloaded or safely disconnected, test one GoPlus2 channel at a time.
