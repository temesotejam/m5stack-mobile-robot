# BMI270 6-axis attitude EKF

## Scope

The attitude estimator uses only the BMI270 accelerometer and gyroscope. It does not consume BMM150, GNSS, BMP280, servo, or wheel data.

The filter is a lightweight error-state EKF with:
- Nominal attitude: unit quaternion (body to world)
- Error state: three small attitude errors and three gyroscope-bias errors
- Prediction: gyroscope integration
- Measurement update: normalized accelerometer direction as the gravity reference

No dynamic memory or external matrix library is used.

## Sensor units and rate

The M5Stack BMI270 library returns:
- Acceleration in G
- Angular velocity in degrees/second

The library configures both accelerometer and gyroscope at 50 Hz. The firmware uses the measured time between gyro samples for quaternion propagation.

## Initialization

When the first accelerometer sample has a magnitude between 0.75 g and 1.25 g:
- The local world frame is initialized with +Z downward, matching the observed BMI270 resting output of approximately -1 g on Z.
- Roll is initialized from atan2(-ay, -az).
- Pitch is initialized from atan2(ax, sqrt(ay^2 + az^2)).
- Yaw is initialized to zero.

This makes yaw relative to the orientation at startup. With the module level, Roll and Pitch should be close to zero; small residual offsets can remain from sensor bias or mounting tilt.

## Prediction

The gyroscope is converted to rad/s and the estimated bias is subtracted. The quaternion is propagated with the body-frame angular velocity.

The error-state covariance is propagated using the linearized attitude and gyro-bias model.

## Accelerometer correction

The accelerometer vector is normalized and compared with the stationary specific-force direction predicted by the quaternion. The local world frame uses +Z downward, so the predicted stationary specific-force direction is R(q)^T * [0, 0, -1]. This matches the observed BMI270 sign when the module is level.

Two simple rejection checks are used:
1. Reject the sample if acceleration magnitude is outside 0.75 g to 1.25 g.
2. Reject a direction innovation that is too large for the current covariance.

This reduces the effect of strong linear acceleration and impacts. It cannot perfectly distinguish gravity from all forms of linear acceleration, especially when the total acceleration magnitude remains close to 1 g.

The covariance update uses the Joseph form to help maintain numerical symmetry and positive semidefiniteness.

## Observability and limitations

A 6-axis IMU has no absolute heading reference:
- Roll and pitch can be corrected from gravity while linear acceleration is small.
- Yaw cannot be corrected from gravity and therefore accumulates gyroscope bias and noise.
- Yaw is reported as a relative angle, not an absolute compass heading.
- During sustained acceleration, the accelerometer may not represent gravity; the magnitude and innovation gates reject some but not all such samples.
- Sensor mounting orientation is not calibrated in software. The output axes are the BMI270 sensor axes.

For absolute or long-term stable yaw, a heading reference such as a calibrated magnetometer, GNSS course while moving, or another external reference is required.

## Runtime output

The display shows roll, pitch, and relative yaw. The USB serial console runs at 115200 baud:
- att prints the current attitude and last accelerometer correction result.
- status prints device status and attitude.

The EKF runs from the existing 50 Hz sensor polling path. Display updates remain at 4 Hz.
