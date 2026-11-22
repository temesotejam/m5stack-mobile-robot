#pragma once

class AttitudeEkf {
public:
    AttitudeEkf();
    void reset();

    // Initialize roll/pitch from a near-1g accelerometer sample; yaw starts at zero.
    bool initializeFromAcceleration(float axG, float ayG, float azG);

    // Gyroscope input is degrees/second; dt is seconds.
    void predict(float gxDps, float gyDps, float gzDps, float dtSeconds);

    // Accelerometer input is G. Returns false if the sample is rejected.
    bool correctFromAcceleration(float axG, float ayG, float azG);

    bool isInitialized() const;
    bool lastAccelCorrectionAccepted() const;
    bool getEulerDegrees(float& roll, float& pitch, float& yaw) const;

private:
    void normalizeQuaternion();

    // Nominal body-to-world quaternion [w, x, y, z].
    float _q[4];
    // Gyroscope bias estimate in rad/s, sensor coordinates.
    float _gyroBias[3];
    // Error-state covariance: [attitude error(3), gyro-bias error(3)].
    float _covariance[6][6];
    bool _initialized;
    bool _lastAccelCorrectionAccepted;
};
