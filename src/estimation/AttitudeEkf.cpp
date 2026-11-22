#include "AttitudeEkf.h"

#include <math.h>
#include <string.h>

namespace {

constexpr float kDegToRad = 0.017453292519943295F;
constexpr float kRadToDeg = 57.29577951308232F;
constexpr float kMinGravityG = 0.75F;
constexpr float kMaxGravityG = 1.25F;
constexpr float kMaxPredictDt = 0.10F;

// Conservative initial noise densities; tune later from stationary/moving logs.
constexpr float kGyroNoiseDensity = 0.012F;        // rad/s/sqrt(Hz)
constexpr float kGyroBiasNoiseDensity = 0.00025F;  // rad/s/sqrt(Hz)
constexpr float kAccelDirectionSigma = 0.04F;

bool invert3x3(const float a[3][3], float inverse[3][3]) {
    const float c00 = a[1][1] * a[2][2] - a[1][2] * a[2][1];
    const float c01 = a[1][2] * a[2][0] - a[1][0] * a[2][2];
    const float c02 = a[1][0] * a[2][1] - a[1][1] * a[2][0];
    const float determinant =
        a[0][0] * c00 + a[0][1] * c01 + a[0][2] * c02;

    if (!isfinite(determinant) || fabsf(determinant) < 1.0e-9F) {
        return false;
    }

    const float invDet = 1.0F / determinant;
    inverse[0][0] = c00 * invDet;
    inverse[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invDet;
    inverse[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invDet;
    inverse[1][0] = c01 * invDet;
    inverse[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invDet;
    inverse[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * invDet;
    inverse[2][0] = c02 * invDet;
    inverse[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * invDet;
    inverse[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * invDet;
    return true;
}

}  // namespace

AttitudeEkf::AttitudeEkf() {
    reset();
}

void AttitudeEkf::reset() {
    _q[0] = 1.0F;
    _q[1] = _q[2] = _q[3] = 0.0F;
    for (int i = 0; i < 3; ++i) {
        _gyroBias[i] = 0.0F;
    }
    memset(_covariance, 0, sizeof(_covariance));
    _initialized = false;
    _lastAccelCorrectionAccepted = false;
}

bool AttitudeEkf::initializeFromAcceleration(
    float axG, float ayG, float azG) {
    const float magnitude = sqrtf(axG * axG + ayG * ayG + azG * azG);
    if (!isfinite(magnitude) ||
        magnitude < kMinGravityG || magnitude > kMaxGravityG) {
        return false;
    }

    // This BMI270 is mounted so its +Z sensing axis points downward
    // when the robot is level. Use a local world frame with +Z downward.
    // At rest, specific force is therefore measured as approximately -1 g on Z.
    const float roll = atan2f(-ayG, -azG);
    const float pitch = atan2f(axG, sqrtf(ayG * ayG + azG * azG));
    const float cr = cosf(roll * 0.5F);
    const float sr = sinf(roll * 0.5F);
    const float cp = cosf(pitch * 0.5F);
    const float sp = sinf(pitch * 0.5F);

    // ZYX Euler order; yaw is unobservable and initialized to zero.
    _q[0] = cr * cp;
    _q[1] = sr * cp;
    _q[2] = cr * sp;
    _q[3] = -sr * sp;
    normalizeQuaternion();

    for (int i = 0; i < 6; ++i) {
        _covariance[i][i] = (i < 3) ? 0.05F : 0.01F;
    }

    _initialized = true;
    _lastAccelCorrectionAccepted = true;
    return true;
}

void AttitudeEkf::predict(
    float gxDps, float gyDps, float gzDps, float dtSeconds) {
    if (!_initialized || !isfinite(dtSeconds) ||
        dtSeconds <= 0.0F || dtSeconds > kMaxPredictDt) {
        return;
    }

    const float omega[3] = {
        gxDps * kDegToRad - _gyroBias[0],
        gyDps * kDegToRad - _gyroBias[1],
        gzDps * kDegToRad - _gyroBias[2],
    };
    const float speed = sqrtf(
        omega[0] * omega[0] +
        omega[1] * omega[1] +
        omega[2] * omega[2]);

    float dq[4];
    const float angle = speed * dtSeconds;
    if (speed > 1.0e-6F) {
        const float halfAngle = 0.5F * angle;
        const float scale = sinf(halfAngle) / speed;
        dq[0] = cosf(halfAngle);
        dq[1] = omega[0] * scale;
        dq[2] = omega[1] * scale;
        dq[3] = omega[2] * scale;
    } else {
        dq[0] = 1.0F;
        dq[1] = 0.5F * omega[0] * dtSeconds;
        dq[2] = 0.5F * omega[1] * dtSeconds;
        dq[3] = 0.5F * omega[2] * dtSeconds;
    }

    // q <- q * delta_q; gyro is expressed in the sensor/body frame.
    const float w = _q[0], x = _q[1], y = _q[2], z = _q[3];
    _q[0] = w * dq[0] - x * dq[1] - y * dq[2] - z * dq[3];
    _q[1] = w * dq[1] + x * dq[0] + y * dq[3] - z * dq[2];
    _q[2] = w * dq[2] - x * dq[3] + y * dq[0] + z * dq[1];
    _q[3] = w * dq[3] + x * dq[2] - y * dq[1] + z * dq[0];
    normalizeQuaternion();

    // First-order error-state transition:
    // d(theta)/dt = -skew(omega)*theta - gyro_bias_error.
    float F[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        F[i][i] = 1.0F;
    }
    const float wx = omega[0], wy = omega[1], wz = omega[2];
    F[0][1] =  wz * dtSeconds;
    F[0][2] = -wy * dtSeconds;
    F[1][0] = -wz * dtSeconds;
    F[1][2] =  wx * dtSeconds;
    F[2][0] =  wy * dtSeconds;
    F[2][1] = -wx * dtSeconds;
    F[0][3] = -dtSeconds;
    F[1][4] = -dtSeconds;
    F[2][5] = -dtSeconds;

    float temp[6][6] = {};
    float propagated[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                temp[i][j] += F[i][k] * _covariance[k][j];
            }
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                propagated[i][j] += temp[i][k] * F[j][k];
            }
        }
    }

    const float gyroNoiseVariance =
        kGyroNoiseDensity * kGyroNoiseDensity * dtSeconds;
    const float biasNoiseVariance =
        kGyroBiasNoiseDensity * kGyroBiasNoiseDensity * dtSeconds;
    for (int i = 0; i < 3; ++i) {
        propagated[i][i] += gyroNoiseVariance;
        propagated[i + 3][i + 3] += biasNoiseVariance;
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            _covariance[i][j] = 0.5F * (propagated[i][j] + propagated[j][i]);
        }
    }
}

bool AttitudeEkf::correctFromAcceleration(
    float axG, float ayG, float azG) {
    _lastAccelCorrectionAccepted = false;
    if (!_initialized) {
        return false;
    }

    const float magnitude = sqrtf(axG * axG + ayG * ayG + azG * azG);
    if (!isfinite(magnitude) ||
        magnitude < kMinGravityG || magnitude > kMaxGravityG) {
        return false;
    }

    const float measurement[3] = {
        axG / magnitude, ayG / magnitude, azG / magnitude
    };

    // Predicted stationary specific-force direction in sensor coordinates.
    // The local world frame uses +Z downward, so at rest:
    // h = R(q)^T * [0, 0, -1].
    const float w = _q[0], x = _q[1], y = _q[2], z = _q[3];
    const float h[3] = {
        -2.0F * (x * z - w * y),
        -2.0F * (y * z + w * x),
        -1.0F + 2.0F * (x * x + y * y),
    };
    const float residual[3] = {
        measurement[0] - h[0],
        measurement[1] - h[1],
        measurement[2] - h[2],
    };

    // Jacobian for right-multiplicative attitude error.
    const float H[3][6] = {
        { 0.0F, -h[2],  h[1], 0.0F, 0.0F, 0.0F },
        { h[2],  0.0F, -h[0], 0.0F, 0.0F, 0.0F },
        {-h[1],  h[0],  0.0F, 0.0F, 0.0F, 0.0F },
    };

    const float magnitudeError = magnitude - 1.0F;
    const float measurementVariance =
        kAccelDirectionSigma * kAccelDirectionSigma *
        (1.0F + 20.0F * magnitudeError * magnitudeError);

    float S[3][3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int a = 0; a < 6; ++a) {
                for (int b = 0; b < 6; ++b) {
                    S[i][j] += H[i][a] * _covariance[a][b] * H[j][b];
                }
            }
        }
        S[i][i] += measurementVariance;
    }

    float invS[3][3] = {};
    if (!invert3x3(S, invS)) {
        return false;
    }

    // Reject large direction innovations that are unlikely to be gravity.
    float mahalanobis = 0.0F;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mahalanobis += residual[i] * invS[i][j] * residual[j];
        }
    }
    if (!isfinite(mahalanobis) || mahalanobis > 25.0F) {
        return false;
    }

    float PHt[6][3] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 6; ++k) {
                PHt[i][j] += _covariance[i][k] * H[j][k];
            }
        }
    }

    float K[6][3] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                K[i][j] += PHt[i][k] * invS[k][j];
            }
        }
    }

    float correction[6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            correction[i] += K[i][j] * residual[j];
        }
    }

    // Inject attitude correction into the nominal quaternion.
    const float dx = correction[0];
    const float dy = correction[1];
    const float dz = correction[2];
    const float angle = sqrtf(dx * dx + dy * dy + dz * dz);
    float dq[4];
    if (angle > 1.0e-6F) {
        const float halfAngle = 0.5F * angle;
        const float scale = sinf(halfAngle) / angle;
        dq[0] = cosf(halfAngle);
        dq[1] = dx * scale;
        dq[2] = dy * scale;
        dq[3] = dz * scale;
    } else {
        dq[0] = 1.0F;
        dq[1] = 0.5F * dx;
        dq[2] = 0.5F * dy;
        dq[3] = 0.5F * dz;
    }

    const float qw = _q[0], qx = _q[1], qy = _q[2], qz = _q[3];
    _q[0] = qw * dq[0] - qx * dq[1] - qy * dq[2] - qz * dq[3];
    _q[1] = qw * dq[1] + qx * dq[0] + qy * dq[3] - qz * dq[2];
    _q[2] = qw * dq[2] - qx * dq[3] + qy * dq[0] + qz * dq[1];
    _q[3] = qw * dq[3] + qx * dq[2] - qy * dq[1] + qz * dq[0];
    normalizeQuaternion();

    for (int i = 0; i < 3; ++i) {
        _gyroBias[i] += correction[i + 3];
    }

    // Joseph covariance update.
    float A[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            A[i][j] = (i == j) ? 1.0F : 0.0F;
            for (int k = 0; k < 3; ++k) {
                A[i][j] -= K[i][k] * H[k][j];
            }
        }
    }

    float AP[6][6] = {};
    float updated[6][6] = {};
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                AP[i][j] += A[i][k] * _covariance[k][j];
            }
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            for (int k = 0; k < 6; ++k) {
                updated[i][j] += AP[i][k] * A[j][k];
            }
            for (int k = 0; k < 3; ++k) {
                updated[i][j] +=
                    K[i][k] * measurementVariance * K[j][k];
            }
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            _covariance[i][j] = 0.5F * (updated[i][j] + updated[j][i]);
        }
    }

    _lastAccelCorrectionAccepted = true;
    return true;
}

bool AttitudeEkf::isInitialized() const {
    return _initialized;
}

bool AttitudeEkf::lastAccelCorrectionAccepted() const {
    return _lastAccelCorrectionAccepted;
}

bool AttitudeEkf::getEulerDegrees(
    float& roll, float& pitch, float& yaw) const {
    if (!_initialized) {
        roll = pitch = yaw = 0.0F;
        return false;
    }

    const float w = _q[0], x = _q[1], y = _q[2], z = _q[3];
    const float sinRoll = 2.0F * (w * x + y * z);
    const float cosRoll = 1.0F - 2.0F * (x * x + y * y);
    const float sinPitch = 2.0F * (w * y - z * x);
    const float sinYaw = 2.0F * (w * z + x * y);
    const float cosYaw = 1.0F - 2.0F * (y * y + z * z);
    const float clampedSinPitch =
        (sinPitch > 1.0F) ? 1.0F :
        (sinPitch < -1.0F) ? -1.0F : sinPitch;

    roll = atan2f(sinRoll, cosRoll) * kRadToDeg;
    pitch = asinf(clampedSinPitch) * kRadToDeg;
    yaw = atan2f(sinYaw, cosYaw) * kRadToDeg;
    return true;
}

void AttitudeEkf::normalizeQuaternion() {
    const float norm = sqrtf(
        _q[0] * _q[0] + _q[1] * _q[1] +
        _q[2] * _q[2] + _q[3] * _q[3]);

    if (!isfinite(norm) || norm < 1.0e-8F) {
        _q[0] = 1.0F;
        _q[1] = _q[2] = _q[3] = 0.0F;
        return;
    }

    const float inverseNorm = 1.0F / norm;
    for (int i = 0; i < 4; ++i) {
        _q[i] *= inverseNorm;
    }
}
