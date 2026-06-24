#include "RollPitchEKF.h"

RollPitchEKF::RollPitchEKF()
{
    _angleQ = 0.0005f;
    _biasQ = 0.00001f;
    _accelAngleR = 0.08f;  // rad^2-ish equivalent; tune from logs.
    reset();
}

void RollPitchEKF::reset()
{
    _roll = 0.0f;
    _pitch = 0.0f;
    _yaw = 0.0f;
    _bgx = 0.0f;
    _bgy = 0.0f;

    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            _covP[r][c] = 0.0f;
        }
    }

    setInitialUncertainty(0.05f, 0.10f);
}

void RollPitchEKF::setProcessNoise(float angleQ, float biasQ)
{
    _angleQ = angleQ;
    _biasQ = biasQ;
}

void RollPitchEKF::setMeasurementNoise(float accelAngleR)
{
    _accelAngleR = accelAngleR;
}

void RollPitchEKF::setInitialUncertainty(float angleP, float biasP)
{
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            _covP[r][c] = 0.0f;
        }
    }
    _covP[0][0] = angleP;
    _covP[1][1] = angleP;
    _covP[2][2] = biasP;
    _covP[3][3] = biasP;
}

void RollPitchEKF::_updateOneAxis(int angleIndex, int biasIndex, float measurementRad)
{
    // Measurement model: z = angle + v
    // H = [1, 0] for the selected axis/bias pair.
    const float y = measurementRad - (angleIndex == 0 ? _roll : _pitch);

    const float S = _covP[angleIndex][angleIndex] + _accelAngleR;
    if (S <= 0.000001f) return;

    float K[4];
    for (int i = 0; i < 4; i++) {
        K[i] = _covP[i][angleIndex] / S;
    }

    _roll  += K[0] * y;
    _pitch += K[1] * y;
    _bgx   += K[2] * y;
    _bgy   += K[3] * y;

    // P = (I - K H) P
    float oldRow[4];
    for (int j = 0; j < 4; j++) {
        oldRow[j] = _covP[angleIndex][j];
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            _covP[i][j] -= K[i] * oldRow[j];
        }
    }
}

bool RollPitchEKF::update(const AHRSInput& in, float dt, AttitudeEstimate& out)
{
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.001f;
    }

    const float gx = in.gx_dps * AHRS_DEG_TO_RAD;
    const float gy = in.gy_dps * AHRS_DEG_TO_RAD;
    const float gz = in.gz_dps * AHRS_DEG_TO_RAD;

    // Predict state. This small EKF intentionally uses a simple small-angle
    // roll/pitch model. It is not meant for acro flips or full navigation.
    _roll  += (gx - _bgx) * dt;
    _pitch += (gy - _bgy) * dt;
    _yaw   += gz * dt;

    // Predict covariance for each angle/bias pair:
    // angle_k = angle + dt*(gyro - bias)
    // bias_k  = bias
    for (int axis = 0; axis < 2; axis++) {
        const int a = axis;      // 0 roll, 1 pitch
        const int b = axis + 2;  // 2 gx bias, 3 gy bias

        const float Paa = _covP[a][a];
        const float Pab = _covP[a][b];
        const float Pba = _covP[b][a];
        const float Pbb = _covP[b][b];

        _covP[a][a] = Paa - dt*Pba - dt*Pab + dt*dt*Pbb + _angleQ;
        _covP[a][b] = Pab - dt*Pbb;
        _covP[b][a] = Pba - dt*Pbb;
        _covP[b][b] = Pbb + _biasQ;
    }

    if (ahrsAccelValid(in.ax_g, in.ay_g, in.az_g)) {
        float accRollDeg = 0.0f;
        float accPitchDeg = 0.0f;
        ahrsAccelAnglesDeg(in.ax_g, in.ay_g, in.az_g, accRollDeg, accPitchDeg);

        _updateOneAxis(0, 2, accRollDeg * AHRS_DEG_TO_RAD);
        _updateOneAxis(1, 3, accPitchDeg * AHRS_DEG_TO_RAD);
    }

    _yaw = ahrsWrap180(_yaw * AHRS_RAD_TO_DEG) * AHRS_DEG_TO_RAD;

    _quatFromEuler(_roll, _pitch, _yaw, out);
    out.roll_deg = _roll * AHRS_RAD_TO_DEG;
    out.pitch_deg = _pitch * AHRS_RAD_TO_DEG;
    out.yaw_deg = ahrsWrap360(_yaw * AHRS_RAD_TO_DEG);

    return true;
}

void RollPitchEKF::_quatFromEuler(float roll, float pitch, float yaw, AttitudeEstimate& out)
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    out.q0 = cr*cp*cy + sr*sp*sy;
    out.q1 = sr*cp*cy - cr*sp*sy;
    out.q2 = cr*sp*cy + sr*cp*sy;
    out.q3 = cr*cp*sy - sr*sp*cy;
}
