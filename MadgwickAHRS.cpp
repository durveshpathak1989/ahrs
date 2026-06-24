#include "MadgwickAHRS.h"

MadgwickAHRS::MadgwickAHRS()
{
    _beta = 0.08f;   // Conservative starting point for 400-1000 Hz IMU update.
    reset();
}

void MadgwickAHRS::reset()
{
    _q0 = 1.0f; _q1 = 0.0f; _q2 = 0.0f; _q3 = 0.0f;
}

void MadgwickAHRS::setBeta(float beta)
{
    _beta = beta;
}

void MadgwickAHRS::setQuaternion(float q0, float q1, float q2, float q3)
{
    const float n2 = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    if (n2 <= 0.000001f) {
        reset();
        return;
    }
    const float invN = ahrsFastInvSqrt(n2);
    _q0 = q0 * invN;
    _q1 = q1 * invN;
    _q2 = q2 * invN;
    _q3 = q3 * invN;
}

bool MadgwickAHRS::update(const AHRSInput& in, float dt, AttitudeEstimate& out)
{
    return updateIMU(in.ax_g, in.ay_g, in.az_g,
                     in.gx_dps, in.gy_dps, in.gz_dps,
                     dt, out);
}

bool MadgwickAHRS::updateIMU(float ax_g, float ay_g, float az_g,
                             float gx_dps, float gy_dps, float gz_dps,
                             float dt,
                             AttitudeEstimate& out)
{
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.001f;
    }

    float gx = gx_dps * AHRS_DEG_TO_RAD;
    float gy = gy_dps * AHRS_DEG_TO_RAD;
    float gz = gz_dps * AHRS_DEG_TO_RAD;

    float qDot0 = 0.5f * (-_q1*gx - _q2*gy - _q3*gz);
    float qDot1 = 0.5f * ( _q0*gx + _q2*gz - _q3*gy);
    float qDot2 = 0.5f * ( _q0*gy - _q1*gz + _q3*gx);
    float qDot3 = 0.5f * ( _q0*gz + _q1*gy - _q2*gx);

    if (ahrsAccelValid(ax_g, ay_g, az_g)) {
        const float invA = ahrsFastInvSqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
        const float ax = ax_g * invA;
        const float ay = ay_g * invA;
        const float az = az_g * invA;

        const float _2q0 = 2.0f * _q0;
        const float _2q1 = 2.0f * _q1;
        const float _2q2 = 2.0f * _q2;
        const float _2q3 = 2.0f * _q3;
        const float _4q0 = 4.0f * _q0;
        const float _4q1 = 4.0f * _q1;
        const float _4q2 = 4.0f * _q2;
        const float _8q1 = 8.0f * _q1;
        const float _8q2 = 8.0f * _q2;
        const float q0q0 = _q0 * _q0;
        const float q1q1 = _q1 * _q1;
        const float q2q2 = _q2 * _q2;
        const float q3q3 = _q3 * _q3;

        float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*_q1 - _2q0*ay - _4q1
                 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        float s2 = 4.0f*q0q0*_q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2
                 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        float s3 = 4.0f*q1q1*_q3 - _2q1*ax + 4.0f*q2q2*_q3 - _2q2*ay;

        const float sNorm2 = s0*s0 + s1*s1 + s2*s2 + s3*s3;
        if (sNorm2 > 0.000001f) {
            const float invS = ahrsFastInvSqrt(sNorm2);
            s0 *= invS; s1 *= invS; s2 *= invS; s3 *= invS;

            qDot0 -= _beta * s0;
            qDot1 -= _beta * s1;
            qDot2 -= _beta * s2;
            qDot3 -= _beta * s3;
        }
    }

    _q0 += qDot0 * dt;
    _q1 += qDot1 * dt;
    _q2 += qDot2 * dt;
    _q3 += qDot3 * dt;

    const float invQ = ahrsFastInvSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
    _q0 *= invQ; _q1 *= invQ; _q2 *= invQ; _q3 *= invQ;

    ahrsQuatToEuler(_q0, _q1, _q2, _q3, out);
    return true;
}
