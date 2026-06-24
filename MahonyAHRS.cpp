#include "MahonyAHRS.h"

MahonyAHRS::MahonyAHRS()
{
    _kp = 1.0f;
    _ki = 0.005f;
    reset();
}

void MahonyAHRS::reset()
{
    _q0 = 1.0f; _q1 = 0.0f; _q2 = 0.0f; _q3 = 0.0f;
    _ix = 0.0f; _iy = 0.0f; _iz = 0.0f;
}

void MahonyAHRS::setGains(float kp, float ki)
{
    _kp = kp;
    _ki = ki;
    if (_ki <= 0.0f) {
        _ix = _iy = _iz = 0.0f;
    }
}

void MahonyAHRS::setQuaternion(float q0, float q1, float q2, float q3)
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

bool MahonyAHRS::update(float ax_g, float ay_g, float az_g,
                        float gx_dps, float gy_dps, float gz_dps,
                        float mx_uT, float my_uT, float mz_uT,
                        bool magValid,
                        float dt,
                        AttitudeEstimate& out)
{
    AHRSInput in;
    in.ax_g = ax_g; in.ay_g = ay_g; in.az_g = az_g;
    in.gx_dps = gx_dps; in.gy_dps = gy_dps; in.gz_dps = gz_dps;
    in.mx_uT = mx_uT; in.my_uT = my_uT; in.mz_uT = mz_uT;
    in.magValid = magValid;
    return update(in, dt, out);
}

bool MahonyAHRS::update(const AHRSInput& in, float dt, AttitudeEstimate& out)
{
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.001f;
    }

    float gx = in.gx_dps * AHRS_DEG_TO_RAD;
    float gy = in.gy_dps * AHRS_DEG_TO_RAD;
    float gz = in.gz_dps * AHRS_DEG_TO_RAD;

    float ax = in.ax_g;
    float ay = in.ay_g;
    float az = in.az_g;

    float mx = in.mx_uT;
    float my = in.my_uT;
    float mz = in.mz_uT;

    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;

    if (ahrsAccelValid(ax, ay, az)) {
        const float invA = ahrsFastInvSqrt(ax*ax + ay*ay + az*az);
        ax *= invA; ay *= invA; az *= invA;

        // Estimated gravity direction from quaternion.
        const float vx = 2.0f * (_q1*_q3 - _q0*_q2);
        const float vy = 2.0f * (_q0*_q1 + _q2*_q3);
        const float vz = _q0*_q0 - _q1*_q1 - _q2*_q2 + _q3*_q3;

        // Error is measured gravity cross estimated gravity.
        ex += ay*vz - az*vy;
        ey += az*vx - ax*vz;
        ez += ax*vy - ay*vx;

        const float mNorm2 = mx*mx + my*my + mz*mz;
        if (in.magValid && mNorm2 > 0.01f) {
            const float invM = ahrsFastInvSqrt(mNorm2);
            mx *= invM; my *= invM; mz *= invM;

            const float hx = 2.0f * (
                mx*(0.5f - _q2*_q2 - _q3*_q3) +
                my*(_q1*_q2 - _q0*_q3) +
                mz*(_q1*_q3 + _q0*_q2));

            const float hy = 2.0f * (
                mx*(_q1*_q2 + _q0*_q3) +
                my*(0.5f - _q1*_q1 - _q3*_q3) +
                mz*(_q2*_q3 - _q0*_q1));

            const float bx = sqrtf(hx*hx + hy*hy);
            const float bz = 2.0f * (
                mx*(_q1*_q3 - _q0*_q2) +
                my*(_q2*_q3 + _q0*_q1) +
                mz*(0.5f - _q1*_q1 - _q2*_q2));

            const float wx = 2.0f * (
                bx*(0.5f - _q2*_q2 - _q3*_q3) +
                bz*(_q1*_q3 - _q0*_q2));

            const float wy = 2.0f * (
                bx*(_q1*_q2 - _q0*_q3) +
                bz*(_q0*_q1 + _q2*_q3));

            const float wz = 2.0f * (
                bx*(_q0*_q2 + _q1*_q3) +
                bz*(0.5f - _q1*_q1 - _q2*_q2));

            ex += my*wz - mz*wy;
            ey += mz*wx - mx*wz;
            ez += mx*wy - my*wx;
        }

        if (_ki > 0.0f) {
            _ix += _ki * ex * dt;
            _iy += _ki * ey * dt;
            _iz += _ki * ez * dt;
        } else {
            _ix = _iy = _iz = 0.0f;
        }

        gx += _kp * ex + _ix;
        gy += _kp * ey + _iy;
        gz += _kp * ez + _iz;
    }

    const float halfDt = 0.5f * dt;
    const float dq0 = (-_q1*gx - _q2*gy - _q3*gz) * halfDt;
    const float dq1 = ( _q0*gx + _q2*gz - _q3*gy) * halfDt;
    const float dq2 = ( _q0*gy - _q1*gz + _q3*gx) * halfDt;
    const float dq3 = ( _q0*gz + _q1*gy - _q2*gx) * halfDt;

    _q0 += dq0;
    _q1 += dq1;
    _q2 += dq2;
    _q3 += dq3;

    const float invQ = ahrsFastInvSqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
    _q0 *= invQ; _q1 *= invQ; _q2 *= invQ; _q3 *= invQ;

    ahrsQuatToEuler(_q0, _q1, _q2, _q3, out);
    return true;
}
