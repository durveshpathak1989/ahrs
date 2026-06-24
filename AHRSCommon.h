/*
  AHRSCommon.h
  Common attitude data structures and helpers for the Test_Quad flight controller.

  AHRS = Attitude and Heading Reference System

  Units expected by all estimators:
    accel: g
    gyro:  deg/s
    mag:   uT, optional
    dt:    seconds
    output angles: degrees
*/

#pragma once
#ifndef AHRS_COMMON_H
#define AHRS_COMMON_H

#include <Arduino.h>
#include <math.h>

#ifndef AHRS_PI
#define AHRS_PI 3.14159265358979323846f
#endif

#ifndef AHRS_DEG_TO_RAD
#define AHRS_DEG_TO_RAD (AHRS_PI / 180.0f)
#endif

#ifndef AHRS_RAD_TO_DEG
#define AHRS_RAD_TO_DEG (180.0f / AHRS_PI)
#endif

struct AHRSInput {
    float ax_g = 0.0f;
    float ay_g = 0.0f;
    float az_g = 1.0f;

    float gx_dps = 0.0f;
    float gy_dps = 0.0f;
    float gz_dps = 0.0f;

    float mx_uT = 0.0f;
    float my_uT = 0.0f;
    float mz_uT = 0.0f;

    bool magValid = false;
};

struct AttitudeEstimate {
    float roll_deg  = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg   = 0.0f;

    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
};

static inline float ahrsFastInvSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    union { float f; uint32_t i; } conv = { x };
    conv.i = 0x5F3759DFul - (conv.i >> 1);
    conv.f *= 1.5f - (x * 0.5f * conv.f * conv.f);
    return conv.f;
}

static inline float ahrsWrap360(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    return deg;
}

static inline float ahrsWrap180(float deg)
{
    while (deg > 180.0f)   deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
}

static inline bool ahrsAccelValid(float ax, float ay, float az)
{
    const float n2 = ax*ax + ay*ay + az*az;
    return n2 > 0.0001f;
}

static inline void ahrsAccelAnglesDeg(float ax, float ay, float az,
                                      float& rollDeg, float& pitchDeg)
{
    // Same convention used for data analysis:
    // roll  = atan2(ay, az)
    // pitch = atan2(-ax, sqrt(ay^2 + az^2))
    rollDeg  = atan2f(ay, az) * AHRS_RAD_TO_DEG;
    pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az)) * AHRS_RAD_TO_DEG;
}

static inline void ahrsQuatToEuler(const float q0, const float q1,
                                   const float q2, const float q3,
                                   AttitudeEstimate& out)
{
    out.q0 = q0;
    out.q1 = q1;
    out.q2 = q2;
    out.q3 = q3;

    out.roll_deg = AHRS_RAD_TO_DEG *
        atan2f(2.0f * (q0*q1 + q2*q3),
               1.0f - 2.0f * (q1*q1 + q2*q2));

    const float sinP = constrain(2.0f * (q0*q2 - q3*q1), -1.0f, 1.0f);
    out.pitch_deg = AHRS_RAD_TO_DEG * asinf(sinP);

    out.yaw_deg = AHRS_RAD_TO_DEG *
        atan2f(2.0f * (q0*q3 + q1*q2),
               1.0f - 2.0f * (q2*q2 + q3*q3));

    out.yaw_deg = ahrsWrap360(out.yaw_deg);
}

#endif // AHRS_COMMON_H
