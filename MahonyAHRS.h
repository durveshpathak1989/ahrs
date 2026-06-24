/*
  MahonyAHRS.h
  Standalone Mahony AHRS estimator for Test_Quad.

  Good for:
    - lightweight roll/pitch/yaw attitude estimation
    - 6-DOF operation when magnetometer is absent
    - direct comparison against current MPU9250::mahonyUpdate()
*/

#pragma once
#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

#include "AHRSCommon.h"

class MahonyAHRS {
public:
    MahonyAHRS();

    void reset();
    void setGains(float kp, float ki);
    void setQuaternion(float q0, float q1, float q2, float q3);

    float kp() const { return _kp; }
    float ki() const { return _ki; }

    // Update using AHRSInput. Uses magnetometer only when input.magValid is true.
    bool update(const AHRSInput& in, float dt, AttitudeEstimate& out);

    // Convenience update for your existing MPU_SensorData-style variables.
    bool update(float ax_g, float ay_g, float az_g,
                float gx_dps, float gy_dps, float gz_dps,
                float mx_uT, float my_uT, float mz_uT,
                bool magValid,
                float dt,
                AttitudeEstimate& out);

private:
    float _q0, _q1, _q2, _q3;
    float _ix, _iy, _iz;
    float _kp;
    float _ki;
};

#endif // MAHONY_AHRS_H
