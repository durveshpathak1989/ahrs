/*
  MadgwickAHRS.h
  Standalone Madgwick IMU/AHRS estimator for Test_Quad.

  The 6-DOF update is implemented and is the path you should use first
  because your current hardware often runs without a valid AK8963 magnetometer.
*/

#pragma once
#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include "AHRSCommon.h"

class MadgwickAHRS {
public:
    MadgwickAHRS();

    void reset();
    void setBeta(float beta);
    void setQuaternion(float q0, float q1, float q2, float q3);

    float beta() const { return _beta; }

    // Uses 6-DOF accel+gyro. If magValid is true, this version currently
    // still falls back to 6-DOF to avoid using untrusted mag data.
    bool update(const AHRSInput& in, float dt, AttitudeEstimate& out);

    bool updateIMU(float ax_g, float ay_g, float az_g,
                   float gx_dps, float gy_dps, float gz_dps,
                   float dt,
                   AttitudeEstimate& out);

private:
    float _q0, _q1, _q2, _q3;
    float _beta;
};

#endif // MADGWICK_AHRS_H
