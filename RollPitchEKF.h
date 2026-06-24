/*
  RollPitchEKF.h
  Small roll/pitch EKF-style estimator for Test_Quad.

  Honest scope:
    - This is NOT a full drone navigation EKF.
    - It estimates roll, pitch, gyro-x bias, gyro-y bias.
    - It uses accel-derived roll/pitch as measurements.
    - Yaw is integrated from gz and will drift without magnetometer.
    - This is useful for bench comparison and Angle-mode roll/pitch testing.

  State:
    x = [ roll_rad, pitch_rad, gyro_x_bias_radps, gyro_y_bias_radps ]

  Units:
    input gyro: deg/s
    input accel: g
    output: degrees
*/

#pragma once
#ifndef ROLL_PITCH_EKF_H
#define ROLL_PITCH_EKF_H

#include "AHRSCommon.h"

class RollPitchEKF {
public:
    RollPitchEKF();

    void reset();
    void setProcessNoise(float angleQ, float biasQ);
    void setMeasurementNoise(float accelAngleR);
    void setInitialUncertainty(float angleP, float biasP);

    bool update(const AHRSInput& in, float dt, AttitudeEstimate& out);

    float rollBiasDps() const  { return _bgx * AHRS_RAD_TO_DEG; }
    float pitchBiasDps() const { return _bgy * AHRS_RAD_TO_DEG; }

private:
    float _roll;
    float _pitch;
    float _yaw;
    float _bgx;
    float _bgy;

    float _covP[4][4];

    float _angleQ;
    float _biasQ;
    float _accelAngleR;

    static void _quatFromEuler(float roll, float pitch, float yaw, AttitudeEstimate& out);
    void _updateOneAxis(int angleIndex, int biasIndex, float measurementRad);
};

#endif // ROLL_PITCH_EKF_H
