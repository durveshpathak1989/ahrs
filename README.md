# Test Quad AHRS Library

## Explain It Simply

This module is the drone's balance sense. It takes numbers from the motion sensor and turns them into simple answers: how much the drone is leaning left or right, nose up or down, and which way it is pointing. The flight controller uses those answers to keep the drone level.

AHRS means Attitude and Heading Reference System. This library contains the common attitude data structures plus three standalone estimators used by the flight controller: `MahonyAHRS`, `MadgwickAHRS`, and `RollPitchEKF`. The production firmware can also select `AttitudeEKF`, which lives in the sibling `EKF` repository but uses the same `AHRSInput` and `AttitudeEstimate` types.

## Files

| File | Purpose |
| --- | --- |
| `AHRSCommon.h` | Shared input/output structs, angle wrapping helpers, quaternion-to-Euler conversion, accel tilt helpers. |
| `MahonyAHRS.h/.cpp` | Quaternion complementary AHRS with proportional/integral gyro correction. |
| `MadgwickAHRS.h/.cpp` | Gradient-descent IMU attitude filter. The current implementation uses accel + gyro first for robustness. |
| `RollPitchEKF.h/.cpp` | Lightweight 4-state roll/pitch estimator for bench comparison and angle-mode testing. |

## Pin Map

AHRS does not drive pins directly. It consumes calibrated IMU samples produced by the IMU driver. In the main `RC_FlightController.ino`, those samples come from this hardware map:

| Signal | ESP32 pin | Notes |
| --- | ---: | --- |
| SPI SCK | GPIO 5 | MPU-9250/MPU-6500 clock |
| SPI MISO | GPIO 19 | MPU data to ESP32 |
| SPI MOSI | GPIO 18 | ESP32 data to MPU |
| MPU CS | GPIO 33 | Chip select passed to `MPU9250 imu(PIN_MPU_CS)` |
| MPU INT | GPIO 27 | Optional data-ready interrupt; current firmware does not require it |
| Motor FL | GPIO 25 | Front-left ESC signal |
| Motor FR | GPIO 15 | Front-right ESC signal |
| Motor RL | GPIO 14 | Rear-left ESC signal |
| Motor RR | GPIO 32 | Rear-right ESC signal |
| iBUS RX | GPIO 16 | FS-iA6B iBUS TX into ESP32 UART2 RX |
| iBUS TX | GPIO 4 | Spare UART TX; avoids GPIO17 GPS conflict |
| I2C SDA | GPIO 21 | BMP280 and VL53L4CX ToF bus |
| I2C SCL | GPIO 22 | BMP280 and VL53L4CX ToF bus |
| GPS RX | GPIO 13 | GPS TXD into ESP32 UART1 RX |
| GPS TX | GPIO 17 | Optional GPS RXD from ESP32 UART1 TX |


## Main INO Integration Example

Use the AHRS types in the sketch by including the estimator headers, creating one global object per estimator you want to run, and filling `AHRSInput` from the filtered IMU sample in the control loop.

```cpp
#include "AHRSCommon.h"
#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"
#include "RollPitchEKF.h"
#include "AttitudeEKF.h"   // From the sibling EKF library.

MahonyAHRS mahony;
MadgwickAHRS madgwickAHRS;
RollPitchEKF rollPitchEKF;
AttitudeEKF attitudeEKF;

void setup() {
    mahony.setGains(1.0f, 0.005f);
    madgwickAHRS.setBeta(0.08f);
    rollPitchEKF.setProcessNoise(0.0008f, 0.000001f);
    rollPitchEKF.setMeasurementNoise(0.060f);
    attitudeEKF.setProcessNoise(0.0008f, 0.000001f);
    attitudeEKF.setAccelMeasurementNoise(0.060f);
    attitudeEKF.setMagMeasurementNoise(0.200f);
}
```


In the main 400 Hz control task, copy IMU values into `AHRSInput`. Accel uses g, gyro uses degrees/second, magnetometer uses microtesla, and `dt` is seconds.

```cpp
MPU_SensorData sf;       // Filtered IMU sample from MPU9250::readScaled().
AttitudeEstimate att;    // Output used by the PID controller and telemetry.
float dt = 0.0025f;      // 400 Hz control loop.

AHRSInput in;
in.ax_g = sf.ax_g;
in.ay_g = sf.ay_g;
in.az_g = sf.az_g;
in.gx_dps = sf.gx_dps;
in.gy_dps = sf.gy_dps;
in.gz_dps = sf.gz_dps;
in.mx_uT = sf.mx_uT;
in.my_uT = sf.my_uT;
in.mz_uT = sf.mz_uT;
in.magValid = imu.isMagConnected();
```


## Filter Implementation Examples

### Full Attitude EKF

Use this as the default flight filter when the magnetometer is present or when you want gyro-bias estimation on all three axes. It estimates roll, pitch, yaw, and gyro bias.

```cpp
AttitudeEstimate ekfOut;
if (attitudeEKF.update(in, dt, ekfOut)) {
    att = ekfOut;
}
```


### RollPitchEKF

Use this for roll/pitch bench testing, angle-mode comparison, or when yaw is not trusted. Yaw is integrated and will drift without magnetometer correction.

```cpp
AttitudeEstimate rpOut;
if (rollPitchEKF.update(in, dt, rpOut)) {
    att.roll_deg = rpOut.roll_deg;
    att.pitch_deg = rpOut.pitch_deg;
    att.yaw_deg = rpOut.yaw_deg;
}
```


### Mahony

Use this when you want a simple quaternion complementary filter. The implementation accepts `MPU_SensorData` and uses 9-DOF when magnetometer data is valid, otherwise it falls back to 6-DOF.

```cpp
AttitudeEstimate mahonyOut;
if (mahony.update(sf, dt, mahonyOut)) {
    att = mahonyOut;
}
```


### Madgwick

Use this for a responsive accel/gyro quaternion estimate. The current firmware keeps this path conservative and treats it as a 6-DOF IMU filter.

```cpp
AttitudeEstimate madgwickOut;
if (madgwickAHRS.update(in, dt, madgwickOut)) {
    att = madgwickOut;
}
```


### Runtime Selection Used By `RC_FlightController.ino`

The main firmware exposes `ahrs_filter_mode`: `0=EKF`, `1=Mahony`, `2=Madgwick`.

```cpp
uint8_t ahrsMode = constrain((int)roundf(g_tuning.ahrs_filter_mode), 0, 2);

if (ahrsMode == 1) {
    mahony.update(sf, dt, att);
} else if (ahrsMode == 2) {
    madgwickAHRS.update(in, dt, att);
} else {
    attitudeEKF.update(in, dt, att);
}
```


## Why These Data Types

`float` is used for sensor values, tuning constants, quaternions, and angles because the ESP32 has hardware floating-point support and attitude math needs fractional precision. `bool magValid` keeps magnetometer trust explicit instead of inferring it from zero values. `AttitudeEstimate` stores both Euler angles and quaternion values: Euler angles are convenient for PID and telemetry, while quaternions avoid singularities inside the filters.
